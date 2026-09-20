// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Package uploader provides a stateful uploader for Asset-related artifacts.
package uploader

import (
	"context"
	"crypto/sha512"
	"fmt"
	"hash"
	"strings"
	"sync"
	"time"

	"intrinsic/storage/content_addressable_storage/pkg/clienthelpers"

	log "github.com/golang/glog"
	"github.com/pborman/uuid"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"

	rdpb "intrinsic/assets/data/proto/v1/referenced_data_go_proto"
	caspb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"
)

const (
	defaultIdleTimeout                = 30 * time.Second
	defaultMaxConcurrentActiveUploads = 100
	defaultFinalizedRetentionTimeout  = 5 * time.Minute
)

// UploaderOption is a functional option for configuring an Uploader.
type UploaderOption func(*Uploader)

// WithIdleTimeout sets the inactivity timeout for uploads created by the Uploader.
func WithIdleTimeout(d time.Duration) UploaderOption {
	return func(u *Uploader) {
		u.idleTimeout = d
	}
}

// WithRetentionTimeout sets the retention timeout for uploads created by the Uploader.
func WithRetentionTimeout(d time.Duration) UploaderOption {
	return func(u *Uploader) {
		u.retentionTimeout = d
	}
}

// New creates a new Uploader.
func New(client caspb.ContentAddressableStorageServiceClient, opts ...UploaderOption) *Uploader {
	u := &Uploader{
		casClient:        client,
		idleTimeout:      defaultIdleTimeout,
		retentionTimeout: defaultFinalizedRetentionTimeout,
	}
	for _, opt := range opts {
		opt(u)
	}
	return u
}

// Uploader manages stateful uploads.
type Uploader struct {
	casClient        caspb.ContentAddressableStorageServiceClient
	idleTimeout      time.Duration
	retentionTimeout time.Duration
}

// Start starts a new stateful upload.
func (u *Uploader) Start(ctx context.Context, opts ...UploadOption) (*Upload, error) {
	uploadCtx, cancel := context.WithCancel(ctx)

	casStream, err := u.casClient.Create(uploadCtx)
	if err != nil {
		cancel()
		log.Errorf("Failed to create CAS stream for upload: %v", err)
		return nil, status.Errorf(codes.Internal, "failed to create CAS stream: %v", err)
	}

	upload := &Upload{
		cancel:           cancel,
		casStream:        casStream,
		checksummer:      clienthelpers.NewChecksummer(),
		idleTimeout:      u.idleTimeout,
		retentionTimeout: u.retentionTimeout,
		shaChecksummer:   sha512.New(),
	}

	for _, opt := range opts {
		opt(upload)
	}

	upload.idleTimer = time.AfterFunc(upload.idleTimeout, func() {
		log.Warningf("Upload session timed out after %v of inactivity; aborting upload", upload.idleTimeout)
		_ = upload.Abort()
	})

	return upload, nil
}

// UploadOption is a functional option for configuring an individual Upload in Uploader.Start.
type UploadOption func(*Upload)

// WithOnExpired registers a callback invoked when the retention period expires.
func WithOnExpired(fn func()) UploadOption {
	return func(u *Upload) {
		u.onExpired = fn
	}
}

// WithOnFinalized registers a callback invoked when the upload is finalized or aborted.
func WithOnFinalized(fn func()) UploadOption {
	return func(u *Upload) {
		u.onFinalized = fn
	}
}

// Upload represents an active upload session.
type Upload struct {
	// Network streaming
	cancel    context.CancelFunc
	casStream caspb.ContentAddressableStorageService_CreateClient
	sendMu    sync.Mutex // Serializes Send and Finalize calls to CAS stream.

	// Stream position & retry deduplication
	expectedOffset    int64
	lastChunkMetadata *chunkMetadata

	// Checksumming
	checksummer    hash.Hash32
	shaChecksummer hash.Hash

	// Lifecycle state & cached result
	err       error
	finalized bool
	ref       *rdpb.ReferencedData
	stateMu   sync.Mutex // Protects lifecycle state, timer, and cached results.

	// Retention & lifecycle callbacks
	idleTimeout      time.Duration
	idleTimer        *time.Timer
	onExpired        func()
	onFinalized      func()
	retentionTimeout time.Duration
	retentionTimer   *time.Timer
}

// Send uploads a chunk of data.
//
// Enforces sequential offsets.
func (u *Upload) Send(ctx context.Context, offset int64, data []byte) error {
	u.sendMu.Lock()
	defer u.sendMu.Unlock()

	if err := u.touch(); err != nil {
		log.WarningContextf(ctx, "Upload Send rejected: session already finalized/inactive: %v", err)
		return err
	}

	chunkMetadata, err := u.newChunkMetadata(offset, data)
	if err != nil {
		log.ErrorContextf(ctx, "Upload Send failed to compute chunk metadata (offset %d, size %d): %v", offset, len(data), err)
		return err
	}

	if offset != u.expectedOffset {
		if chunkMetadata.Equal(u.lastChunkMetadata) {
			log.V(2).InfoContextf(ctx, "Ignoring duplicate chunk retry at offset %d (size %d bytes)", offset, len(data))
			return nil
		}
		log.WarningContextf(ctx, "Upload chunk offset mismatch: got offset %d, expected %d", offset, u.expectedOffset)
		return status.Errorf(codes.InvalidArgument, "offset mismatch: got %d, expected %d", offset, u.expectedOffset)
	}

	if err := u.casStream.Send(&caspb.CreateRequest{
		ChecksummedData: &caspb.ChecksummedData{
			Content: data,
			Crc32C:  proto.Uint32(chunkMetadata.crc32c),
		},
	}); err != nil {
		log.ErrorContextf(ctx, "Failed to send chunk (offset %d, size %d) to CAS stream: %v", offset, len(data), err)
		return status.Errorf(codes.Internal, "failed to send chunk to CAS: %v", err)
	}

	if _, err := u.shaChecksummer.Write(data); err != nil {
		log.ErrorContextf(ctx, "Failed to write chunk (offset %d, size %d) to SHA checksummer: %v", offset, len(data), err)
		return err
	}

	u.lastChunkMetadata = chunkMetadata
	u.expectedOffset += int64(len(data))

	return nil
}

// Finalize closes the upload, verifies the digest, and returns the ReferencedData.
//
// If the upload has already been finalized, it returns the cached result.
func (u *Upload) Finalize(ctx context.Context, expectedDigest string) (*rdpb.ReferencedData, error) {
	u.sendMu.Lock()
	defer u.sendMu.Unlock()

	if finalized, ref, err := u.getFinalState(); finalized {
		log.InfoContextf(ctx, "Upload Finalize called on already finalized session; returning cached result (err: %v)", err)
		return ref, err
	}

	defer u.cancel()

	casResp, err := u.casStream.CloseAndRecv()
	if err != nil {
		log.ErrorContextf(ctx, "Failed to close CAS stream on Finalize: %v", err)
		return u.setFinalState(nil, status.Errorf(codes.Internal, "failed to close CAS stream: %v", err))
	}

	computedHash := fmt.Sprintf("sha512:%x", u.shaChecksummer.Sum(nil))
	if expectedDigest != "" && expectedDigest != computedHash {
		var err error
		if !strings.HasPrefix(expectedDigest, "sha512:") {
			log.WarningContextf(ctx, "Upload finalization failed: unsupported digest algorithm in %q", expectedDigest)
			err = status.Errorf(codes.InvalidArgument, "unsupported digest algorithm: %q (only sha512 is supported)", expectedDigest)
		} else {
			log.WarningContextf(ctx, "Upload finalization digest mismatch: expected %s, computed %s", expectedDigest, computedHash)
			err = status.Errorf(codes.InvalidArgument, "digest mismatch: expected %s, computed %s", expectedDigest, computedHash)
		}
		return u.setFinalState(nil, err)
	}

	return u.setFinalState(&rdpb.ReferencedData{
		Data: &rdpb.ReferencedData_Reference{
			Reference: casResp.GetObjectId(),
		},
		Digest: computedHash,
	}, nil)
}

// Abort cancels the upload and cleans up all resources.
func (u *Upload) Abort() error {
	if finalized, _, err := u.getFinalState(); finalized {
		return err
	}

	defer u.cancel()

	log.Info("Upload session aborted")
	u.setFinalState(nil, status.Error(codes.Aborted, "upload session aborted"))

	return nil
}

func (u *Upload) getFinalState() (bool, *rdpb.ReferencedData, error) {
	u.stateMu.Lock()
	defer u.stateMu.Unlock()

	return u.finalized, u.ref, u.err
}

func (u *Upload) setFinalState(ref *rdpb.ReferencedData, err error) (*rdpb.ReferencedData, error) {
	u.stateMu.Lock()
	defer u.stateMu.Unlock()

	u.idleTimer.Stop()

	u.finalized = true
	u.ref = ref
	u.err = err

	if u.onFinalized != nil {
		u.onFinalized()
		u.onFinalized = nil
	}

	if u.onExpired != nil {
		onExpired := u.onExpired
		u.onExpired = nil
		u.retentionTimer = time.AfterFunc(u.retentionTimeout, onExpired)
	}

	return ref, err
}

func (u *Upload) touch() error {
	u.stateMu.Lock()
	defer u.stateMu.Unlock()

	if u.finalized {
		if u.err != nil {
			return u.err
		}
		return status.Error(codes.FailedPrecondition, "upload already finalized")
	}

	u.idleTimer.Reset(u.idleTimeout)

	return nil
}

func (u *Upload) newChunkMetadata(offset int64, data []byte) (*chunkMetadata, error) {
	u.checksummer.Reset()
	if _, err := u.checksummer.Write(data); err != nil {
		return nil, err
	}

	return &chunkMetadata{
		crc32c: u.checksummer.Sum32(),
		offset: offset,
		size:   int64(len(data)),
	}, nil
}

type chunkMetadata struct {
	crc32c uint32
	offset int64
	size   int64
}

func (m *chunkMetadata) Equal(other *chunkMetadata) bool {
	if m == nil || other == nil {
		return m == other
	}
	return *m == *other
}

// UploadsOption is a functional option for configuring the Uploads manager.
type UploadsOption func(*Uploads)

// WithMaxConcurrentUploads sets the maximum number of concurrent active uploads.
func WithMaxConcurrentUploads(limit int) UploadsOption {
	return func(u *Uploads) {
		u.maxConcurrentUploads = limit
	}
}

// Uploads manages a bounded collection of stateful uploads with active limits and retention cleanup.
type Uploads struct {
	activeCount          int
	maxConcurrentUploads int
	mu                   sync.Mutex // Protects access to the uploads map and activeCount.
	uploader             *Uploader
	uploads              map[string]*Upload
}

// NewUploads creates a new Uploads manager.
func NewUploads(uploader *Uploader, opts ...UploadsOption) *Uploads {
	u := &Uploads{
		maxConcurrentUploads: defaultMaxConcurrentActiveUploads,
		uploader:             uploader,
		uploads:              make(map[string]*Upload),
	}
	for _, opt := range opts {
		opt(u)
	}

	return u
}

// Add checks limits, starts a new upload, registers it, and returns the new upload's ID.
func (u *Uploads) Add(ctx context.Context) (string, error) {
	u.mu.Lock()
	defer u.mu.Unlock()

	if u.activeCount >= u.maxConcurrentUploads {
		log.WarningContextf(ctx, "Max concurrent active uploads limit (%d) reached", u.maxConcurrentUploads)
		return "", status.Error(codes.ResourceExhausted, "too many concurrent active uploads, try again later")
	}

	id := uuid.New()
	upload, err := u.uploader.Start(
		context.Background(),
		WithOnExpired(u.makeDeleteUpload(id)),
		WithOnFinalized(u.decrementActiveCount),
	)
	if err != nil {
		log.ErrorContextf(ctx, "Failed to start new upload in Uploads.Add: %v", err)
		return "", err
	}

	u.uploads[id] = upload
	u.activeCount++

	return id, nil
}

// Get retrieves an upload by ID.
func (u *Uploads) Get(id string) (*Upload, error) {
	u.mu.Lock()
	defer u.mu.Unlock()

	upload, ok := u.uploads[id]
	if !ok {
		log.Warningf("Upload session %q not found in active uploads map", id)
		return nil, status.Errorf(codes.NotFound, "upload %q not found", id)
	}

	return upload, nil
}

func (u *Uploads) decrementActiveCount() {
	u.mu.Lock()
	defer u.mu.Unlock()

	u.activeCount--
}

func (u *Uploads) makeDeleteUpload(id string) func() {
	return func() {
		u.mu.Lock()
		defer u.mu.Unlock()

		if _, ok := u.uploads[id]; ok {
			log.V(1).Infof("Retention expired; removing finalized upload session %q", id)
			delete(u.uploads, id)
		}
	}
}
