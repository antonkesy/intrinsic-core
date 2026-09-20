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

// Package registry provides ContentUpdaters for uploading images into OCI compatible
// remote registries.
package registry

import (
	"context"
	"fmt"
	"io"
	"sync"
	"time"

	"intrinsic/storage/artifacts/internal/utils"

	"github.com/containerd/containerd/content"
	log "github.com/golang/glog"
	"github.com/google/go-containerregistry/pkg/name"
	v1 "github.com/google/go-containerregistry/pkg/v1"
	"github.com/google/go-containerregistry/pkg/v1/partial"
	"github.com/google/go-containerregistry/pkg/v1/remote"
	"github.com/google/go-containerregistry/pkg/v1/remote/transport"
	"github.com/google/go-containerregistry/pkg/v1/types"
	"github.com/opencontainers/go-digest"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

var (
	// ErrUserAbort indicates abort action called by client.
	ErrUserAbort = errors.New("user requested abort")
	// ErrUpdaterAlreadyFinalized indicates error where client sends update requests to finalized updater.
	ErrUpdaterAlreadyFinalized = errors.New("updater already finalized")
	// ErrUnimplementedValue is error indicating that given code path is not available.
	ErrUnimplementedValue = errors.New("unimplemented method, nothing to return")
)

// MemoryLayer implements internal.ContentUpdater facilitating streaming upload of
// layer content from client to target registry.
type MemoryLayer struct {
	writer       *TimeoutWriter
	ctx          context.Context
	request      *artifactpb.UpdateRequest
	uploadErr    *atomic.Error
	finalized    *atomic.Bool
	writtenSoFar int64
	startedTime  time.Time
	wg           *sync.WaitGroup
	repository   name.Repository
	remoteOpts   []remote.Option
}

// NewLayerUpdater create new instance of ContentUpdater for updating an image layer.
func NewLayerUpdater(ctx context.Context, request *artifactpb.UpdateRequest, repository name.Repository, opts ...remote.Option) (*MemoryLayer, error) {
	remoteOptions := makeOptions(ctx, opts...)
	if err := checkLayerExists(repository.Digest(request.Ref), remoteOptions...); err != nil {
		return nil, err
	}

	memLayer := &MemoryLayer{
		ctx:          ctx,
		request:      request,
		uploadErr:    atomic.NewError(nil),
		finalized:    atomic.NewBool(false),
		writtenSoFar: 0,
		startedTime:  time.Now(),
		wg:           new(sync.WaitGroup),
		repository:   repository,
		remoteOpts:   opts,
	}

	pr, pw := NewTimeoutWriter(ctx)
	layer, err := newProxyLayer(request, pr)
	if err != nil {
		return nil, err
	}
	memLayer.writer = pw

	memLayer.wg.Go(func() {
		defer pw.Close()
		if err := remote.WriteLayer(repository, layer, remoteOptions...); err != nil {
			log.Warningf("[%s] error writing layer: %s", utils.AsShortName(request.Ref), err)
			errToReport := status.Errorf(codes.InvalidArgument, "error writing layer to remote: %s", err)
			memLayer.uploadErr.Store(errToReport)
			pr.CloseWithError(errToReport)
		}
		log.Infof("[%s] remote.WriteLayer done", utils.AsShortName(request.Ref))
	})
	// upload error could be delayed further, so this is mostly the best effort.
	return memLayer, memLayer.uploadErr.Load()
}

// Update implements corresponding method from internal.ContentUpdater.
func (m *MemoryLayer) Update(ctx context.Context, request *artifactpb.UpdateRequest) (*artifactpb.UpdateResponse, error) {
	if m.finalized.Load() {
		log.Warningf("Requesting work on finalized updater for %s", request.Ref)
		return nil, ErrUpdaterAlreadyFinalized
	}

	log.Infof("[%s:%5d]: update request action: %q", utils.AsShortName(request.Ref), request.ChunkId, request.Action)

	if m.request.Ref != request.Ref {
		return nil, errors.New("invalid reference update, reference mismatch")
	}
	if m.ctx.Err() != nil {
		return nil, fmt.Errorf("already cancelled")
	}

	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_UNDEFINED {
		return nil, errors.New("undefined update action")
	}

	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_STAT {
		return utils.ActionStatResponse(m.getStatus(), request), nil
	}

	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_ABORT {
		return nil, m.Abort()
	}

	length, err := m.writer.WriteContext(ctx, request.Data)
	if err != nil {
		log.Warningf("[%s:%d] error writing to pipe: %s", utils.AsShortName(request.Ref), request.ChunkId, err)
		return nil, fmt.Errorf("cannot write to upstream: %w", err)
	}
	if err := m.uploadErr.Load(); err != nil {
		// error from remote writer, we are going to send it to user
		log.Warningf("[%s:%d] error writing to remote: %s", utils.AsShortName(request.Ref), request.ChunkId, err)
		return nil, err
	}
	m.writtenSoFar += int64(length)

	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_COMMIT {
		m.finalized.Store(true)
		m.writer.Close()
		log.Infof("[%s:%d] wg.Wait - waiting for upload to finish ", utils.AsShortName(request.Ref), request.ChunkId)
		m.wg.Wait()
	}
	return utils.ActionStatResponse(m.getStatus(), request), err
}

func (m *MemoryLayer) getStatus() content.Status {
	return content.Status{
		Ref:       m.request.Ref,
		Offset:    m.writtenSoFar,
		Total:     *m.request.Content.Size,
		Expected:  digest.Digest(utils.ValueOrEmptyStr(m.request.ExpectedDigest)),
		StartedAt: m.startedTime,
		UpdatedAt: time.Now(),
	}
}

// Abort implements corresponding method from internal.ContentUpdater.
func (m *MemoryLayer) Abort() error {
	if m.finalized.CompareAndSwap(false, true) {
		return m.writer.Close()
	}
	return nil // we already finalized this writer, so nothing to abort here.
}

// IsFinalized implements corresponding method from internal.ContentUpdater.
func (m *MemoryLayer) IsFinalized() bool {
	return m.finalized.Load()
}

// Close implements corresponding method from internal.ContentUpdater.
func (m *MemoryLayer) Close() error {
	if m.finalized.CompareAndSwap(false, true) {
		log.WarningContextf(m.ctx, "closing unfinished writer for %s", utils.AsShortName(m.request.Ref))
		return m.writer.Close()
	}
	return nil
}

func (m *MemoryLayer) getRequestDigest(request *artifactpb.UpdateRequest) digest.Digest {
	digestStr := utils.ValueOrEmptyStr(request.ExpectedDigest)
	if digestStr == "" {
		digestStr = utils.ValueOrEmptyStr(m.request.ExpectedDigest)
		if digestStr == "" && m.request.Content != nil {
			digestStr = *m.request.Content.Digest
		}
	}
	result, err := digest.Parse(digestStr)
	if err != nil {
		return digest.Canonical.FromBytes(nil)
	}
	return result
}

func newProxyLayer(request *artifactpb.UpdateRequest, compressed *io.PipeReader) (*proxyLayer, error) {
	if request.Content == nil {
		return nil, errors.New("content description missing, UpdateRequest.Content value required")
	}
	layerDigest := utils.ValueOrEmptyStr(request.ExpectedDigest)
	if layerDigest == "" {
		layerDigest = utils.ValueOrEmptyStr(request.Content.Digest)
	}
	contentHash, err := v1.NewHash(layerDigest)
	if err != nil {
		return nil, fmt.Errorf("cannot parse content digest: %w", err)
	}

	result := &proxyLayer{
		compressed: compressed,
		hash:       contentHash,
		mediaType:  types.MediaType(request.MediaType),
		totalSize:  -1,
	}
	if request.Content.Size != nil {
		result.totalSize = *request.Content.Size
	}
	return result, nil
}

type proxyLayer struct {
	compressed io.ReadCloser
	hash       v1.Hash
	totalSize  int64
	mediaType  types.MediaType
}

func (p *proxyLayer) Digest() (v1.Hash, error) {
	return p.hash, nil
}

func (p *proxyLayer) DiffID() (v1.Hash, error) {
	return v1.Hash{}, ErrUnimplementedValue
}

func (p *proxyLayer) Compressed() (io.ReadCloser, error) {
	return p.compressed, nil
}

func (p *proxyLayer) Uncompressed() (io.ReadCloser, error) {
	return nil, ErrUnimplementedValue
}

func (p *proxyLayer) Size() (int64, error) {
	if p.totalSize < 0 {
		return p.totalSize, fmt.Errorf("unknown content size")
	}
	return p.totalSize, nil
}

func (p *proxyLayer) MediaType() (types.MediaType, error) {
	return p.mediaType, nil
}

func makeOptions(ctx context.Context, opts ...remote.Option) []remote.Option {
	// we are specifying remote.WithContext last to ensure we have the right
	// context for the call.
	return append(opts, remote.WithContext(ctx))
}

func checkLayerExists(ref name.Digest, opts ...remote.Option) error {
	remoteLayer, err := remote.Layer(ref, opts...)
	if err != nil {
		if tErr, ok := err.(*transport.Error); ok {
			if !tErr.Temporary() {
				return fmt.Errorf("transport error: %w", err)
			}
			// we encountered temporary error, while we cannot determine if blob exists
			// however, returning error to caller would not be correct action.
			// If blob exists, the subsequent update will fail.
			return nil
		}
		// this is some weird, unexpected error, let's tell caller as system
		// might be unhealthy anyway.
		return fmt.Errorf("updater error: %w", err)
	}

	exists, err := partial.Exists(remoteLayer)
	if err == nil && exists {
		return status.Errorf(codes.AlreadyExists, "%s already exists", utils.AsShortName(ref.DigestStr()))
	}
	// We cannot say if blob exists or not, let's pretend it does not.
	// caller will get subsequent error if failure was material.
	return nil
}
