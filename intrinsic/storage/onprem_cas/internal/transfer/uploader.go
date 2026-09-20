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

package transfer

import (
	"context"
	"fmt"
	"io"
	"time"

	"intrinsic/storage/content_addressable_storage/pkg/idhandling"
	caspb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"

	"log/slog"

	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

// UploadResult represents the outcome of a successful upload operation.
type UploadResult struct {
	// Digest is the CAS digest (SHA-512) of the uploaded object.
	Digest string
	// Size is the size of the uploaded object in bytes.
	Size uint64
	// Err is set if the async operation failed.
	Err error
}

// Uploader manages the concurrent uploading of local objects to an upstream CAS service.
type Uploader struct {
	upstreamClient caspb.ContentAddressableStorageServiceClient
	localClient    caspb.ContentAddressableStorageServiceClient
	dualStatter    *dualStatter
	timeout        Timeout
}

// NewUploader creates a new uploader using the provided CAS clients.
func NewUploader(
	upstreamClient caspb.ContentAddressableStorageServiceClient,
	localClient caspb.ContentAddressableStorageServiceClient,
	dualStatter *dualStatter,
	cfg Timeout,
) *Uploader {
	if cfg.MinSpeedBps == 0 {
		cfg.MinSpeedBps = 1
	}
	return &Uploader{
		upstreamClient: upstreamClient,
		localClient:    localClient,
		dualStatter:    dualStatter,
		timeout:        cfg,
	}
}

// Upload pushes the local object associated with the given digest to the upstream CAS service.
// It skips the upload if the object already exists upstream.
func (u *Uploader) Upload(ctx context.Context, digest string) (*UploadResult, error) {
	objectID, err := idhandling.AddSchema(digest) // performs also validation
	if err != nil {
		return nil, fmt.Errorf("invalid digest: %w", err)
	}

	// Determine size and upstream/local status of the object.
	statRes, err := u.dualStatter.stat(ctx, digest)
	if err != nil {
		return nil, fmt.Errorf("stat failed: %w", err)
	}

	if statRes.Upstream {
		return &UploadResult{Digest: digest, Size: statRes.Size}, nil
	}

	if !statRes.Locally {
		return nil, status.Errorf(codes.NotFound, "object %q not found locally for upload", digest)
	}

	recordAttempt(ctx, OpUpload)

	trackActiveOperations(ctx, OpUpload, 1)
	defer trackActiveOperations(ctx, OpUpload, -1)

	start := time.Now()

	read, err := u.uploadImpl(ctx, objectID, statRes.Size)
	duration := time.Since(start)
	if err != nil {
		recordFailed(ctx, OpUpload)
		slog.Error("Upload failed", slog.String("digest", digest), slog.Int64("duration_millis", duration.Milliseconds()), slog.Uint64("bytes_transferred", read), slog.Any("error", err))
		return nil, err
	}

	recordTransferredBytes(ctx, OpUpload, int64(read))
	recordCompleted(ctx, OpUpload)
	slog.Info("Upload succeeded", slog.String("digest", digest), slog.Int64("duration_millis", duration.Milliseconds()), slog.Uint64("bytes_transferred", read))
	if u.dualStatter != nil {
		u.dualStatter.RecordUpload(digest, read)
	}
	return &UploadResult{Digest: digest, Size: read}, nil
}

func (u *Uploader) uploadImpl(ctx context.Context, objectID string, size uint64) (uint64, error) {
	timeout := u.timeout.Effective(size)

	runCtx, cancel := context.WithTimeout(ctx, timeout)
	defer cancel()

	getStream, err := u.localClient.Get(runCtx, &caspb.GetRequest{ObjectId: objectID})
	if err != nil {
		return 0, fmt.Errorf("local get failed: %w", err)
	}

	createStream, err := u.upstreamClient.Create(runCtx)
	if err != nil {
		return 0, fmt.Errorf("upstream create failed: %w", err)
	}

	var read uint64
	for {
		res, err := getStream.Recv()
		if err == io.EOF {
			break
		}
		if err != nil {
			recordWastedBytes(ctx, OpUpload, int64(read))
			return read, fmt.Errorf("local recv failed: %w", err)
		}

		err = createStream.Send(&caspb.CreateRequest{
			ChecksummedData: res.ChecksummedData,
		})
		if err != nil {
			recordWastedBytes(ctx, OpUpload, int64(read))
			return read, fmt.Errorf("upstream send failed: %w", err)
		}
		read += uint64(len(res.ChecksummedData.GetContent()))
	}

	uploadedResponse, err := createStream.CloseAndRecv()
	if err != nil {
		return read, fmt.Errorf("upstream close and recv failed: %w", err)
	}
	if uploadedResponse.GetObjectId() != objectID {
		recordWastedBytes(ctx, OpUpload, int64(read))
		return read, fmt.Errorf("hash mismatch: got %q, want %q", uploadedResponse.GetObjectId(), objectID)
	}

	return read, nil
}
