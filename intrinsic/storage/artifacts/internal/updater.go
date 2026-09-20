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

package internal

import (
	"context"
	"fmt"
	"io"
	"time"

	"intrinsic/storage/artifacts/internal/utils"
	"intrinsic/storage/artifacts/writer/writer"

	"github.com/containerd/containerd/content"
	"github.com/containerd/containerd/errdefs"
	log "github.com/golang/glog"
	"github.com/opencontainers/go-digest"
	ocispec "github.com/opencontainers/image-spec/specs-go/v1"
	"github.com/pkg/errors"
	"go.uber.org/atomic"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

// ErrUpdaterAlreadyFinalized indicates update operation on already closed updater
var ErrUpdaterAlreadyFinalized = errors.New("updater already finalized")

// contentUpdater updates local containerd storage directly using its API
type contentUpdater struct {
	reference string
	lastChunk int64
	total     int64
	updatedAt time.Time
	writer    writer.UpdateWriter
	finalized *atomic.Bool
}

func newContentUpdater(ctx context.Context, store writer.ArtifactStore, request *artifactpb.UpdateRequest) (*contentUpdater, error) {
	if checkBlobExists(ctx, store.ContainerdClient().ContentStore(), request) {
		// we already have this blob present, we are going to short circuit here
		// to not waste resources. We already probably got some data flowing here
		// but whatever, better than uploading everything and just be ignored
		// when commit happens. Client can send small probing request to prevent
		// wasteful uploads. See b/327799134
		return nil, status.Errorf(codes.AlreadyExists, "%s already exists", utils.AsShortName(request.Ref))
	}

	writer, err := writer.NewWriter(ctx, store, request)
	if err != nil {
		return nil, status.Errorf(codes.Unavailable, "cannot create updater: %s", err)
	}
	updater := &contentUpdater{
		reference: request.Ref,
		lastChunk: -1,
		total:     0,
		finalized: atomic.NewBool(false),
		writer:    writer,
	}
	return updater, nil
}

func (c *contentUpdater) IsFinalized() bool {
	return c.finalized.Load()
}

func (c *contentUpdater) Update(ctx context.Context, request *artifactpb.UpdateRequest) (*artifactpb.UpdateResponse, error) {
	if c.finalized.Load() {
		log.WarningContextf(ctx, "Update called on finalized updater. req: %v", request)
		return nil, status.Errorf(codes.Aborted, "operation aborted: %s [%d] %v", utils.AsShortName(request.Ref), request.ChunkId, ErrUpdaterAlreadyFinalized)
	}
	if request.Ref != "" && request.Ref != c.reference {
		// this is server programmer error. This should be rare.
		// we do not require user to send reference after first update
		return nil, errors.New("invalid reference update, reference mismatch")
	}
	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_UNDEFINED {
		return nil, status.Error(codes.InvalidArgument, "undefined update action")
	}

	log.Infof("[%s]: update request action: %q", utils.AsShortName(request.Ref), request.Action)

	// this action can be performed without increasing the chunkId.
	// this is to allow for asynchronous status checks.
	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_STAT {
		return c.actionStat(request)
	}

	if c.lastChunk >= request.ChunkId {
		return nil, status.Errorf(codes.Aborted,
			"[%s]: repeated update request: request %d, last seen %d", utils.AsShortName(request.Ref), request.ChunkId, c.lastChunk)
	}

	c.lastChunk = request.ChunkId
	if request.Length > 0 {
		written, err := c.writer.Write(request.Data[:request.Length])
		if err != nil && !errors.Is(err, io.EOF) {
			log.WarningContextf(ctx, "[%s:%5d]: error from writer: %v", utils.AsShortName(request.Ref), request.ChunkId, err)
			return nil, status.Errorf(codes.Aborted, "[%s]: error writing chunk %d to storage: %v", utils.AsShortName(request.Ref), request.ChunkId, err)
		}
		c.total += int64(written)
		log.Infof("[%s:%5d]: added %d of %d bytes (total of %d) to ref: %s", utils.AsShortName(request.Ref), request.ChunkId, written, request.Length, c.total, request.Ref)
	}

	response, err := c.actionStat(request)
	if err != nil {
		if !errors.Is(err, io.EOF) {
			return nil, fmt.Errorf("cannot stat action %s: %w", request.Action, err)
		}
	}
	log.Infof("[%s:%5d]: writer stat: %d", utils.AsShortName(request.Ref), request.ChunkId, response.Total)
	switch request.Action {
	case artifactpb.UpdateAction_UPDATE_ACTION_UPDATE:
		log.Infof("[%s]: update writer %q", utils.AsShortName(request.Ref), c.reference)
		return response, err
	case artifactpb.UpdateAction_UPDATE_ACTION_COMMIT:
		log.Infof("[%s]: commit writer %q", utils.AsShortName(request.Ref), c.reference)
		c.finalized.Store(true)
		digestStr := ""
		if request.ExpectedDigest != nil {
			digestStr = *request.ExpectedDigest
		}
		// commit closes the writer as well
		err = c.writer.Commit(c.total, digest.Digest(digestStr))
		if err != nil && !errdefs.IsAlreadyExists(err) {
			return nil, err
		}
		response.ExpectedDigest = utils.AsStringDigest(c.writer.Digest())
		return response, nil
	case artifactpb.UpdateAction_UPDATE_ACTION_ABORT:
		log.Infof("[%s]: abort writer %q", utils.AsShortName(request.Ref), c.reference)
		return response, c.Abort()
	}

	return nil, fmt.Errorf("unknown action %v", request.Action)
}

func (c *contentUpdater) Abort() (err error) {
	c.finalized.Store(true)
	defer c.Close()
	err = c.writer.Abort()
	return err
}

func (c *contentUpdater) actionStat(request *artifactpb.UpdateRequest) (*artifactpb.UpdateResponse, error) {
	status, err := c.writer.Status()
	if err != nil {
		return nil, fmt.Errorf("cannot stat: %w", err)
	}
	return actionStatResponse(status, request), nil
}

func (c *contentUpdater) Close() error {
	if !c.finalized.Load() {
		log.Warningf("closing unfinished updater for %s", c.reference)
		panic(fmt.Sprintf("closing unfinished updater for %s", c.reference))
	}
	return c.writer.Close()
}

func actionStatResponse(status content.Status, request *artifactpb.UpdateRequest) *artifactpb.UpdateResponse {
	return &artifactpb.UpdateResponse{
		Ref:       status.Ref,
		Action:    &request.Action,
		ChunkId:   request.ChunkId,
		Total:     &status.Offset,
		UpdatedAt: utils.AsTimestamp(status.UpdatedAt),
	}
}

func checkBlobExists(ctx context.Context, cs content.Store, request *artifactpb.UpdateRequest) bool {
	reqDig := digest.Digest(request.Ref)
	if err := reqDig.Validate(); err != nil {
		// this is not valid digest, this indicates we are uploading manifest,
		// and we want to make sure we always write manifest.
		return false
	}
	reader, err := cs.ReaderAt(ctx, ocispec.Descriptor{
		MediaType: request.MediaType,
		Digest:    reqDig,
	})
	if err != nil {
		// we do not have a blob for this digest. The error could be of different
		// type, but we are going to let this fail later on.
		return false
	}
	reader.Close() // to be good citizen
	return true
}
