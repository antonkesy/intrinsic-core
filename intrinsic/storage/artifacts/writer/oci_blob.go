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

package writer

import (
	"context"
	"fmt"
	"time"

	"github.com/containerd/containerd/content"
	"github.com/containerd/containerd/leases"
	"github.com/opencontainers/go-digest"
	"github.com/pborman/uuid"

	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

func newOciWriter(ctx context.Context, store ArtifactStore, req *artifactpb.UpdateRequest) (UpdateWriter, error) {
	ociClient := store.ContainerdClient()
	if ociClient == nil {
		return nil, fmt.Errorf("containerd client is nil")
	}

	ingestRef := "ingest-" + req.Ref + "-" + uuid.New()
	writer := &ociWriter{
		store:     ociClient.ContentStore(),
		reference: req.Ref,
		ingestRef: ingestRef,
	}
	var err error
	writer.ctx, writer.leaseRelease, err = ociClient.WithLease(ctx,
		leases.WithExpiration(4*time.Hour),
		leases.WithRandomID())
	if err != nil {
		return nil, fmt.Errorf("error getting lease: %w", err)
	}

	// Use writer.ctx (which has the attached lease) and unique ingestRef to prevent containerd lock contention/deadlocks
	writer.writer, err = writer.store.Writer(writer.ctx, content.WithRef(ingestRef))
	if err != nil {
		_ = writer.leaseRelease(writer.ctx)
		return nil, err
	}
	return writer, nil
}

type ociWriter struct {
	store        content.Store
	reference    string
	ingestRef    string
	ctx          context.Context
	leaseRelease func(context.Context) error
	writer       content.Writer
}

func (o *ociWriter) Digest() digest.Digest {
	return o.writer.Digest()
}

func (o *ociWriter) Write(p []byte) (n int, err error) {
	return o.writer.Write(p)
}

func (o *ociWriter) Close() error {
	return o.writer.Close()
}

func (o *ociWriter) Status() (content.Status, error) {
	return o.writer.Status()
}

func (o *ociWriter) Commit(_ int64, _ digest.Digest) error {
	return o.writer.Commit(o.ctx, 0, "")
}

func (o *ociWriter) Abort() error {
	if err := o.store.Abort(o.ctx, o.ingestRef); err != nil {
		return err
	}
	if o.leaseRelease != nil {
		return o.leaseRelease(o.ctx)
	}
	return nil
}
