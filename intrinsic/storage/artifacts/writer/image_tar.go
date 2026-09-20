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
	"os"
	"path/filepath"

	"github.com/containerd/containerd"
	"github.com/containerd/containerd/content"
	"github.com/containerd/containerd/platforms"
	log "github.com/golang/glog"
	"github.com/opencontainers/go-digest"
	"github.com/pborman/uuid"
	"go.opencensus.io/trace"

	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

type imageTarWriter struct {
	store      ArtifactStore
	gzipWriter *fileWriter
	origin     *artifactpb.UpdateRequest
	ctx        context.Context
	filename   string
}

func newImageTarWriter(ctx context.Context, store ArtifactStore, req *artifactpb.UpdateRequest) (UpdateWriter, error) {
	filename := uuid.New() // use globally unique ID as internal filename
	// original request contains image name, we don't want to use that for filename
	fileReq := &artifactpb.UpdateRequest{
		Ref:            filename,
		MediaType:      req.MediaType,
		Action:         req.Action,
		ChunkId:        req.ChunkId,
		Length:         req.Length,
		Data:           req.Data,
		ExpectedDigest: req.ExpectedDigest,
	}
	writer, err := newFileWriter(ctx, store, fileReq)
	if err != nil {
		return nil, fmt.Errorf("cannot create ephemeral writer: %w", err)
	}
	gzipWriter, ok := writer.(*fileWriter)
	if !ok {
		return nil, fmt.Errorf("unexpected writer type; got %T", writer)
	}

	imageWriter := &imageTarWriter{
		store:      store,
		gzipWriter: gzipWriter,
		origin:     req,
		ctx:        ctx,
	}

	return imageWriter, nil
}

func (i *imageTarWriter) Write(p []byte) (n int, err error) {
	return i.gzipWriter.Write(p)
}

func (i *imageTarWriter) Close() error {
	return i.gzipWriter.Close()
}

func (i *imageTarWriter) Status() (content.Status, error) {
	status, err := i.gzipWriter.Status()
	status.Ref = i.origin.Ref
	return status, err
}

func (i *imageTarWriter) Commit(size int64, expected digest.Digest) error {
	ctx, span := trace.StartSpan(i.ctx, "imageTarWriter.Commit")
	defer span.End()
	// to continue the span, let's force this onto our writer
	oldWriterCtx := i.gzipWriter.ctx
	i.gzipWriter.ctx = ctx
	defer func() {
		i.gzipWriter.ctx = oldWriterCtx
	}()

	if err := i.gzipWriter.Commit(size, expected); err != nil {
		return fmt.Errorf("write of image file failed: %w", err)
	}

	filename := filepath.Join(i.store.EphemeralFileStore(), i.gzipWriter.reference)
	tarFile, err := os.Open(filename)
	if err != nil {
		return fmt.Errorf("cannot read image from file: %w", err)
	}
	defer os.Remove(filename) // there is no reasonable path how to reuse this file

	if ctx.Err() != nil {
		// the context were closed; we are not going to continue with expensive operation
		return fmt.Errorf("skipping image import: %w", ctx.Err())
	}

	ociClient := i.store.ContainerdClient()
	importedImages, err := ociClient.Import(ctx, tarFile, containerd.WithIndexName(i.origin.Ref),
		containerd.WithImportPlatform(platforms.DefaultStrict()))
	if err != nil {
		return fmt.Errorf("cannot import image to runtime: %w", err)
	}

	for _, img := range importedImages {
		log.InfoContextf(oldWriterCtx, "image imported: %s > %s\n", img.Name, img.Target.Digest)
	}
	return nil
}

func (i *imageTarWriter) Abort() error {
	return i.gzipWriter.Abort()
}

func (i *imageTarWriter) Digest() digest.Digest {
	return i.gzipWriter.Digest()
}
