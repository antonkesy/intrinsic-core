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
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"math"
	"time"

	"github.com/containerd/containerd"
	"github.com/containerd/containerd/content"
	"github.com/containerd/containerd/errdefs"
	"github.com/containerd/containerd/images"
	"github.com/containerd/containerd/leases"
	"github.com/containerd/containerd/reference"
	"github.com/opencontainers/go-digest"
	specs "github.com/opencontainers/image-spec/specs-go"
	ocispec "github.com/opencontainers/image-spec/specs-go/v1"
	"github.com/pborman/uuid"
	"go.uber.org/atomic"

	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

const (
	defaultLeaseDuration = 4 * time.Hour
)

func newImageWriter(ctx context.Context, store ArtifactStore, req *artifactpb.UpdateRequest) (UpdateWriter, error) {
	if req.Content == nil {
		return nil, fmt.Errorf("manifest content details not provided, ref: %s", req.Ref)
	}

	return &ociImageWriter{
		buffer:        bytes.NewBuffer(make([]byte, 0, 4096)),
		originRequest: req,
		artStore:      store,
		ctx:           ctx,
		started:       time.Now(),
		updated:       time.Now(),
		finalized:     atomic.NewBool(false),
	}, nil
}

type ociImageWriter struct {
	buffer        *bytes.Buffer
	originRequest *artifactpb.UpdateRequest
	artStore      ArtifactStore
	ctx           context.Context
	started       time.Time
	updated       time.Time
	finalized     *atomic.Bool
}

func (o *ociImageWriter) Write(p []byte) (n int, err error) {
	if o.buffer.Len()+len(p) >= math.MaxInt32 {
		return 0, ErrMaxContentSizeExceeded
	}
	o.updated = time.Now()
	return o.buffer.Write(p)
}

func (o *ociImageWriter) Close() error {
	o.buffer.Truncate(0)
	return nil
}

func (o *ociImageWriter) Status() (content.Status, error) {
	expectedDigest := ""
	if o.originRequest.ExpectedDigest != nil {
		expectedDigest = *o.originRequest.ExpectedDigest
	}
	ed, err := digest.Parse(expectedDigest)
	return content.Status{
		Ref:       o.originRequest.Ref,
		Offset:    int64(o.buffer.Len()),
		Total:     int64(o.buffer.Len()),
		Expected:  ed,
		StartedAt: o.started,
		UpdatedAt: o.updated,
	}, err
}

func (o *ociImageWriter) Commit(size int64, expected digest.Digest) error {
	if size > 0 {
		if size != int64(o.buffer.Len()) {
			return fmt.Errorf("precondition: size mismatch: got %d, expected %d", o.buffer.Len(), size)
		}
	}

	manifestBytes := o.buffer.Bytes()
	expectedDigest := o.getRequestDigest(expected)
	if expectedDigest.Validate() != nil {
		// we got expected digest, let's make sure we have correct data.
		// condition above will fail if digest is never provided to us.
		contentDigest := expectedDigest.Algorithm().FromBytes(manifestBytes)
		if contentDigest != expectedDigest {
			return fmt.Errorf("precondition: digest mismatch: got %q, expected %q", contentDigest, expectedDigest)
		}
	}

	client := o.artStore.ContainerdClient()
	lease, err := client.LeasesService().Create(o.ctx,
		leases.WithExpiration(defaultLeaseDuration),
		leases.WithRandomID())
	if err != nil {
		return fmt.Errorf("cannot obtain necessary lease: %w", err)
	}
	ctx := leases.WithLease(o.ctx, lease.ID)
	cs := client.ContentStore()

	imageName := o.originRequest.Ref
	ociImageName := imageName

	if spec, err := reference.Parse(imageName); err == nil {
		ociImageName = spec.Object
	}

	manifestDesc, err := o.writeBlob(ctx, cs, manifestBytes, o.originRequest.MediaType)
	if err != nil {
		return fmt.Errorf("cannot write manifest: %w", err)
	}

	// Create index object to tie manifests to images
	idx := ocispec.Index{
		Versioned: specs.Versioned{
			SchemaVersion: 2,
		},
		MediaType: ocispec.MediaTypeImageIndex,
		Manifests: []ocispec.Descriptor{manifestDesc},
		Annotations: map[string]string{
			ocispec.AnnotationRefName:  ociImageName,
			images.AnnotationImageName: imageName,
		},
	}

	idxDesc, err := o.writeValue(ctx, cs, &idx, idx.MediaType)
	if err != nil {
		return fmt.Errorf("cannot write index: %w", err)
	}

	handler := images.SetChildrenLabels(cs, images.ChildrenHandler(cs))
	if err = images.WalkNotEmpty(ctx, handler, idxDesc); err != nil {
		return fmt.Errorf("cannot update GC labels: %w", err)
	}

	// Create image object safely under concurrent writes
	is := client.ImageService()
	img, err := is.Get(ctx, imageName)
	if err != nil {
		if !errdefs.IsNotFound(err) {
			// this indicates broken connection or some terrible thing.
			return fmt.Errorf("error reading image store: %w", err)
		}
		// okey, we do not have image yet, let's create new one
		img = images.Image{
			Name:   imageName,
			Target: idxDesc,
		}
		img, err = is.Create(ctx, img)
		if err != nil && errdefs.IsAlreadyExists(err) {
			// Race condition: image was already created by other client uploading same image.
			// This is generally very rare in case of IPC, but may happen
			img, err = is.Get(ctx, imageName)
			if err == nil {
				img.Target = idxDesc
				img, err = is.Update(ctx, img, "target")
			}
		}
		if err != nil {
			return fmt.Errorf("cannot write image object: %w", err)
		}
	} else {
		img.Target = idxDesc
		img, err = is.Update(ctx, img, "target")
		if err != nil {
			return fmt.Errorf("cannot update image target: %w", err)
		}
	}

	// and now finally, unpack image, making it ready for containers
	clientImage := containerd.NewImage(client, img)
	if err = clientImage.Unpack(ctx, containerd.DefaultSnapshotter, containerd.WithSnapshotterPlatformCheck()); err != nil && !errdefs.IsAlreadyExists(err) {
		return fmt.Errorf("cannot unpack image %q: %w", imageName, err)
	}
	return nil
}

func (o *ociImageWriter) Abort() error {
	o.finalized.Store(true)
	return o.Close()
}

func (o *ociImageWriter) Digest() digest.Digest {
	if o.originRequest.ExpectedDigest != nil {
		return digest.Digest(*o.originRequest.ExpectedDigest)
	}
	return ""
}

func (o *ociImageWriter) writeValue(ctx context.Context, cs content.Ingester, value any, mediaType string, opts ...content.Opt) (ocispec.Descriptor, error) {
	blobBytes, err := json.Marshal(value)
	if err != nil {
		return ocispec.Descriptor{}, fmt.Errorf("cannot marshal value: %w", err)
	}

	return o.writeBlob(ctx, cs, blobBytes, mediaType, opts...)
}

func (o *ociImageWriter) getRequestDigest(expected digest.Digest) digest.Digest {
	digestStr := string(expected)
	if digestStr == "" {
		if o.originRequest.ExpectedDigest != nil {
			digestStr = *o.originRequest.ExpectedDigest
		}
	}
	result, err := digest.Parse(digestStr)
	if err != nil {
		return digest.Canonical.FromBytes(nil)
	}
	return result
}

func (o *ociImageWriter) writeBlob(ctx context.Context, cs content.Ingester, blobBytes []byte, mediaType string, opts ...content.Opt) (ocispec.Descriptor, error) {
	descriptor := ocispec.Descriptor{
		MediaType: mediaType,
		Digest:    digest.FromBytes(blobBytes),
		Size:      int64(len(blobBytes)),
	}

	ingestRef := "manifest-" + descriptor.Digest.String() + "-" + uuid.New()
	err := content.WriteBlob(ctx, cs, ingestRef, bytes.NewReader(blobBytes), descriptor, opts...)
	if err != nil {
		return ocispec.Descriptor{}, fmt.Errorf("cannot write value %q to store: %w", descriptor.Digest, err)
	}
	return descriptor, nil
}
