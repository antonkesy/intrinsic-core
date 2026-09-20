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
	"encoding/json"
	"errors"
	"fmt"
	"time"

	"intrinsic/storage/artifacts/internal/utils"

	"github.com/containerd/containerd"
	"github.com/containerd/containerd/content"
	"github.com/containerd/containerd/images"
	"github.com/containerd/containerd/namespaces"
	"github.com/containerd/containerd/platforms"
	log "github.com/golang/glog"
	"github.com/opencontainers/go-digest"
	ocispec "github.com/opencontainers/image-spec/specs-go/v1"

	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

func newContainerDBackend(_ context.Context, options *backendOptions) (*containerDBackend, error) {
	client, err := containerd.New(options.address,
		containerd.WithDefaultNamespace(options.namespace))
	if err != nil {
		return nil, fmt.Errorf("cannot create containerd client: %w", err)
	}
	return &containerDBackend{
		backendOptions: *options,
		client:         client,
	}, nil
}

type containerDBackend struct {
	backendOptions
	client *containerd.Client
}

func (c *containerDBackend) CheckImage(ctx context.Context, request *artifactpb.ImageRequest) (*artifactpb.ArtifactResponse, error) {
	response := new(artifactpb.ArtifactResponse)
	response.Ref = request.Name

	var descriptor ocispec.Descriptor
	if request.Manifest != nil {
		// user provided desired descriptor. Let's see if we can make this work
		descriptor = utils.AsDescriptor(request.Manifest)
	} else {
		// todo b/327799134: Very strange behavior here.
		log.WarningContextf(ctx, "caller didn't provide image manifest to check (%s)", request.Name)
	}
	ctx = namespaces.WithNamespace(ctx, c.namespace)
	// error in this context may or may not indicate caller reportable error.
	image, err := c.client.ImageService().Get(ctx, request.Name)
	if err != nil {
		err = errors.Unwrap(err)
		// I wish this would be a bit more robust :(
		if err.Error() == "not found" {
			// we do not have an image matching given name, but we still may have its content
			response.Available = false
		} else {
			return nil, err
		}
	} else {
		// we found the image ... do we have all its content?
		// content to the containerd store can be essentially uploaded in random
		// order as image layers are independent blobs
		descriptor, err = c.findManifest(ctx, image)
		response.Available = err == nil // this is temporary truth
		response.Digest = utils.ToPointer(descriptor.Digest.String())
	}

	// we are going to check if we have all the image parts for our platform only.
	available, _, present, missing, err := images.Check(ctx, c.client.ContentStore(), descriptor, platforms.Default())
	if err != nil {
		return nil, err
	}

	// if response.Available is false, this indicates that we have content under
	// different name. So user still needs to publish image even if no content
	// is needed to be transferred.
	response.Available = response.Available && available
	response.MissingRefs = utils.AsRefArray(missing...)
	response.PresentRefs = utils.AsRefArray(present...)

	return response, nil
}

func (c *containerDBackend) GetInfo(ctx context.Context, ref string) (content.Info, error) {
	return c.client.ContentStore().Info(ctx, digest.Digest(ref))
}

func (c *containerDBackend) CheckStatus(ctx context.Context) error {
	if c.client != nil {
		timeoutCtx, cancelFx := context.WithTimeout(ctx, 1*time.Second)
		defer cancelFx()
		serving, err := c.client.IsServing(timeoutCtx)
		if err != nil && !serving {
			err = c.client.Reconnect()
		}
		return err
	}
	return fmt.Errorf("nil client")
}

func (c *containerDBackend) Updater(ctx context.Context, request *artifactpb.UpdateRequest) (ContentUpdater, error) {
	ctx = namespaces.WithNamespace(ctx, c.namespace)
	return newContentUpdater(ctx, c, request)
}

func (c *containerDBackend) ContainerdClient() *containerd.Client {
	return c.client
}

func (c *containerDBackend) EphemeralFileStore() string {
	return c.objectStoreRoot
}

func (c *containerDBackend) findManifest(ctx context.Context, image images.Image) (ocispec.Descriptor, error) {
	// we will always return this if we run into problems to return manifest
	// this may look uncommon, but we have image, so we just need to return
	// its digest, so we prefer image digest over nothing
	target := image.Target
	if images.IsManifestType(target.MediaType) {
		return target, nil
	}

	if images.IsIndexType(target.MediaType) {
		idxBlob, err := content.ReadBlob(ctx, c.client.ContentStore(), target)
		if err != nil {
			return target, err
		}
		var idx ocispec.Index
		if err = json.Unmarshal(idxBlob, &idx); err != nil {
			return target, err
		}

		for _, m := range idx.Manifests {
			if m.Platform == nil || platforms.Default().Match(*m.Platform) {
				return m, nil
			}
		}
	}
	// this indicates that we didn't find manifest matching the platform.
	return target, errors.New("manifest not found")
}
