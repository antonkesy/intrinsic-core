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
	"bytes"
	"context"
	"fmt"
	"io"
	"net/http"
	"strings"
	"time"

	"intrinsic/storage/artifacts/internal/registry/registry"
	"intrinsic/storage/artifacts/internal/utils"

	"github.com/containerd/containerd/content"
	log "github.com/golang/glog"
	"github.com/google/go-containerregistry/pkg/name"
	v1 "github.com/google/go-containerregistry/pkg/v1"
	"github.com/google/go-containerregistry/pkg/v1/partial"
	"github.com/google/go-containerregistry/pkg/v1/remote"
	"github.com/google/go-containerregistry/pkg/v1/remote/transport"
	"github.com/google/go-containerregistry/pkg/v1/types"
	ocidigest "github.com/opencontainers/go-digest"
	"github.com/pkg/errors"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

// Ensuring these conform to ContentUpdater interface, while preventing
// circular dependencies.
var (
	_ ContentUpdater = (*registry.MemoryLayer)(nil)
	_ ContentUpdater = (*registry.MemoryManifest)(nil)

	ErrRepositoryMismatch = errors.Errorf("repository not found")
)

type ociRegistry struct {
	backendOptions
	remoteOptions []remote.Option
	baseRegistry  name.Repository
}

func newRegistryBackend(ctx context.Context, options *backendOptions, remoteOpts ...remote.Option) (ContentBackend, error) {
	repository, err := name.NewRepository(options.address)
	if err != nil {
		return nil, fmt.Errorf("cannot parse address: %w", err)
	}

	result := &ociRegistry{
		backendOptions: *options,
		remoteOptions:  remoteOpts,
		baseRegistry:   repository,
	}

	return result, result.validatePermissions(ctx)
}

func (o *ociRegistry) makeOptions(ctx context.Context, opts ...remote.Option) []remote.Option {
	return append(append(o.remoteOptions, opts...), remote.WithContext(ctx))
}

func (o *ociRegistry) Updater(ctx context.Context, request *artifactpb.UpdateRequest) (ContentUpdater, error) {
	mediaType := types.MediaType(request.MediaType)
	if mediaType.IsIndex() {
		// TODO (@rkomara): do we need to add support for indexes?
		return nil, fmt.Errorf("image indexes are not supported, please upload single manifest only")
	}

	// find image name in appropriate format
	imageName := request.Ref
	if ocidigest.DigestRegexpAnchored.MatchString(imageName) {
		var err error
		imageName, err = utils.ImageNameFromDescriptor(request.Content)
		if err != nil {
			return nil, err
		}
	}
	// Making uploader for manifest
	imgRef, err := o.parseAsReference(imageName)
	if err != nil {
		return nil, err
	}

	if mediaType.IsImage() {
		return registry.NewManifestUpdater(ctx, request, imgRef, o.remoteOptions...)
	}
	if mediaType.IsConfig() || mediaType.IsDistributable() {
		// we upload every distributable as binary blob
		return registry.NewLayerUpdater(ctx, request, imgRef.Context(), o.remoteOptions...)
	}
	return nil, fmt.Errorf("unsupported media type %q", mediaType)
}

func (o *ociRegistry) parseAsReference(imageName string) (name.Reference, error) {
	imgRef, err := name.ParseReference(imageName)
	if err != nil {
		return nil, fmt.Errorf("cannot parse %q: %w", imageName, err)
	}
	if !strings.HasPrefix(imgRef.String(), o.baseRegistry.String()) {
		return nil, ErrRepositoryMismatch
	}
	return imgRef, nil
}

func (o *ociRegistry) validatePermissions(ctx context.Context) error {
	_, err := remote.CatalogPage(o.baseRegistry.Registry, "", 1, o.makeOptions(ctx)...)
	return err
}

func (o *ociRegistry) CheckStatus(_ context.Context) error {
	return nil
}

func (o *ociRegistry) GetInfo(ctx context.Context, ref string) (content.Info, error) {
	digest := o.baseRegistry.Digest(ref)
	result, err := remote.Get(digest, o.makeOptions(ctx)...)
	if err != nil {
		return content.Info{}, fmt.Errorf("cannot check ref %q: %w", ref, err)
	}
	return content.Info{
		Digest:    ocidigest.Digest(result.Digest.String()),
		Size:      result.Size,
		CreatedAt: time.Time{}, // we are returning time zero as we don't know this information
		UpdatedAt: time.Time{},
		Labels:    result.Annotations,
	}, nil
}

func (o *ociRegistry) CheckImage(ctx context.Context, request *artifactpb.ImageRequest) (*artifactpb.ArtifactResponse, error) {
	reference, err := o.parseAsReference(request.Name)
	if err != nil {
		return nil, status.Error(codes.InvalidArgument, err.Error())
	}

	response := &artifactpb.ArtifactResponse{
		Ref:       reference.String(),
		Available: false,
	}

	hasDetails := request.Manifest != nil && len(request.Manifest.Data) > 0
	imgFound, err := remote.Image(reference, o.makeOptions(ctx)...)
	if err != nil {
		if tErr, ok := err.(*transport.Error); ok {
			if tErr.StatusCode == http.StatusNotFound {
				if !hasDetails {
					// we got 404 from registry, and we don't have any further details
					return response, nil
				}
				reader := bytes.NewReader(request.Manifest.Data)
				return o.checkImageLayers(ctx, reader, response, reference), nil
			}
		}
		return nil, fmt.Errorf("cannot read image %q: %w", reference, err)
	}
	response.Available = true
	if digest, err := imgFound.Digest(); err == nil { // if we get error, let's ignore.
		response.Digest = utils.ToPointer(digest.String())
	}

	if hasDetails {
		reader := bytes.NewReader(request.Manifest.Data)
		response = o.checkImageLayers(ctx, reader, response, reference)
		response.Available = response.Available && len(response.MissingRefs) == 0
	}

	return response, nil
}

func (o *ociRegistry) checkImageLayers(ctx context.Context, reader io.Reader, response *artifactpb.ArtifactResponse, reference name.Reference) *artifactpb.ArtifactResponse {
	// to minimize amount of traffic user sends our way, we are going to
	// check the manifest if we already have some blobs
	manifest, err := v1.ParseManifest(reader)
	if err != nil {
		// could not parse this manifest, let's log error and move on.
		log.WarningContextf(ctx, "cannot parse manifest: %s", err)
		return response
	}
	descriptors := append(manifest.Layers, manifest.Config)
	for _, desc := range descriptors {
		digestStr := desc.Digest.String()
		layer, err := remote.Layer(reference.Context().Digest(digestStr), o.makeOptions(ctx)...)
		if err != nil {
			response.MissingRefs = append(response.MissingRefs, digestStr)
			continue
		}
		exists, err := partial.Exists(layer)
		if err != nil || !exists {
			response.MissingRefs = append(response.MissingRefs, digestStr)
			continue
		}
		response.PresentRefs = append(response.PresentRefs, digestStr)
	}
	return response
}
