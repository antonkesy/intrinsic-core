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

// Package internal provides internal implementation of artifacts service
package internal

import (
	"context"
	"fmt"
	"io"
	"net/http"

	"intrinsic/storage/artifacts/internal/utils"

	"github.com/containerd/containerd/content"
	"github.com/google/go-containerregistry/pkg/authn"
	"github.com/google/go-containerregistry/pkg/v1/google"
	"github.com/google/go-containerregistry/pkg/v1/remote"

	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

type backendStorageType int

const (
	storageContainerD backendStorageType = iota // local containerd backend
	storageRegistry                             // remote OCI compatible registry
)

type backendOptions struct {
	backendStorage     backendStorageType
	address            string
	namespace          string
	objectStoreRoot    string
	useClusterIdentity bool
	username           string
	password           string
	transport          http.RoundTripper
}

// BackendOption represents option for setting up ContentBackend implementations.
type BackendOption = utils.AnOption[*backendOptions]

// WithBackendContainerD sets NewBackend factory method to create containerd
// backend for storing images locally in host containerd storage.
func WithBackendContainerD(containerdAddress string, objectStoreRoot string) BackendOption {
	return func(opts *backendOptions) error {
		opts.backendStorage = storageContainerD
		opts.address = containerdAddress
		opts.objectStoreRoot = objectStoreRoot
		return nil
	}
}

// WithNamespace defines a namespace used by ContentBackend during its operations.
func WithNamespace(namespace string) BackendOption {
	return func(base *backendOptions) error {
		base.namespace = namespace
		return nil
	}
}

// WithBackendRegistry sets NewBackend factory method to create image storage
// backed by remote OCI compatible container registry.
func WithBackendRegistry(address string, useClusterIdentity bool) BackendOption {
	return func(opts *backendOptions) error {
		opts.backendStorage = storageRegistry
		opts.useClusterIdentity = useClusterIdentity
		opts.address = address
		return nil
	}
}

// WithBasicAuth sets username and password to use with remote registry
func WithBasicAuth(username, password string) BackendOption {
	return func(opts *backendOptions) error {
		opts.username = username
		opts.password = password
		return nil
	}
}

// WithTransport sets http.RoundTripper for remote registry. Useful for testing
func WithTransport(rt http.RoundTripper) BackendOption {
	return func(opts *backendOptions) error {
		opts.transport = rt
		return nil
	}
}

var defaultOptions = backendOptions{
	backendStorage: storageContainerD,
	transport:      http.DefaultTransport,
}

// ContentBackend implements specific operations on give backend, e.g: containerd.
type ContentBackend interface {
	Updater(ctx context.Context, request *artifactpb.UpdateRequest) (ContentUpdater, error)
	CheckStatus(ctx context.Context) error
	GetInfo(ctx context.Context, ref string) (content.Info, error)
	CheckImage(ctx context.Context, request *artifactpb.ImageRequest) (*artifactpb.ArtifactResponse, error)
}

// ContentUpdater is updater used by ContentBackend to perform update operations
// on individual storage objects/blobs such as image layers.
type ContentUpdater interface {
	io.Closer
	Update(ctx context.Context, request *artifactpb.UpdateRequest) (*artifactpb.UpdateResponse, error)
	Abort() error
	IsFinalized() bool
}

// NewBackend is factory method for creating a new backed based on set of BackendOption
// values. The returned object implements ContentBackend interface and is responsible
// to handle implementation details of storing objects in particular backend.
func NewBackend(ctx context.Context, opts ...BackendOption) (ContentBackend, error) {
	options := defaultOptions
	if err := utils.ApplyOptions(&options, opts...); err != nil {
		return nil, fmt.Errorf("cannot apply options: %w", err)
	}
	switch options.backendStorage {
	case storageContainerD:
		return newContainerDBackend(ctx, &options)
	case storageRegistry:
		authorizer := remote.WithAuth(authn.Anonymous)
		if options.useClusterIdentity {
			authorizer = remote.WithAuthFromKeychain(google.Keychain)
		} else if options.username != "" {
			authorizer = remote.WithAuth(authn.FromConfig(authn.AuthConfig{
				Username: options.username,
				Password: options.password,
			}))
		}
		return newRegistryBackend(ctx, &options, authorizer, remote.WithTransport(options.transport))
	default:
		return nil, fmt.Errorf("unsupported backend type: %v", options.backendStorage)
	}
}
