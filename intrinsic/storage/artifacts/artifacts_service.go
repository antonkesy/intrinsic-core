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

// Package artifacts implements artifacts upload service API
package artifacts

import (
	"context"
	"fmt"
	"io"
	"net"
	"sync"
	"time"

	"intrinsic/storage/artifacts/internal/internal"
	"intrinsic/storage/artifacts/internal/utils"

	"github.com/containerd/containerd/content"
	"github.com/containerd/containerd/namespaces"
	log "github.com/golang/glog"
	"github.com/opencontainers/go-digest"
	"github.com/pkg/errors"
	"go.opencensus.io/plugin/ochttp"
	"go.opencensus.io/trace"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/metadata"
	"google.golang.org/grpc/peer"
	"google.golang.org/grpc/status"

	artifactgrpcpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

const headerIntrinsicClientID = "intrinsic-client-id"

// ServiceOptions represents runtime configuration for service
type ServiceOptions struct {
	Address         string
	Namespace       string
	MaxBlobSize     int
	MaxObjectSize   int64
	ObjectStoreRoot string
	RegistryBackend bool
	AnonymousAccess bool
	Username        string
	Password        string
}

// DefaultOptions returns reasonable service defaults
func DefaultOptions() *ServiceOptions {
	return &ServiceOptions{
		Address:         "/run/containerd/containerd.sock",
		Namespace:       namespaces.Default,
		MaxBlobSize:     5 * 1024 * 1024,
		MaxObjectSize:   512 * 1024 * 1024,
		ObjectStoreRoot: "/data/artifact_store",
	}
}

// New creates an instance of ArtifactServiceApiServer.
func New(ctx context.Context, opt *ServiceOptions) (artifactgrpcpb.ArtifactServiceApiServer, error) {
	return createService(ctx, opt)
}

func createService(ctx context.Context, opt *ServiceOptions) (*serverImpl, error) {
	var options []internal.BackendOption
	if opt.RegistryBackend {
		// remote container registry backend
		useClusterIdentity := !opt.AnonymousAccess && opt.Username == ""
		options = []internal.BackendOption{
			internal.WithBackendRegistry(opt.Address, useClusterIdentity),
			internal.WithTransport(&ochttp.Transport{}),
		}
		if opt.Username != "" {
			options = append(options, internal.WithBasicAuth(opt.Username, opt.Password))
		}
	} else {
		// containerd backend
		options = []internal.BackendOption{
			internal.WithBackendContainerD(opt.Address, opt.ObjectStoreRoot),
			internal.WithNamespace(opt.Namespace),
		}
	}
	backend, err := internal.NewBackend(ctx, options...)
	if err != nil {
		return nil, fmt.Errorf("cannot create backend: %w", err)
	}

	service := &serverImpl{
		backend:       backend,
		maxUpdateSize: int32(opt.MaxBlobSize),
		updaters:      make(map[string]*updateFinalizer, 16),
		serverCtx:     ctx,
		namespace:     opt.Namespace,
	}
	return service, service.checkStatus(ctx)
}

type serverImpl struct {
	backend       internal.ContentBackend
	maxUpdateSize int32
	updatersMu    sync.Mutex
	updaters      map[string]*updateFinalizer
	// we are keeping reference for server context here to control lifespan
	// of non-streaming updaters.
	serverCtx context.Context
	namespace string
}

func (a *serverImpl) CheckImage(ctx context.Context, request *artifactpb.ImageRequest) (*artifactpb.ArtifactResponse, error) {
	ctx, span := trace.StartSpan(ctx, "artifact_service.CheckImage")
	defer span.End()
	if err := a.checkStatus(ctx); err != nil {
		return nil, err
	}
	response, err := a.backend.CheckImage(ctx, request)
	if err != nil {
		return nil, err
	}

	response.MaxUpdateSize = a.maxUpdateSize

	return response, nil
}

func (a *serverImpl) getSessionKey(ctx context.Context, ref string) string {
	if md, ok := metadata.FromIncomingContext(ctx); ok {
		if ids := md.Get(headerIntrinsicClientID); len(ids) > 0 && ids[0] != "" {
			return ids[0] + ":" + ref
		}
	}
	if p, ok := peer.FromContext(ctx); ok {
		if host, _, err := net.SplitHostPort(p.Addr.String()); err == nil {
			return host + ":" + ref
		}
		return p.Addr.String() + ":" + ref
	}
	return ref
}

func (a *serverImpl) getOrCreateFinalizer(sessionKey string, request *artifactpb.UpdateRequest) (*updateFinalizer, error) {
	a.updatersMu.Lock()
	defer a.updatersMu.Unlock()

	finalizer, ok := a.updaters[sessionKey]
	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_STAT {
		// special handing for case where user sends STAT action on non-existent updater.
		if !ok || finalizer.IsUpdaterFinished() {
			return nil, status.Errorf(codes.FailedPrecondition, "[%s]: cannot stat, updater not open", utils.AsShortName(request.Ref))
		}
	}

	if !ok {
		finalizer = &updateFinalizer{
			ref: request.Ref,
		}
		// we are tying updates lifecycle to server context, not to request context
		// as this is request-response RPC call, thus request context is short-lived
		// and will cause updater to terminate prematurely at the end.
		updaterCtx, updaterCancelFx := context.WithTimeout(a.serverCtx, 4*time.Hour)
		finalizer.cancelFx = updaterCancelFx
		updaterCtx = namespaces.WithNamespace(updaterCtx, a.namespace)
		var err error
		finalizer.updater, err = a.backend.Updater(updaterCtx, request)
		if err != nil {
			return nil, err
		}
		a.updaters[sessionKey] = finalizer
		finalizer.monitor(updaterCtx)
	} else if finalizer.IsUpdaterFinished() {
		return nil, fmt.Errorf("[%s]: updater already finalized", utils.AsShortName(request.Ref))
	}

	return finalizer, nil
}

// removeFinalizer closes the given finalizer and removes it from the updaters map
// if it has not already been replaced by a newer session for sessionKey.
func (a *serverImpl) removeFinalizer(sessionKey string, finalizer *updateFinalizer) {
	if finalizer == nil {
		return
	}
	a.updatersMu.Lock()
	defer a.updatersMu.Unlock()

	finalizer.CloseUpdater()
	if current, ok := a.updaters[sessionKey]; ok && current == finalizer {
		delete(a.updaters, sessionKey)
	}
}

func (a *serverImpl) WriteContent(ctx context.Context, request *artifactpb.UpdateRequest) (response *artifactpb.UpdateResponse, err error) {
	ctx, span := trace.StartSpan(ctx, "artifact_service.WriteContent")
	defer span.End()
	if err = a.checkStatus(ctx); err != nil {
		return nil, err
	}

	if err = a.validateUpdateRequest(request); err != nil {
		return nil, status.Errorf(codes.InvalidArgument, "[%s]: invalid request: %v", utils.AsShortName(request.Ref), err)
	}

	sessionKey := a.getSessionKey(ctx, request.Ref)

	finalizer, err := a.getOrCreateFinalizer(sessionKey, request)
	if err != nil {
		return nil, err
	}

	response, err = finalizer.updater.Update(ctx, request)

	if finalizer.IsUpdaterFinished() || err != nil {
		if err != nil {
			log.ErrorContextf(ctx, "[%s]: error writing content (aborting writer): %v", utils.AsShortName(request.Ref), err)
		}
		a.removeFinalizer(sessionKey, finalizer)
	}

	return response, err
}

func (a *serverImpl) UploadContent(server artifactgrpcpb.ArtifactServiceApi_UploadContentServer) error {
	ctx, span := trace.StartSpan(server.Context(), "artifact_service.UploadContent")
	defer span.End()

	var updater internal.ContentUpdater
	var initRequest *artifactpb.UpdateRequest

	finalizer := new(updateFinalizer)
	defer finalizer.CloseUpdater()

	for ctx.Err() == nil {
		// Well behaving client will send at least one update message
		request, err := server.Recv()
		if err != nil {
			if err == io.EOF {
				// client side connection closure, we are done. If this is premature
				// end, finalizer will take care of that case.
				return nil
			}
			// this usually indicates that connection with client is broken.
			return fmt.Errorf("terminating, received error: %w", err)
		}
		log.InfoContextf(ctx, "[%s:%5d]: requests %s with %d bytes of data", utils.AsShortName(request.Ref), request.ChunkId, request.Action, request.Length)

		if updater == nil {
			// this is first run ...
			initRequest = request
			updater, err = a.backend.Updater(ctx, request)
			if err != nil {
				// short circuit error, return as is to caller
				return err
			}
			finalizer.ref = request.Ref
			finalizer.updater = updater
		}

		response, err := updater.Update(ctx, request)
		if err != nil {
			return fmt.Errorf("update failed for %q: %w", utils.AsShortName(request.Ref), err)
		}

		action := request.Action

		if action == artifactpb.UpdateAction_UPDATE_ACTION_COMMIT {
			digestRef, ok := findDigest(request, initRequest)
			if ok {
				info, err := a.backend.GetInfo(ctx, digestRef.String())
				if err == nil {
					// we are going to ignore error here, as we cannot really stat the thing
					response = a.responseFromInfo(request, info)
				}
			}
			if err := server.SendAndClose(response); err != nil {
				log.WarningContextf(ctx, "[%s]: error updating client: %s", utils.AsShortName(request.Ref), err)
				return err
			}
		} else if action == artifactpb.UpdateAction_UPDATE_ACTION_ABORT {
			// we are done here, this is terminal action so there is
			// nothing to do really here. Let user know we are done.
			log.InfoContextf(ctx, "terminal action %d reached, ref: %q", action, utils.AsShortName(request.Ref))
			return server.SendAndClose(response) // best effort
		}
	}

	// e.g.: SIGHUP/SIGKILL was called on app
	return fmt.Errorf("context terminated prematurely: %w", ctx.Err())
}

func (a *serverImpl) responseFromInfo(request *artifactpb.UpdateRequest, info content.Info) *artifactpb.UpdateResponse {
	return &artifactpb.UpdateResponse{
		Ref:            request.Ref,
		ChunkId:        request.ChunkId,
		Total:          &info.Size,
		UpdatedAt:      utils.AsTimestamp(info.UpdatedAt),
		Action:         &request.Action,
		ExpectedDigest: utils.AsStringDigest(info.Digest),
		MaxUpdateSize:  &a.maxUpdateSize,
	}
}

func findDigest(requests ...*artifactpb.UpdateRequest) (digest.Digest, bool) {
	for _, request := range requests {
		if result, ok := getDigest(request); ok {
			return result, true
		}
	}
	return "", false
}

func getDigest(request *artifactpb.UpdateRequest) (digest.Digest, bool) {
	result, err := digest.Parse(request.Ref)
	if err != nil {
		// reference is not a digest, let's search for content digest
		if request.ExpectedDigest != nil {
			result, err = digest.Parse(*request.ExpectedDigest)
			if err == nil {
				return result, true
			}
		}
		if request.Content != nil {
			// okey, this is one probably manifest, so ref is image name
			if request.Content.Digest != nil {
				result, err = digest.Parse(*request.Content.Digest)
				if err == nil {
					return result, true
				}
			}
		}
	}
	return result, true
}

func (a *serverImpl) checkStatus(ctx context.Context) error {
	return a.backend.CheckStatus(ctx)
}

func (a *serverImpl) validateUpdateRequest(request *artifactpb.UpdateRequest) error {
	if request.Ref == "" {
		return errors.New("missing reference to update")
	}
	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_UNDEFINED {
		return errors.New("undefined update action")
	}
	return nil
}

type updateFinalizer struct {
	ref      string
	updater  internal.ContentUpdater
	cancelFx context.CancelFunc
}

func (u *updateFinalizer) CloseUpdater() {
	if u == nil {
		return
	}
	if u.updater != nil {
		if !u.updater.IsFinalized() {
			u.updater.Abort()
		}
		u.updater.Close()
	}
	if u.cancelFx != nil {
		u.cancelFx()
	}
}

func (u *updateFinalizer) IsUpdaterFinished() bool {
	return u == nil || u.updater == nil || u.updater.IsFinalized()
}

func (u *updateFinalizer) monitor(ctx context.Context) {
	go func() {
		<-ctx.Done()
		// in case we call u.cancelFx() this is effectively no-op
		u.CloseUpdater()
	}()
}
