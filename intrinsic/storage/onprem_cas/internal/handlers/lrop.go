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

package handlers

import (
	"context"
	"errors"
	"fmt"
	"strconv"
	"strings"
	"time"

	"intrinsic/storage/onprem_cas/internal/syncset"

	lropb "cloud.google.com/go/longrunning/autogen/longrunningpb"
	statuspb "google.golang.org/genproto/googleapis/rpc/status"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/types/known/anypb"
	"google.golang.org/protobuf/types/known/emptypb"
	"google.golang.org/protobuf/types/known/timestamppb"

	ocpsgrpcpb "intrinsic/storage/onprem_cas_preload/proto/v1/ocp_service_go_proto"
	preloadsetpb "intrinsic/storage/onprem_cas_preload/proto/v1/preload_set_go_proto"
)

// CASSyncOperationsHandler implements longrunningpb.OperationsServer using a SyncSet Controller.
type CASSyncOperationsHandler struct {
	lropb.UnimplementedOperationsServer
	Controller *syncset.Controller
}

// NewCASSyncOperationsHandler creates a new CASSyncOperationsHandler backed by the given SyncSet controller.
func NewCASSyncOperationsHandler(controller *syncset.Controller) *CASSyncOperationsHandler {
	return &CASSyncOperationsHandler{
		Controller: controller,
	}
}

// parseOperationName parses a long-running operation name into the SyncSet name and generation.
func parseOperationName(opName string) (string, syncset.Generation, error) {
	if opName == "" {
		return "", 0, errors.New("operation name is required")
	}
	cleanName := strings.TrimPrefix(opName, "syncsets/")
	parts := strings.Split(cleanName, "/")
	if len(parts) != 2 || parts[0] == "" || parts[1] == "" {
		return "", 0, fmt.Errorf("invalid operation name format %q: expected [syncsets/]<name>/<generation>", opName)
	}
	gen, err := strconv.ParseUint(parts[1], 10, 64)
	if err != nil {
		return "", 0, fmt.Errorf("invalid generation in operation name %q: %v", opName, err)
	}
	return parts[0], syncset.Generation(gen), nil
}

// GetOperation checks the state of the syncset associated with the long-running operation.
// The requested operation name requires the format [syncsets/]<name>/<generation>.
// Returns InvalidArgument if the operation name or generation format is invalid, and NotFound if the SyncSet does not exist.
func (h *CASSyncOperationsHandler) GetOperation(ctx context.Context, req *lropb.GetOperationRequest) (*lropb.Operation, error) {
	name, gen, err := parseOperationName(req.GetName())
	if err != nil {
		return nil, status.Error(codes.InvalidArgument, err.Error())
	}

	ss, err := h.Controller.SyncSet(name, gen)
	if err != nil || ss == nil {
		return nil, status.Errorf(codes.NotFound, "operation %q not found", req.GetName())
	}

	return syncSetToOperation(ss, req.GetName(), nil)
}

// DeleteOperation deletes all generations of the associated SyncSet.
func (h *CASSyncOperationsHandler) DeleteOperation(ctx context.Context, req *lropb.DeleteOperationRequest) (*emptypb.Empty, error) {
	// TODO(b/507814403): Implement deletion of long-running operations and their associated SyncSets.
	return nil, status.Error(codes.Unimplemented, "unimplemented")
}

// CancelOperation cancels active synchronization tasks and deletes the associated SyncSet.
func (h *CASSyncOperationsHandler) CancelOperation(ctx context.Context, req *lropb.CancelOperationRequest) (*emptypb.Empty, error) {
	// TODO(b/507814403): Implement cancellation of long-running operations and their active synchronization tasks.
	return nil, status.Error(codes.Unimplemented, "unimplemented")
}

const (
	maxWaitTimeout   = 30 * time.Second
	waitPollInterval = 200 * time.Millisecond
)

// WaitOperation waits until the specified long-running operation is done or reaches at most a specified timeout, returning the latest state.
func (h *CASSyncOperationsHandler) WaitOperation(ctx context.Context, req *lropb.WaitOperationRequest) (*lropb.Operation, error) {
	name, gen, err := parseOperationName(req.GetName())
	if err != nil {
		return nil, status.Error(codes.InvalidArgument, err.Error())
	}

	ss, err := h.Controller.SyncSet(name, gen)
	if err != nil || ss == nil {
		return nil, status.Errorf(codes.NotFound, "operation %q not found", req.GetName())
	}
	if ss.State.IsDone() {
		return syncSetToOperation(ss, req.GetName(), nil)
	}

	timeout := maxWaitTimeout
	if req.GetTimeout() != nil && req.GetTimeout().IsValid() {
		reqTimeout := req.GetTimeout().AsDuration()
		if reqTimeout > 0 && reqTimeout < timeout {
			timeout = reqTimeout
		}
	}

	waitCtx, cancel := context.WithTimeout(ctx, timeout)
	defer cancel()

	ticker := time.NewTicker(waitPollInterval)
	defer ticker.Stop()
	// poll SyncSet state
	for {
		select {
		case <-waitCtx.Done():
			if ctx.Err() != nil {
				return nil, status.FromContextError(ctx.Err()).Err()
			}
			ss, err := h.Controller.SyncSet(name, gen)
			if err != nil || ss == nil {
				return nil, status.Errorf(codes.NotFound, "operation %q not found", req.GetName())
			}
			return syncSetToOperation(ss, req.GetName(), nil)
		case <-ticker.C:
			ss, err := h.Controller.SyncSet(name, gen)
			if err != nil || ss == nil {
				return nil, status.Errorf(codes.NotFound, "operation %q not found", req.GetName())
			}
			if ss.State.IsDone() {
				return syncSetToOperation(ss, req.GetName(), nil)
			}
		}
	}
}

// syncSetToOperation translates a SyncSet to a long-running Operation.
func syncSetToOperation(ss *syncset.SyncSet, opName string, reqPreloadSet *preloadsetpb.PreloadSet) (*lropb.Operation, error) {
	if ss == nil {
		return nil, status.Error(codes.NotFound, "syncset not found")
	}

	progress := float32(0.0)
	if ss.State.IsDone() {
		progress = 1.0
	}
	metadata := &preloadsetpb.Metadata{
		CreateTime: timestamppb.New(ss.Created),
		OperationProgress: &preloadsetpb.Metadata_OperationProgress{
			Progress: progress,
		},
	}
	if reqPreloadSet != nil && reqPreloadSet.GetMetadata() != nil {
		metadata.UsageDescription = reqPreloadSet.GetMetadata().GetUsageDescription()
	}

	// TODO(b/507814403): Add objects to the metadata in the long running operation once clients read those objects from the metadata.

	anyMeta, err := anypb.New(metadata)
	if err != nil {
		return nil, status.Errorf(codes.Internal, "failed to marshal metadata: %v", err)
	}

	op := &lropb.Operation{
		Name:     opName,
		Metadata: anyMeta,
		Done:     ss.State.IsDone(),
	}
	if !op.Done {
		return op, nil
	}
	// If done, populate result field here
	switch ss.State {
	case syncset.SyncStateDone:
		resp := &ocpsgrpcpb.UpsertPreloadSetResponse{
			PreloadSet: &preloadsetpb.PreloadSet{Name: ss.Name},
		}
		anyResp, err := anypb.New(resp)
		if err != nil {
			return nil, status.Errorf(codes.Internal, "failed to marshal response: %v", err)
		}
		op.Result = &lropb.Operation_Response{Response: anyResp}

	case syncset.SyncStateDoneError:
		op.Result = &lropb.Operation_Error{
			Error: &statuspb.Status{
				Code:    int32(codes.Internal),
				Message: "SyncSet failed",
			},
		}
	}

	return op, nil
}
