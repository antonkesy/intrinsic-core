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
	"fmt"

	"intrinsic/storage/content_addressable_storage/pkg/idhandling"
	"intrinsic/storage/onprem_cas/internal/syncset"

	lropb "cloud.google.com/go/longrunning/autogen/longrunningpb"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	ocpsgrpcpb "intrinsic/storage/onprem_cas_preload/proto/v1/ocp_service_go_proto"
)

// PreloadHandler implements ocpsgrpcpb.OnpremCASPreloadServiceServer using a SyncSet Controller.
type PreloadHandler struct {
	ocpsgrpcpb.UnimplementedOnpremCASPreloadServiceServer
	Controller *syncset.Controller
}

// NewPreloadHandler creates a new PreloadHandler backed by the given SyncSet controller.
func NewPreloadHandler(controller *syncset.Controller) *PreloadHandler {
	return &PreloadHandler{
		Controller: controller,
	}
}

// UpsertPreloadSet translates an UpsertPreloadSetRequest to a SyncSet Upsert operation and returns a long-running Operation.
func (h *PreloadHandler) UpsertPreloadSet(ctx context.Context, req *ocpsgrpcpb.UpsertPreloadSetRequest) (*lropb.Operation, error) {
	preloadSet := req.GetPreloadSet()
	if preloadSet == nil {
		return nil, status.Error(codes.InvalidArgument, "preload_set is required")
	}

	name := preloadSet.GetName()
	if name == "" {
		name = "running_solution"
	}

	// Extract digests
	sbo := preloadSet.GetStrategyByObject()
	ds := make([]string, 0, len(sbo))
	for oid := range sbo {
		digest, err := idhandling.StripSchema(oid)
		if err != nil {
			return nil, status.Errorf(codes.InvalidArgument, "invalid object ID %q: %v", oid, err)
		}
		ds = append(ds, digest)
	}

	ss, err := h.Controller.Upsert(name, ds)
	if err != nil {
		return nil, status.Errorf(codes.Internal, "failed to upsert syncset %q: %v", name, err)
	}

	return syncSetToOperation(ss, fmt.Sprintf("%s/%d", ss.Name, ss.Generation), preloadSet)
}

// DeletePreloadSet deletes the specified preload set and its associated SyncSet.
func (h *PreloadHandler) DeletePreloadSet(ctx context.Context, req *ocpsgrpcpb.DeletePreloadSetRequest) (*ocpsgrpcpb.DeletePreloadSetResponse, error) {
	// TODO(b/507814403): Implement deletion of PreloadSet and its associated SyncSet.
	return nil, status.Error(codes.Unimplemented, "unimplemented")
}
