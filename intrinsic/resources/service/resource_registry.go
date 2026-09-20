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

// Package resourceregistry contains a service to manage resource sets and
// individual resource instances within the context of a deployed PPR
// application.
package resourceregistry

import (
	"context"
	"maps"
	"slices"

	"intrinsic/resources/service/resourceregistryutil"
	"intrinsic/util/pagination/pagetoken"

	"go.opencensus.io/trace"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	rrpb "intrinsic/resources/proto/resource_registry_go_proto"
	rtrpb "intrinsic/resources/proto/resource_type_runtime_go_proto"
)

// ResourceReader has methods to retrieve resource instances from the currently
// running application.
type ResourceReader interface {
	ResourceInstancesAsMap(ctx context.Context, includeAllAssetTypes bool) (map[string]*rrpb.ResourceInstance, map[string]*rtrpb.ResourceTypeRuntime, error)
	ResourceInstance(ctx context.Context, id string) (*rrpb.ResourceInstance, *rtrpb.ResourceTypeRuntime, error)
}

// Server contains data associated with a resource registry server.
type Server struct {
	rr ResourceReader
}

type paginatedResourceInstance struct {
	*rrpb.ListResourceInstanceRequest
}

func (r paginatedResourceInstance) ClearPagination() {
	r.PageToken = ""
	r.PageSize = 0
}

func nameAsKey(s *rrpb.ResourceInstance) []string {
	return []string{
		s.GetName(),
	}
}

func (s *Server) ListResourceInstances(ctx context.Context, request *rrpb.ListResourceInstanceRequest) (*rrpb.ListResourceInstanceResponse, error) {
	ctx, span := trace.StartSpan(ctx, "resourceregistry.ListResourceInstances")
	defer span.End()

	pageSize, err := resourceregistryutil.PageSize(request)
	if err != nil {
		return nil, err
	}

	startAfters, err := resourceregistryutil.StartAfters(paginatedResourceInstance{request})
	if err != nil {
		return nil, err
	}

	rim, _, err := s.rr.ResourceInstancesAsMap(
		ctx,
		false, // includeAllAssetTypes
	)
	if err != nil {
		return nil, err
	}

	ris := slices.Collect(maps.Values(rim))
	resourceregistryutil.Filter(request.GetStrictFilter(), &ris)
	ris = slices.DeleteFunc(ris, func(ri *rrpb.ResourceInstance) bool {
		return (slices.Compare(nameAsKey(ri), startAfters) <= 0)
	})
	slices.SortFunc(ris, func(lhs *rrpb.ResourceInstance, rhs *rrpb.ResourceInstance) int {
		return slices.Compare(nameAsKey(lhs), nameAsKey(rhs))
	})

	if len(ris) <= int(pageSize) {
		return &rrpb.ListResourceInstanceResponse{
			Instances: ris,
		}, nil
	}

	ris = ris[:pageSize]
	paginatedResourceInstance{request}.ClearPagination()

	pt, err := pagetoken.Opacify(request, "", nameAsKey(ris[pageSize-1]))
	if err != nil {
		return nil, status.Error(codes.Internal, "could not generate next page token")
	}
	return &rrpb.ListResourceInstanceResponse{
		Instances:     ris,
		NextPageToken: pt,
	}, nil
}

func (s *Server) GetResourceInstance(ctx context.Context, request *rrpb.GetResourceInstanceRequest) (*rrpb.ResourceInstance, error) {
	ctx, span := trace.StartSpan(ctx, "resourceregistry.GetResourceInstance")
	defer span.End()

	ri, _, err := s.rr.ResourceInstance(ctx, request.GetName())
	if err != nil {
		return nil, err
	}
	return ri, nil
}

// NewServer creates a resource registry server for the provided cluster.
func NewServer(rr ResourceReader) *Server {
	return &Server{
		rr: rr,
	}
}
