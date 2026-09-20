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

// Package resourceregistryutil contains utility functions for the resource
// registry. These functions may be used by both the public and internal
// versions of the resource registry.
package resourceregistryutil

import (
	"slices"

	"intrinsic/util/pagination/pagetoken"

	"golang.org/x/exp/maps"
	"google.golang.org/grpc/codes"
	grpcstatus "google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"

	mpb "intrinsic/assets/proto/metadata_go_proto"
	rrpb "intrinsic/resources/proto/resource_registry_go_proto"
)

const (
	defaultNumHandles = 50
	maxNumHandles     = 200
)

// getPageSize is any type that can return a page size.
type getPageSize interface {
	GetPageSize() int64
}

// getPageToken is any type that can return a page token.
type getPageToken interface {
	GetPageToken() string
}

// paginated represents a request in a paginated API.  ClearPagination is
// provided to set the PageToken and PageSize members since the Open V1 API
// doesn't have functions that would allow those fields to be cleared
// automatically.  Otherwise it would probably make more sense to have that in
// the field-specific interfaces above.
type paginated interface {
	proto.Message
	getPageSize
	getPageToken

	// ClearPagination should clear page size and page token, such that their
	// get methods return 0 and an empty string, respectively.
	ClearPagination()
}

// PageSize returns the page size for this request according to AIP guidelines
// using defaultNumHandles and maxNumHandles values.
func PageSize(request getPageSize) (int64, error) {
	pageSize := request.GetPageSize()
	if pageSize < 0 {
		return 0, grpcstatus.Errorf(codes.InvalidArgument, "Invalid page size: %d", pageSize)
	}
	if pageSize == 0 {
		pageSize = defaultNumHandles
	}
	if pageSize > maxNumHandles {
		pageSize = maxNumHandles
	}
	return pageSize, nil
}

// StartAfters returns a sort key to be used when continuing a previously issued
// request. Items that are "<=" than this sort key should be filtered out. This
// function might reset the request's page token and page size fields.
func StartAfters(request paginated) ([]string, error) {
	var startAfters []string
	if request.GetPageToken() != "" {
		pt, err := pagetoken.Deopacify(request.GetPageToken())
		if err != nil {
			return nil, grpcstatus.Error(codes.InvalidArgument, "invalid page token")
		}
		prevReq := request.ProtoReflect().New().Interface()
		if err := proto.Unmarshal(pt.GetRequest(), prevReq); err != nil {
			return nil, grpcstatus.Error(codes.InvalidArgument, "invalid page token")
		}
		request.ClearPagination()
		if !proto.Equal(request, prevReq) {
			return nil, grpcstatus.Error(codes.InvalidArgument, "invalid page token")
		}
		startAfters = pt.GetStartAfter()
	}
	return startAfters, nil
}

// Filter applies the strict filter given in the request to the slice of
// resource instances.
func Filter(filter *rrpb.ListResourceInstanceRequest_StrictFilter, ris *[]*rrpb.ResourceInstance) {
	required := filter.GetCapabilityNames()
	*ris = slices.DeleteFunc(*ris, func(ri *rrpb.ResourceInstance) bool {
		if filter.GetResourceFamilyId() != "" && filter.GetResourceFamilyId() != ri.GetResourceFamilyId() {
			return true
		}
		capabilities := maps.Keys(ri.GetResourceHandle().GetResourceData())
		for _, n := range required {
			if !slices.Contains(capabilities, n) {
				return true
			}
		}
		return false
	})
}

type hasMetadata interface {
	GetMetadata() *mpb.Metadata
}

// IDVersionAsKey returns an array of strings that can be used as a sort key
// based on id_version in a metadata field.
func IDVersionAsKey(s hasMetadata) []string {
	return []string{
		s.GetMetadata().GetIdVersion().GetId().GetPackage(),
		s.GetMetadata().GetIdVersion().GetId().GetName(),
		s.GetMetadata().GetIdVersion().GetVersion(),
	}
}
