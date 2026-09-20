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

// Package applicationview contains the canonical conversion from a full application proto to a
// view of the same type. This view is marked as incomplete and is used in the frontend / portal.
package applicationview

import (
	"google.golang.org/protobuf/proto"

	applicationpb "intrinsic/config/proto/application_go_proto"
	commonpb "intrinsic/config/proto/common_go_proto"
	ompb "intrinsic/config/proto/operation_mode_go_proto"
	processpb "intrinsic/config/proto/process_go_proto"
	resourcesetpb "intrinsic/config/proto/resource_set_go_proto"
)

func withIncompleteSet(m *commonpb.Metadata) *commonpb.Metadata {
	if m == nil {
		return nil
	}
	m = proto.Clone(m).(*commonpb.Metadata)
	m.IsIncomplete = true
	return m
}

// MetadataOnly reduces an application to a view that contains only metadata, migrate time and
// operation mode. Inverse of [WithoutMetadata]. Does not modify the input.
// Always returns a non-nil application.
func MetadataOnly(a *applicationpb.Application) *applicationpb.Application {
	var process *processpb.Process
	if p := a.GetProcess(); p != nil {
		process = &processpb.Process{}
	}
	var resources *resourcesetpb.ResourceSet
	if r := a.GetResources(); r != nil {
		resources = &resourcesetpb.ResourceSet{}
	}
	return &applicationpb.Application{
		Metadata:      withIncompleteSet(a.GetMetadata()),
		OperationMode: a.GetOperationMode(),
		Process:       process,
		Resources:     resources,
		MigrateTime:   a.GetMigrateTime(),
	}
}

// WithoutMetadata reduces an application to a view that contains no metadata, migrate time or
// operation mode. Inverse of [MetadataOnly]. Does not modify the input. Returns
// nil if the input is nil.
//
// Reversible with [WithMetadataFrom] such that:
// a = WithMetadataFrom(WithoutMetadata(a), a)
//
// Used for storage of application protos in modified solutions.
func WithoutMetadata(a *applicationpb.Application) *applicationpb.Application {
	if a == nil {
		return nil
	}
	res := proto.Clone(a).(*applicationpb.Application)
	res.Metadata = nil
	res.OperationMode = ompb.OperationMode_OPERATION_MODE_UNSPECIFIED
	res.MigrateTime = nil
	return res
}

// WithMetadataFrom adds the fields removed by [WithoutMetadata] from src to (a
// clone of) dst and returns the result. Does not modify either input. Returns
// nil if dst is nil. Does not override fields in dst that are not set (`nil` or
// default value) in src.
//
// Used for reconstruction of full application protos from modified solutions.
func WithMetadataFrom(dst *applicationpb.Application, src *applicationpb.Application) *applicationpb.Application {
	if dst == nil {
		return nil
	}
	res := proto.Clone(dst).(*applicationpb.Application)
	if src.GetMetadata() != nil {
		res.Metadata = src.GetMetadata()
	}
	if src.GetOperationMode() != ompb.OperationMode_OPERATION_MODE_UNSPECIFIED {
		res.OperationMode = src.GetOperationMode()
	}
	if src.GetMigrateTime() != nil {
		res.MigrateTime = src.GetMigrateTime()
	}
	return res
}
