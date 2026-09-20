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

// Package resourcefix contains utils that adapt resource data to meet the requirements of
// the latest platform version.
package resourcefix

import (
	"fmt"
	"slices"

	"intrinsic/assets/data/datafix"
	"intrinsic/assets/processes/processfix"
	"intrinsic/skills/skillfix"
	"intrinsic/util/proto/descriptorcompatibility" 

	iopb "intrinsic/assets/proto/installation_origin_go_proto"
	metadatapb "intrinsic/assets/proto/metadata_go_proto"
	drpb "intrinsic/assets/services/proto/v1/dynamic_reconfiguration_go_proto"
	sspb "intrinsic/assets/services/proto/v1/service_state_go_proto"
	rsdpb "intrinsic/resources/proto/resource_service_definition_go_proto"
	rtrpb "intrinsic/resources/proto/resource_type_runtime_go_proto"

	dpb "google.golang.org/protobuf/types/descriptorpb"
)

// fixOpts contains options for fixing resource data.
type fixOpts struct {
	populateOldFields   bool
	clearObsoleteFields bool
}

// FixOption is an option for fixing resource data.
type FixOption func(*fixOpts)

// WithPopulateOldFields specifies whether to backfill old deprecated fields if empty.
func WithPopulateOldFields(populate bool) FixOption {
	return func(opts *fixOpts) {
		opts.populateOldFields = populate
	}
}

// WithClearObsoleteFields specifies whether to clear obsolete manifest fields. A field can only
// be considered obsolete if the platform no longer uses it.
func WithClearObsoleteFields(clear bool) FixOption {
	return func(opts *fixOpts) {
		opts.clearObsoleteFields = clear
	}
}

// resourceServiceDefinition updates a ResourceServiceDefinition to meet the requirements of the
// latest platform version.
func resourceServiceDefinition(rsd *rsdpb.ResourceServiceDefinition, opts *fixOpts) error {
	if rsd == nil {
		return nil
	}

	// Populate the dynamic reconfiguration platform gRPC interface if only the deprecated boolean
	// setting is present and true.
	if conf := rsd.GetDynamicReconfigurationConfig(); conf == nil && rsd.GetSupportsDynamicReconfiguration() {
		rsd.DynamicReconfigurationConfig = &drpb.DynamicReconfigurationConfig{
			ServiceVersions: []drpb.DynamicReconfigurationConfig_ServiceVersion{
				drpb.DynamicReconfigurationConfig_INTRINSIC_PROTO_SERVICES_V1_DYNAMIC_RECONFIGURATION,
			},
		}
	}

	// Populate the service state platform gRPC interface if only the deprecated boolean setting is
	// present and true.
	if conf := rsd.GetServiceStateConfig(); conf == nil && rsd.GetSupportsServiceState() {
		rsd.ServiceStateConfig = &sspb.ServiceStateConfig{
			ServiceVersions: []sspb.ServiceStateConfig_ServiceVersion{
				sspb.ServiceStateConfig_INTRINSIC_PROTO_SERVICES_V1_SERVICE_STATE,
			},
		}
	}

	if opts.populateOldFields {
		// Backfill the deprecated SupportsDynamicReconfiguration field if the new config is present.
		if conf := rsd.GetDynamicReconfigurationConfig(); conf != nil {
			if slices.Contains(conf.GetServiceVersions(), drpb.DynamicReconfigurationConfig_INTRINSIC_PROTO_SERVICES_V1_DYNAMIC_RECONFIGURATION) {
				rsd.SupportsDynamicReconfiguration = true
			}
		}

		// Backfill the deprecated SupportsServiceState field if the new config is present.
		if conf := rsd.GetServiceStateConfig(); conf != nil {
			if slices.Contains(conf.GetServiceVersions(), sspb.ServiceStateConfig_INTRINSIC_PROTO_SERVICES_V1_SERVICE_STATE) {
				rsd.SupportsServiceState = true
			}
		}
	}

	if opts.clearObsoleteFields {
		rsd.SupportsDynamicReconfiguration = false
		rsd.SupportsServiceState = false
	}

	return nil
}

// ResourceTypeRuntime updates a ResourceTypeRuntime to meet the requirements of the latest
// platform version.
func ResourceTypeRuntime(rtr *rtrpb.ResourceTypeRuntime, options ...FixOption) error {
	opts := &fixOpts{}
	for _, opt := range options {
		opt(opts)
	}
	if rtr == nil {
		return nil
	}

	if rtr.InstallationOrigin == iopb.InstallationOrigin_INSTALLATION_ORIGIN_CATALOG {
		// Some Assets were previously released to the catalog without FileDescriptorSets
		// (SceneObjects without user data and Services that don't provide gRPC services). We can
		// backfill these with an empty FileDescriptorSet.
		if rtr.GetMetadata().GetFileDescriptorSet() == nil {
			if rtr.Metadata == nil {
				rtr.Metadata = &metadatapb.Metadata{}
			}
			rtr.Metadata.FileDescriptorSet = &dpb.FileDescriptorSet{}
		}
	}


	if err := descriptorcompatibility.Reconcile(rtr.GetMetadata().GetFileDescriptorSet()); err != nil {
		return fmt.Errorf("unable to reconcile the file descriptor set for ResourceTypeRuntime: %w", err)
	}


	if serviceDef := rtr.GetServiceDef(); serviceDef != nil {
		if err := resourceServiceDefinition(serviceDef, opts); err != nil {
			return fmt.Errorf("failed to fix service definition: %w", err)
		}
	}
	if skill := rtr.GetSkill(); skill != nil {
		if err := skillfix.ProcessedManifest(skill, skillfix.WithPopulateOldFields(opts.populateOldFields), skillfix.WithClearObsoleteFields(opts.clearObsoleteFields)); err != nil {
			return fmt.Errorf("failed to fix the skill processed manifest: %w", err)
		}
	}
	if data := rtr.GetData(); data != nil {
		if err := datafix.DataAsset(data); err != nil {
			return fmt.Errorf("failed to fix the data asset: %w", err)
		}
	}
	if process := rtr.GetProcess(); process != nil {
		if err := processfix.ProcessAsset(process); err != nil {
			return fmt.Errorf("failed to fix the process asset: %w", err)
		}
	}

	return nil
}
