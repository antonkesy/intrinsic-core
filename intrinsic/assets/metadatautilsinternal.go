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

// Package metadatautilsinternal contains internal utilities for populating output-only asset metadata fields.
package metadatautilsinternal

import (
	"intrinsic/assets/hardware_devices/graph"
	"intrinsic/assets/idutils"
	"intrinsic/assets/interfaceutils"
	"intrinsic/util/proto/descriptor"
	"intrinsic/util/proto/names"

	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	dapb "intrinsic/assets/data/proto/v1/data_asset_go_proto"
	papb "intrinsic/assets/processes/proto/process_asset_go_proto"
	metadatapb "intrinsic/assets/proto/metadata_go_proto"
	sompb "intrinsic/assets/scene_objects/proto/scene_object_manifest_go_proto"
	smpb "intrinsic/assets/services/proto/service_manifest_go_proto"
	psmpb "intrinsic/skills/proto/processed_skill_manifest_go_proto"

	dpb "google.golang.org/protobuf/types/descriptorpb"
)

// PopulateOutputOnlyFieldsForData fills in the output-only fields for a Data Asset.
func PopulateOutputOnlyFieldsForData(da *dapb.DataAsset, md *metadatapb.Metadata) error {
	protoName, err := names.AnyToProtoName(da.GetData())
	if err != nil {
		return status.Errorf(codes.Internal, "could not get payload proto name: %v", err)
	}
	if err := names.ValidateProtoName(protoName); err != nil {
		return status.Errorf(codes.InvalidArgument, "invalid proto name %q: %v", protoName, err)
	}
	md.Provides = []*metadatapb.Interface{
		{
			Uri: interfaceutils.DataURIPrefix + protoName,
		},
	}
	md.FileDescriptorSet = da.GetFileDescriptorSet()
	return nil
}

// PopulateOutputOnlyFieldsForProcess fills in the output-only fields for a Process Asset.
func PopulateOutputOnlyFieldsForProcess(pa *papb.ProcessAsset, md *metadatapb.Metadata) error {
	md.Provides = nil

	// Note: The file descriptor set can be legitimately nil for various reasons:
	// - GetDescription() (=the Skill proto) is nil if BT is not a PBT.
	// - GetParameterDescription() is nil if BT is a PBT without parameters.
	md.FileDescriptorSet = pa.GetBehaviorTree().GetDescription().GetParameterDescription().GetParameterDescriptorFileset()
	if md.GetFileDescriptorSet() == nil {
		md.FileDescriptorSet = &dpb.FileDescriptorSet{}
	}

	return nil
}

// PopulateOutputOnlyFieldsForSceneObject fills in the output-only fields for a SceneObjet Asset.
func PopulateOutputOnlyFieldsForSceneObject(so *sompb.ProcessedSceneObjectManifest, md *metadatapb.Metadata) error {
	md.Provides = nil
	md.FileDescriptorSet = so.GetAssets().GetFileDescriptorSet()
	return nil
}

// PopulateOutputOnlyFieldsForService fills in the output-only fields for a service asset.
func PopulateOutputOnlyFieldsForService(psm *smpb.ProcessedServiceManifest, md *metadatapb.Metadata) error {
	var provides []*metadatapb.Interface
	for _, prefix := range psm.GetServiceDef().GetServiceProtoPrefixes() {
		if err := names.ValidateProtoPrefix(prefix); err != nil {
			return status.Errorf(codes.InvalidArgument, "invalid service proto prefix %q: %v", prefix, err)
		}
		provides = append(provides, &metadatapb.Interface{
			Uri: interfaceutils.GRPCURIPrefix + prefix[1:len(prefix)-1],
		})
	}
	md.Provides = provides
	md.FileDescriptorSet = psm.GetAssets().GetFileDescriptorSet()
	return nil
}

// PopulateOutputOnlyFieldsForSkill fills in the output-only fields for a skill asset.
func PopulateOutputOnlyFieldsForSkill(psm *psmpb.ProcessedSkillManifest, md *metadatapb.Metadata) error {
	md.Provides = nil
	md.FileDescriptorSet = psm.GetAssets().GetFileDescriptorSet()
	return nil
}

// PopulateOutputOnlyFieldsForHardwareDevice fills in the output-only fields for a hardware device asset.
func PopulateOutputOnlyFieldsForHardwareDevice(assets *graph.CollectedAssets, md *metadatapb.Metadata) error {
	var provides []*metadatapb.Interface
	for _, service := range assets.Services() {
		for _, prefix := range service.GetServiceDef().GetServiceProtoPrefixes() {
			if err := names.ValidateProtoPrefix(prefix); err != nil {
				return status.Errorf(codes.InvalidArgument, "invalid service proto prefix %q: %v", prefix, err)
			}
			provides = append(provides, &metadatapb.Interface{
				Uri: interfaceutils.GRPCURIPrefix + prefix[1:len(prefix)-1],
			})
		}
	}

	var fdss []*dpb.FileDescriptorSet
	var keys []string
	for key, data := range assets.DataMap {
		fdss = append(fdss, data.GetFileDescriptorSet())
		keys = append(keys, key)
	}
	for key, service := range assets.ServicesMap {
		fdss = append(fdss, service.GetAssets().GetFileDescriptorSet())
		keys = append(keys, key)
	}
	for key, sceneObject := range assets.SceneObjectsMap {
		fdss = append(fdss, sceneObject.GetAssets().GetFileDescriptorSet())
		keys = append(keys, key)
	}

	// Merge them into a single FileDescriptorSet.
	merged, err := descriptor.MergeFileDescriptorSets(fdss,
		descriptor.WithKeys(keys),
		descriptor.WithSkipValidation(true), // TODO(525300455): Figure out what to do here.
	)
	if err != nil {
		return status.Errorf(codes.InvalidArgument, "failed to merge file descriptor sets for HardwareDevice %v: %v", idutils.IDFromProtoUnchecked(md.GetIdVersion().GetId()), err)
	}

	md.Provides = provides
	md.FileDescriptorSet = merged
	return nil
}
