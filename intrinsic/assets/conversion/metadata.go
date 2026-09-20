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

// Package metadata implements helpers for converting Asset metadata to metadata specific to Asset types.
package metadata

import (
	hardwaredevicemanifestpb "intrinsic/assets/hardware_devices/proto/v1/hardware_device_manifest_go_proto"
	assettagpb "intrinsic/assets/proto/asset_tag_go_proto"
	metadatapb "intrinsic/assets/proto/metadata_go_proto"
	sceneobjectmanifestpb "intrinsic/assets/scene_objects/proto/scene_object_manifest_go_proto"
	servicemanifestpb "intrinsic/assets/services/proto/service_manifest_go_proto"
	processedskillmanifestpb "intrinsic/skills/proto/processed_skill_manifest_go_proto"
)

// AssetMetadataToProcessedSkillMetadata converts an Asset Metadata message to an equivalent SkillMetadata message.
func AssetMetadataToProcessedSkillMetadata(metadata *metadatapb.Metadata) *processedskillmanifestpb.SkillMetadata {
	if metadata == nil {
		return nil
	}
	return &processedskillmanifestpb.SkillMetadata{
		Id:            metadata.GetIdVersion().GetId(),
		DisplayName:   metadata.GetDisplayName(),
		Documentation: metadata.GetDocumentation(),
		Vendor:        metadata.GetVendor(),
	}
}

// AssetMetadataToHardwareDeviceMetadata converts an Asset Metadata message to an equivalent HardwareDeviceMetadata message.
func AssetMetadataToHardwareDeviceMetadata(metadata *metadatapb.Metadata) *hardwaredevicemanifestpb.HardwareDeviceMetadata {
	if metadata == nil {
		return nil
	}

	var assetTags []assettagpb.AssetTag
	if metadata.GetAssetTag() != assettagpb.AssetTag_ASSET_TAG_UNSPECIFIED {
		assetTags = append(assetTags, metadata.GetAssetTag())
	}
	return &hardwaredevicemanifestpb.HardwareDeviceMetadata{
		Id:            metadata.GetIdVersion().GetId(),
		DisplayName:   metadata.GetDisplayName(),
		Vendor:        metadata.GetVendor(),
		Documentation: metadata.GetDocumentation(),
		AssetTags:     assetTags,
	}
}

// AssetMetadataToSceneObjectMetadata converts an Asset Metadata message to an equivalent SceneObjectMetadata message.
func AssetMetadataToSceneObjectMetadata(metadata *metadatapb.Metadata) *sceneobjectmanifestpb.SceneObjectMetadata {
	if metadata == nil {
		return nil
	}
	return &sceneobjectmanifestpb.SceneObjectMetadata{
		Id:            metadata.GetIdVersion().GetId(),
		DisplayName:   metadata.GetDisplayName(),
		Vendor:        metadata.GetVendor(),
		Documentation: metadata.GetDocumentation(),
		AssetTag:      metadata.GetAssetTag(),
	}
}

// AssetMetadataToSceneObjectMetadata converts an Asset Metadata message to an equivalent ServiceMetadata message.
func AssetMetadataToServiceMetadata(metadata *metadatapb.Metadata) *servicemanifestpb.ServiceMetadata {
	if metadata == nil {
		return nil
	}
	return &servicemanifestpb.ServiceMetadata{
		Id:            metadata.GetIdVersion().GetId(),
		DisplayName:   metadata.GetDisplayName(),
		Vendor:        metadata.GetVendor(),
		Documentation: metadata.GetDocumentation(),
		AssetTag:      metadata.GetAssetTag(),
	}
}
