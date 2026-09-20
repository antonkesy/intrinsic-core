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

// Package runtime implements the conversion of resource type runtimes into local assets, and vice-versa.
package runtime

import (
	"context"
	"crypto/sha256"
	"fmt"
	"slices"

	"golang.org/x/exp/maps"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"

	"intrinsic/assets/conversion/metadata"
	"intrinsic/assets/data/datavalidate"
	"intrinsic/assets/hardware_devices/graph"
	"intrinsic/assets/idutils"
	"intrinsic/assets/metadatautilsinternal"
	"intrinsic/assets/processes/processmanifest"
	"intrinsic/assets/processes/processvalidate"
	"intrinsic/scene/validate/go/sceneobjectvalidation"
	"intrinsic/util/go/validate"
	es "intrinsic/util/status/extstatus"

	acigrpcpb "intrinsic/assets/catalog/proto/v1/asset_catalog_internal_go_proto"
	dataassetpb "intrinsic/assets/data/proto/v1/data_asset_go_proto"
	hardwaredevicemanifestpb "intrinsic/assets/hardware_devices/proto/v1/hardware_device_manifest_go_proto"
	hdmpb "intrinsic/assets/hardware_devices/proto/v1/hardware_device_manifest_go_proto"
	processassetpb "intrinsic/assets/processes/proto/process_asset_go_proto"
	assettagpb "intrinsic/assets/proto/asset_tag_go_proto"
	assettypepb "intrinsic/assets/proto/asset_type_go_proto"
	idpb "intrinsic/assets/proto/id_go_proto"
	installationoriginpb "intrinsic/assets/proto/installation_origin_go_proto"
	metadatapb "intrinsic/assets/proto/metadata_go_proto"
	assetpb "intrinsic/assets/proto/v1/asset_go_proto"
	assetgraphpb "intrinsic/assets/proto/v1/asset_graph_go_proto"
	processedassetpb "intrinsic/assets/proto/v1/processed_asset_go_proto"
	referencepb "intrinsic/assets/proto/v1/reference_go_proto"
	sceneobjectmanifestpb "intrinsic/assets/scene_objects/proto/scene_object_manifest_go_proto"
	envpb "intrinsic/assets/services/proto/env_var_go_proto"
	servicemanifestpb "intrinsic/assets/services/proto/service_manifest_go_proto"
	servicepermissionspb "intrinsic/assets/services/proto/service_permissions_go_proto"
	servicevolumepb "intrinsic/assets/services/proto/service_volume_go_proto"
	imagepb "intrinsic/kubernetes/workcell_spec/proto/image_go_proto"
	resourcepermissionspb "intrinsic/resources/proto/resource_permissions_go_proto"
	resourceservicedefinitionpb "intrinsic/resources/proto/resource_service_definition_go_proto"
	resourcetyperuntimepb "intrinsic/resources/proto/resource_type_runtime_go_proto"
	resourcevolumepb "intrinsic/resources/proto/resource_volume_go_proto"
	processedskillmanifestpb "intrinsic/skills/proto/processed_skill_manifest_go_proto"
)

const (
	// errComponent is used to indicate the origin of extended status errors
	errComponent = "ai.intrinsic.runtime"
)

// TODO(b/368034618): Move these error codes to a proto definition.
const (
	// To keep stability of error codes, do not change the sequence of existing error codes. Only append new error codes below.
	// 1-9999 are reserved by the internals of extstatus
	errCodeInvalidSceneObject = 10000
)

type toRuntimeOptions struct {
	// An internal asset catalog client that is only needed for hardware device conversion.
	aciClient          acigrpcpb.AssetCatalogInternalClient
	installationOrigin installationoriginpb.InstallationOrigin
	// If true, skip some validation steps.
	skipValidation bool
	version        string
}

// ToRuntimeOption is a functional option for ToRuntime.
type ToRuntimeOption func(*toRuntimeOptions)

// WithACIClient sets the ACIClient option.
func WithACIClient(client acigrpcpb.AssetCatalogInternalClient) ToRuntimeOption {
	return func(opts *toRuntimeOptions) {
		opts.aciClient = client
	}
}

// WithSkipValidation sets the skipValidation option.
func WithSkipValidation() ToRuntimeOption {
	return func(opts *toRuntimeOptions) {
		opts.skipValidation = true
	}
}

// WithInstallationOrigin sets the installation origin to use in the output ResourceTypeRuntime.
//
// If the origin is anything other than INSTALLATION_ORIGIN_SIDELOADED, the version must be
// specified explicitly using WithVersion.
func WithInstallationOrigin(origin installationoriginpb.InstallationOrigin) ToRuntimeOption {
	return func(opts *toRuntimeOptions) {
		opts.installationOrigin = origin
	}
}

// WithVersion sets the version to use in the output ResourceTypeRuntime.
func WithVersion(version string) ToRuntimeOption {
	return func(opts *toRuntimeOptions) {
		opts.version = version
	}
}

func resolveToRuntimeOptions(sideloadedAsset proto.Message, options []ToRuntimeOption) (*toRuntimeOptions, error) {
	opts := &toRuntimeOptions{}
	for _, opt := range options {
		opt(opts)
	}

	if opts.installationOrigin == installationoriginpb.InstallationOrigin_INSTALLATION_ORIGIN_UNSPECIFIED {
		return nil, fmt.Errorf("installation origin must be specified")
	}
	if opts.installationOrigin == installationoriginpb.InstallationOrigin_INSTALLATION_ORIGIN_SIDELOADED {
		version, err := SideloadedVersion(sideloadedAsset)
		if err != nil {
			return nil, err
		}
		if opts.version != "" && opts.version != version {
			return nil, fmt.Errorf("version must not be specified for sideloaded assets")
		}
		opts.version = version
	} else if opts.version == "" {
		return nil, fmt.Errorf("version must be specified for non-sideloaded assets")
	}

	return opts, nil
}

func validateResourceTypeRuntime(rtr *resourcetyperuntimepb.ResourceTypeRuntime) error {
	if err := idutils.ValidateIDProto(rtr.GetMetadata().GetIdVersion().GetId()); err != nil {
		return fmt.Errorf("invalid name or package: %v", err)
	}
	// Not a strict requirement now that we have package, but this still is
	// expected by a handful of places, including IntrinsicDB still, so
	// continue to enforce it to not cause problem.
	if rtr.GetMetadata().GetVendor().GetDisplayName() == "" {
		return fmt.Errorf("vendor.display_name must be specified")
	}
	if rtr.GetServiceDef() != nil && rtr.GetServiceDef().GetSimSpec() == nil {
		return fmt.Errorf("a sim_spec must be specified if a service_def is provided")
	}
	if ss := rtr.GetServiceDef().GetSimSpec(); ss.GetHostNetwork() {
		return fmt.Errorf("asset requested host network in simulation; this probably is not necessary and can slow down adding/removing assets in simulation")
	}
	for _, imageWithSettings := range rtr.GetServiceDef().GetSimSpec().GetImage() {
		if imageWithSettings.GetRequiresRtpcNode() {
			return fmt.Errorf("asset requested requires_rtpc_node in simulation; this probably is not intended and can cause software to run on unexpected nodes")
		}
	}
	return nil
}

// Converts a ResourceTypeRuntime to an Asset.
func RuntimeToAsset(rtr *resourcetyperuntimepb.ResourceTypeRuntime) (*assetpb.Asset, error) {
	if rtr == nil {
		return nil, fmt.Errorf("runtime is nil")
	}
	if rtr.GetMetadata() == nil {
		return nil, fmt.Errorf("runtime does not have metadata")
	}

	if rtr.GetInstallationOrigin() == installationoriginpb.InstallationOrigin_INSTALLATION_ORIGIN_CATALOG {
		return &assetpb.Asset{
			Source: &assetpb.Asset_Catalog{
				Catalog: &referencepb.CatalogAsset{
					AssetType: rtr.GetMetadata().GetAssetType(),
					IdVersion: rtr.GetMetadata().GetIdVersion(),
				},
			},
		}, nil
	}

	local, err := RuntimeToProcessedAsset(rtr)
	if err != nil {
		return nil, err
	}
	return &assetpb.Asset{
		Source: &assetpb.Asset_Local{
			Local: local,
		},
	}, nil
}

// RuntimeToProcessedAsset converts ResourceTypeRuntime to a local, ProcessedAsset.
//
// This conversion is lossy, as it drops version information.
func RuntimeToProcessedAsset(rtr *resourcetyperuntimepb.ResourceTypeRuntime) (*processedassetpb.ProcessedAsset, error) {
	assetType := rtr.GetMetadata().GetAssetType()
	switch assetType {
	case assettypepb.AssetType_ASSET_TYPE_DATA:
		data, err := RuntimeToData(rtr)
		if err != nil {
			return nil, fmt.Errorf("could not convert runtime to Data for %s: %w", findAssetID(rtr), err)
		}
		return &processedassetpb.ProcessedAsset{
			Variant: &processedassetpb.ProcessedAsset_Data{
				Data: data,
			},
		}, nil
	case assettypepb.AssetType_ASSET_TYPE_HARDWARE_DEVICE:
		hwDevice, err := RuntimeToHardwareDevice(rtr)
		if err != nil {
			return nil, fmt.Errorf("could not convert runtime to HardwareDevice for %s: %w", findAssetID(rtr), err)
		}
		return &processedassetpb.ProcessedAsset{
			Variant: &processedassetpb.ProcessedAsset_HardwareDevice{
				HardwareDevice: hwDevice,
			},
		}, nil
	case assettypepb.AssetType_ASSET_TYPE_PROCESS:
		process, err := RuntimeToProcess(rtr)
		if err != nil {
			return nil, fmt.Errorf("could not convert runtime to Process for %s: %w", findAssetID(rtr), err)
		}
		return &processedassetpb.ProcessedAsset{
			Variant: &processedassetpb.ProcessedAsset_Process{
				Process: process,
			},
		}, nil
	case assettypepb.AssetType_ASSET_TYPE_SCENE_OBJECT:
		sceneObject, err := RuntimeToSceneObject(rtr)
		if err != nil {
			return nil, fmt.Errorf("could not convert runtime to SceneObject for %s: %w", findAssetID(rtr), err)
		}
		return &processedassetpb.ProcessedAsset{
			Variant: &processedassetpb.ProcessedAsset_SceneObject{
				SceneObject: sceneObject,
			},
		}, nil
	case assettypepb.AssetType_ASSET_TYPE_SERVICE:
		service, err := RuntimeToService(rtr)
		if err != nil {
			return nil, fmt.Errorf("could not convert runtime to Service for %s: %w", findAssetID(rtr), err)
		}
		return &processedassetpb.ProcessedAsset{
			Variant: &processedassetpb.ProcessedAsset_Service{
				Service: service,
			},
		}, nil
	case assettypepb.AssetType_ASSET_TYPE_SKILL:
		skill, err := runtimeToSkill(rtr)
		if err != nil {
			return nil, fmt.Errorf("could not convert runtime to Skill for %s: %w", findAssetID(rtr), err)
		}
		return &processedassetpb.ProcessedAsset{
			Variant: &processedassetpb.ProcessedAsset_Skill{
				Skill: skill,
			},
		}, nil
	case assettypepb.AssetType_ASSET_TYPE_UNSPECIFIED:
		var hint string
		if at := inferRuntimeAssetType(rtr); at != assettypepb.AssetType_ASSET_TYPE_UNSPECIFIED {
			hint = fmt.Sprintf(" (it might be %s)", at.String())
		}

		return nil, fmt.Errorf("Asset type must be specified for %s%s", findAssetID(rtr), hint)
	default:
		return nil, fmt.Errorf("unknown runtime Asset type for %s: %v", findAssetID(rtr), assetType)
	}
}

// RuntimeToData converts ResourceTypeRuntime to DataAsset.
func RuntimeToData(rtr *resourcetyperuntimepb.ResourceTypeRuntime) (*dataassetpb.DataAsset, error) {
	if rtr.GetData() == nil {
		return nil, fmt.Errorf("runtime does not have Data")
	}
	return rtr.GetData(), nil
}

// runtimeWithoutReferencesToHardwareDevice converts a ResourceTypeRuntime to a
// ProcessedHardwareDeviceManifest without reading the `referenced_assets`field,
// and instead is constructed by reading the scene object and service defs.
//
// This is required for compatibility with older platform versions that do not
// support the `referenced_assets` field in the ResourceTypeRuntime.
func runtimeWithoutReferencesToHardwareDevice(rtr *resourcetyperuntimepb.ResourceTypeRuntime) (map[string]*hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset, map[string]*assetgraphpb.AssetNode, error) {
	hwAssets := make(map[string]*hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset)
	// Create the scene_object asset in the hardware_device asset.
	sceneObject, err := RuntimeToSceneObject(rtr)
	if err != nil {
		return nil, nil, fmt.Errorf("failed to create scene object: %w", err)
	}
	sceneObject.Metadata.Id = &idpb.Id{
		Name:    rtr.GetMetadata().GetIdVersion().GetId().GetName() + "_scene_object",
		Package: rtr.GetMetadata().GetIdVersion().GetId().GetPackage(),
	}
	sceneObjectIDStr := idutils.IDFromProtoUnchecked(sceneObject.GetMetadata().GetId())
	hwAssets[sceneObjectIDStr] = &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset{
		Variant: &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset_SceneObject{
			SceneObject: sceneObject,
		},
	}

	// Create the service asset in the hardware_device asset.
	service, err := RuntimeToService(rtr)
	if err != nil {
		return nil, nil, fmt.Errorf("failed to create service: %w", err)
	}
	service.Metadata.Id = &idpb.Id{
		Name:    rtr.GetMetadata().GetIdVersion().GetId().GetName() + "_service",
		Package: rtr.GetMetadata().GetIdVersion().GetId().GetPackage(),
	}
	serviceIDStr := idutils.IDFromProtoUnchecked(service.GetMetadata().GetId())
	hwAssets[serviceIDStr] = &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset{
		Variant: &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Service{
			Service: service,
		},
	}
	hwdNodes := map[string]*assetgraphpb.AssetNode{
		"scene_object": {Asset: sceneObjectIDStr},
		"service":      {Asset: serviceIDStr},
	}
	return hwAssets, hwdNodes, nil
}

// RuntimeToHardwareDevice converts a ResourceTypeRuntime (RTR) to a ProcessedHardwareDeviceManifest (HWD).
// Note that the resulting HardwareDevice may NOT be identical to the original one that was
// used to create the ResourceTypeRuntime.
func RuntimeToHardwareDevice(rtr *resourcetyperuntimepb.ResourceTypeRuntime) (*hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest, error) {
	if rtr.GetSceneObject() == nil {
		return nil, fmt.Errorf("runtime is missing SceneObject")
	}
	if rtr.GetServiceDef() == nil {
		return nil, fmt.Errorf("runtime is missing Service definition")
	}

	phdm := &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest{
		Metadata: metadata.AssetMetadataToHardwareDeviceMetadata(rtr.GetMetadata()),
	}

	// Solutions running an older platform version will not have the referenced assets field
	// in the ResourceTypeRuntime. In that case, we need to create the hardware device
	// from the scene object and service definition in the runtime.
	if rtr.GetReferencedAssets() == nil {
		assets, nodes, err := runtimeWithoutReferencesToHardwareDevice(rtr)
		if err != nil {
			return nil, err
		}
		phdm.Assets = assets
		phdm.Graph = &assetgraphpb.AssetGraph{
			Nodes: nodes,
			Edges: []*assetgraphpb.AssetEdge{},
		}
	} else {
		hwAssets := make(map[string]*hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset)
		hwdNodes := make(map[string]*assetgraphpb.AssetNode)
		for id, asset := range rtr.GetReferencedAssets() {
			switch s := asset.GetSource().(type) {
			case *assetpb.Asset_Catalog:
				hwAssets[id] = &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset{
					Variant: &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Catalog{
						Catalog: s.Catalog,
					},
				}
				switch assetType := s.Catalog.GetAssetType(); assetType {
				case assettypepb.AssetType_ASSET_TYPE_SCENE_OBJECT:
					hwdNodes["scene_object"] = &assetgraphpb.AssetNode{Asset: id}
				case assettypepb.AssetType_ASSET_TYPE_SERVICE:
					hwdNodes["service"] = &assetgraphpb.AssetNode{Asset: id}
				case assettypepb.AssetType_ASSET_TYPE_DATA:
				default:
					return nil, fmt.Errorf("asset %q has type %T that is not supported in hardware device", id, assetType)
				}
			case *assetpb.Asset_Local:
				switch v := s.Local.GetVariant().(type) {
				case *processedassetpb.ProcessedAsset_SceneObject:
					hwAssets[id] = &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset{
						Variant: &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset_SceneObject{
							SceneObject: v.SceneObject,
						},
					}
					hwdNodes["scene_object"] = &assetgraphpb.AssetNode{Asset: id}
				case *processedassetpb.ProcessedAsset_Service:
					hwAssets[id] = &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset{
						Variant: &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Service{
							Service: v.Service,
						},
					}
					hwdNodes["service"] = &assetgraphpb.AssetNode{Asset: id}
				case *processedassetpb.ProcessedAsset_Data:
					hwAssets[id] = &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset{
						Variant: &hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Data{
							Data: v.Data,
						},
					}
				default:
					return nil, fmt.Errorf("asset %q has invalid variant type: %T", id, v)
				}
			default:
				return nil, fmt.Errorf("asset %q has unknown source type: %T", id, s)
			}
		}
		phdm.Assets = hwAssets
		phdm.Graph = &assetgraphpb.AssetGraph{
			Nodes: hwdNodes,
			Edges: []*assetgraphpb.AssetEdge{},
		}
	}

	return phdm, nil
}

// RuntimeToProcess converts ResourceTypeRuntime to ProcessAsset.
func RuntimeToProcess(rtr *resourcetyperuntimepb.ResourceTypeRuntime) (*processassetpb.ProcessAsset, error) {
	if rtr.GetProcess() == nil {
		return nil, fmt.Errorf("runtime does not have Process data")
	}
	process := rtr.GetProcess()
	if process.GetMetadata() == nil {
		process.Metadata = rtr.GetMetadata()
	}
	if len(rtr.GetReferencedAssets()) > 0 {
		process.Assets = make(map[string]*processassetpb.ProcessAsset_ProcessedAsset, len((rtr.GetReferencedAssets())))
		for refID, refAsset := range rtr.GetReferencedAssets() {
			switch v := refAsset.GetSource().(type) {
			case *assetpb.Asset_Catalog:
				process.Assets[refID] = &processassetpb.ProcessAsset_ProcessedAsset{
					Variant: &processassetpb.ProcessAsset_ProcessedAsset_Catalog{
						Catalog: v.Catalog,
					},
				}
			default:
				return nil, status.Errorf(codes.InvalidArgument, "unsupported Asset source type for Asset %q: %T", refID, v)
			}
		}
	}
	return process, nil
}

// RuntimeToSceneObject converts ResourceTypeRuntime to ProcessedSceneObjectManifest.
func RuntimeToSceneObject(rtr *resourcetyperuntimepb.ResourceTypeRuntime) (*sceneobjectmanifestpb.ProcessedSceneObjectManifest, error) {
	if rtr.GetSceneObject() == nil {
		return nil, fmt.Errorf("runtime does not have SceneObject data")
	}
	return &sceneobjectmanifestpb.ProcessedSceneObjectManifest{
		Metadata: metadata.AssetMetadataToSceneObjectMetadata(rtr.GetMetadata()),
		Assets: &sceneobjectmanifestpb.ProcessedSceneObjectAssets{
			DefaultSceneObjectConfig: rtr.GetDefaultSceneObjectConfig(),
			FileDescriptorSet:        rtr.GetMetadata().GetFileDescriptorSet(),
			SceneObjectModel:         rtr.GetSceneObject(),
		},
	}, nil
}

// RuntimeToService converts ResourceTypeRuntime to ProcessedServiceManifest.
func RuntimeToService(rtr *resourcetyperuntimepb.ResourceTypeRuntime) (*servicemanifestpb.ProcessedServiceManifest, error) {
	if rtr.GetServiceDef() == nil {
		return nil, fmt.Errorf("runtime does not have Service data")
	}
	sd, images, err := resourceServiceDefToServiceDef(rtr.GetServiceDef())
	if err != nil {
		return nil, err
	}
	return &servicemanifestpb.ProcessedServiceManifest{
		Metadata: metadata.AssetMetadataToServiceMetadata(rtr.GetMetadata()),
		Assets: &servicemanifestpb.ProcessedServiceAssets{
			DefaultConfiguration: rtr.GetDefaultConfiguration(),
			FileDescriptorSet:    rtr.GetMetadata().GetFileDescriptorSet(),
			Images:               images,
		},
		ServiceDef: sd,
	}, nil
}

// runtimeToSkill converts ResourceTypeRuntime to ProcessedSkillManifest.
func runtimeToSkill(rtr *resourcetyperuntimepb.ResourceTypeRuntime) (*processedskillmanifestpb.ProcessedSkillManifest, error) {
	if rtr.GetSkill() == nil {
		return nil, fmt.Errorf("runtime does not have Skill data")
	}
	skill := rtr.GetSkill()
	if skill.GetMetadata() == nil {
		skill.Metadata = metadata.AssetMetadataToProcessedSkillMetadata(rtr.GetMetadata())
	}
	if skill.Assets == nil {
		skill.Assets = &processedskillmanifestpb.ProcessedSkillAssets{}
		skill.Assets.FileDescriptorSet = rtr.GetMetadata().GetFileDescriptorSet()
	}
	return skill, nil
}

// Converts an Asset to a ResourceTypeRuntime.
// This is the inverse of [RuntimeToAsset].
func AssetToRuntime(ctx context.Context, asset *assetpb.Asset, options ...ToRuntimeOption) (*resourcetyperuntimepb.ResourceTypeRuntime, error) {
	if asset == nil || asset.GetLocal() == nil {
		return nil, fmt.Errorf("asset is nil or local is nil")
	}
	options = append(options,
		WithInstallationOrigin(installationoriginpb.InstallationOrigin_INSTALLATION_ORIGIN_SIDELOADED),
	)
	switch v := asset.GetLocal().GetVariant().(type) {
	case *processedassetpb.ProcessedAsset_Data:
		sideloadedVersion, err := SideloadedVersion(v.Data)
		if err != nil {
			return nil, fmt.Errorf("could not get sideloaded version of data: %w", err)
		}
		options = append(options, WithVersion(sideloadedVersion))
		return DataToRuntime(ctx, v.Data, options...)
	case *processedassetpb.ProcessedAsset_HardwareDevice:
		sideloadedVersion, err := SideloadedVersion(v.HardwareDevice)
		if err != nil {
			return nil, fmt.Errorf("could not get sideloaded version of hardware device: %w", err)
		}
		options = append(options, WithVersion(sideloadedVersion))
		return HardwareDeviceToRuntime(ctx, v.HardwareDevice, options...)
	case *processedassetpb.ProcessedAsset_Process:
		sideloadedVersion, err := SideloadedVersion(v.Process)
		if err != nil {
			return nil, fmt.Errorf("could not get sideloaded version of process: %w", err)
		}
		options = append(options, WithVersion(sideloadedVersion))
		return ProcessToRuntime(ctx, v.Process, options...)
	case *processedassetpb.ProcessedAsset_SceneObject:
		sideloadedVersion, err := SideloadedVersion(v.SceneObject)
		if err != nil {
			return nil, fmt.Errorf("could not get sideloaded version of scene object: %v", err)
		}
		options = append(options, WithVersion(sideloadedVersion))
		return SceneObjectToRuntime(ctx, v.SceneObject, options...)
	case *processedassetpb.ProcessedAsset_Service:
		sideloadedVersion, err := SideloadedVersion(v.Service)
		if err != nil {
			return nil, fmt.Errorf("could not get sideloaded version of service: %w", err)
		}
		options = append(options, WithVersion(sideloadedVersion))
		return ServiceToRuntime(ctx, v.Service, options...)
	case *processedassetpb.ProcessedAsset_Skill:
		sideloadedVersion, err := SideloadedVersion(v.Skill)
		if err != nil {
			return nil, fmt.Errorf("could not get sideloaded version of skill: %v", err)
		}
		options = append(options, WithVersion(sideloadedVersion))
		return SkillToRuntime(ctx, v.Skill, options...)
	default:
		return nil, fmt.Errorf("unknown asset variant: %T", v)
	}
}

// SkillToRuntime converts ProcessedServiceManifest to ResourceTypeRuntime.
func SkillToRuntime(ctx context.Context, skill *processedskillmanifestpb.ProcessedSkillManifest, options ...ToRuntimeOption) (*resourcetyperuntimepb.ResourceTypeRuntime, error) {
	opts, err := resolveToRuntimeOptions(skill, options)
	if err != nil {
		return nil, err
	}

	if !opts.skipValidation {
		if err := idutils.ValidateIDProto(skill.GetMetadata().GetId()); err != nil {
			return nil, status.Errorf(codes.InvalidArgument, "invalid manifest: %v", err)
		}
		if skill.GetMetadata().GetVendor().GetDisplayName() == "" {
			return nil, status.Error(codes.InvalidArgument, "vendor.display_name must be specified")
		}
		if err := validate.Image(skill.GetAssets().GetImage()); err != nil {
			return nil, status.Errorf(codes.InvalidArgument, "%v", err)
		}
	}

	rtr := &resourcetyperuntimepb.ResourceTypeRuntime{
		Metadata: &metadatapb.Metadata{
			IdVersion: &idpb.IdVersion{
				Id:      skill.GetMetadata().GetId(),
				Version: opts.version,
			},
			Vendor:        skill.GetMetadata().GetVendor(),
			Documentation: skill.GetMetadata().GetDocumentation(),
			DisplayName:   skill.GetMetadata().GetDisplayName(),
			AssetType:     assettypepb.AssetType_ASSET_TYPE_SKILL,
		},
		InstallationOrigin: opts.installationOrigin,
		Variant: &resourcetyperuntimepb.ResourceTypeRuntime_Skill{
			Skill: skill,
		},
	}

	if err := metadatautilsinternal.PopulateOutputOnlyFieldsForSkill(skill, rtr.GetMetadata()); err != nil {
		return nil, err
	}
	return rtr, nil
}

// ProcessToRuntime converts ProcessAsset to ResourceTypeRuntime.
func ProcessToRuntime(ctx context.Context, process *processassetpb.ProcessAsset, options ...ToRuntimeOption) (*resourcetyperuntimepb.ResourceTypeRuntime, error) {
	opts, err := resolveToRuntimeOptions(process, options)
	if err != nil {
		return nil, err
	}

	if !opts.skipValidation {
		if err := processvalidate.ProcessAsset(ctx, process); err != nil {
			return nil, err
		}
	}

	// Clone to prevent mutating the input
	cloned := proto.Clone(process).(*processassetpb.ProcessAsset)
	metadata := proto.Clone(process.GetMetadata()).(*metadatapb.Metadata)

	metadata.AssetType = assettypepb.AssetType_ASSET_TYPE_PROCESS

	if opts.version != "" {
		metadata.IdVersion.Version = opts.version
	}

	// There is an optional Skill proto at the top level of the BehaviorTree proto which contains
	// metadata derived from the asset's metadata. We have just "updated" the asset version so the
	// Skill proto needs to be updated as well (if present).
	processmanifest.FillInSkillIDVersionFromAssetMetadata(cloned.GetBehaviorTree().GetDescription(), metadata)

	if cloned.GetBehaviorTree().GetDescription() != nil && opts.installationOrigin == installationoriginpb.InstallationOrigin_INSTALLATION_ORIGIN_SIDELOADED {
		cloned.BehaviorTree.Description.Sideloaded = true
	}

	rtr := &resourcetyperuntimepb.ResourceTypeRuntime{
		Metadata:           metadata,
		InstallationOrigin: opts.installationOrigin,
		Variant: &resourcetyperuntimepb.ResourceTypeRuntime_Process{
			Process: cloned,
		},
	}

	if len(process.GetAssets()) > 0 {
		rtr.ReferencedAssets = make(map[string]*assetpb.Asset, len(process.GetAssets()))
		for refID, refAsset := range process.GetAssets() {
			switch v := refAsset.GetVariant().(type) {
			case *processassetpb.ProcessAsset_ProcessedAsset_Catalog:
				rtr.ReferencedAssets[refID] = &assetpb.Asset{
					Source: &assetpb.Asset_Catalog{
						Catalog: v.Catalog,
					},
				}
			default:
				return nil, status.Errorf(codes.InvalidArgument, "unsupported Asset variant for Asset %q: %T", refID, v)
			}
		}
	}

	if err := metadatautilsinternal.PopulateOutputOnlyFieldsForProcess(cloned, rtr.GetMetadata()); err != nil {
		return nil, err
	}

	return rtr, nil
}

// ServiceToRuntime converts ProcessedServiceManifest to ResourceTypeRuntime.
func ServiceToRuntime(ctx context.Context, service *servicemanifestpb.ProcessedServiceManifest, options ...ToRuntimeOption) (*resourcetyperuntimepb.ResourceTypeRuntime, error) {
	opts, err := resolveToRuntimeOptions(service, options)
	if err != nil {
		return nil, err
	}

	rsd, err := serviceDefToResourceServiceDef(service.GetServiceDef(), service.GetAssets().GetImages())
	if err != nil {
		return nil, status.Error(codes.InvalidArgument, "invalid service_def provided")
	}
	if rsd == nil {
		return nil, status.Error(codes.InvalidArgument, "service_def was not provided")
	}
	rtr := &resourcetyperuntimepb.ResourceTypeRuntime{
		Metadata: &metadatapb.Metadata{
			IdVersion: &idpb.IdVersion{
				Id:      service.GetMetadata().GetId(),
				Version: opts.version,
			},
			Vendor:        service.GetMetadata().GetVendor(),
			DisplayName:   service.GetMetadata().GetDisplayName(),
			Documentation: service.GetMetadata().GetDocumentation(),
			AssetType:     assettypepb.AssetType_ASSET_TYPE_SERVICE,
			AssetTag:      service.GetMetadata().GetAssetTag(),
		},
		InstallationOrigin:   opts.installationOrigin,
		DefaultConfiguration: service.GetAssets().GetDefaultConfiguration(),
		ServiceDef:           rsd,
	}
	if !opts.skipValidation {
		if err := validateResourceTypeRuntime(rtr); err != nil {
			return nil, status.Errorf(codes.InvalidArgument, "invalid manifest: %v", err)
		}
	}

	if err := metadatautilsinternal.PopulateOutputOnlyFieldsForService(service, rtr.GetMetadata()); err != nil {
		return nil, err
	}
	return rtr, nil
}

// SceneObjectToRuntime converts ProcessedSceneObjectManifest to ResourceTypeRuntime.
// It validates a processed service manifest and turns it
// into a ResourceTypeRuntime message that can be installed into the solution.
func SceneObjectToRuntime(ctx context.Context, sceneObject *sceneobjectmanifestpb.ProcessedSceneObjectManifest, options ...ToRuntimeOption) (*resourcetyperuntimepb.ResourceTypeRuntime, error) {
	opts, err := resolveToRuntimeOptions(sceneObject, options)
	if err != nil {
		return nil, err
	}

	rtr := &resourcetyperuntimepb.ResourceTypeRuntime{
		Metadata: &metadatapb.Metadata{
			IdVersion: &idpb.IdVersion{
				Id:      sceneObject.GetMetadata().GetId(),
				Version: opts.version,
			},
			Vendor:        sceneObject.GetMetadata().GetVendor(),
			DisplayName:   sceneObject.GetMetadata().GetDisplayName(),
			Documentation: sceneObject.GetMetadata().GetDocumentation(),
			AssetType:     assettypepb.AssetType_ASSET_TYPE_SCENE_OBJECT,
			AssetTag:      sceneObject.GetMetadata().GetAssetTag(),
		},
		InstallationOrigin:       opts.installationOrigin,
		DefaultSceneObjectConfig: sceneObject.GetAssets().GetDefaultSceneObjectConfig(),
		SceneObject:              sceneObject.GetAssets().GetSceneObjectModel(),
	}
	if !opts.skipValidation {
		if sceneObject.GetAssets().GetSceneObjectModel() == nil {
			return nil, status.Error(codes.InvalidArgument, "scene_object was not provided")
		}
		if err := sceneobjectvalidation.ValidateSceneObject(sceneObject.GetAssets().GetSceneObjectModel()); err != nil {
			// TODO - b/372759205: use extstatus APIs more directly when they can
			// set the status code and message more directly.
			status := status.Newf(codes.InvalidArgument, "scene object validation failed: %v", err)
			if withExt, err := status.WithDetails(
				es.New(errComponent, errCodeInvalidSceneObject,
					es.WithTitle("Scene Object validation failed"),
					es.WithUserMessage(fmt.Sprintf("Failed to install scene object: %v", sceneObject.GetMetadata().GetDisplayName())),
					es.WithDebugMessage(fmt.Sprintf("Scene Object: %v", sceneObject.GetAssets().GetSceneObjectModel())),
					es.WithContextFromError(err),
				).Proto(),
			); err == nil {
				status = withExt
			}
			return nil, status.Err()
		}

		if err := validateResourceTypeRuntime(rtr); err != nil {
			return nil, status.Errorf(codes.InvalidArgument, "invalid manifest: %v", err)
		}
	}

	if err := metadatautilsinternal.PopulateOutputOnlyFieldsForSceneObject(sceneObject, rtr.GetMetadata()); err != nil {
		return nil, err
	}
	return rtr, nil
}

// DataToRuntime converts DataAsset to ResourceTypeRuntime.
func DataToRuntime(ctx context.Context, data *dataassetpb.DataAsset, options ...ToRuntimeOption) (*resourcetyperuntimepb.ResourceTypeRuntime, error) {
	opts, err := resolveToRuntimeOptions(data, options)
	if err != nil {
		return nil, err
	}

	if !opts.skipValidation {
		if err := datavalidate.DataAsset(ctx, data,
			datavalidate.WithReferencedDataOptions(datavalidate.WithDisallowFileReferences(true)),
		); err != nil {
			return nil, status.Errorf(codes.InvalidArgument, "invalid DataAsset: %v", err)
		}
	}

	// Cloned to prevent mutating the input (e.g., in tests that call the service implementation
	// directly.)
	metadata := proto.Clone(data.GetMetadata()).(*metadatapb.Metadata)
	metadata.ReleaseNotes = ""
	metadata.UpdateTime = nil
	if opts.version != "" {
		metadata.GetIdVersion().Version = opts.version
	}
	metadata.AssetType = assettypepb.AssetType_ASSET_TYPE_DATA
	if err := metadatautilsinternal.PopulateOutputOnlyFieldsForData(data, metadata); err != nil {
		return nil, err
	}
	return &resourcetyperuntimepb.ResourceTypeRuntime{
		Metadata:           metadata,
		InstallationOrigin: opts.installationOrigin,
		Variant:            &resourcetyperuntimepb.ResourceTypeRuntime_Data{Data: data},
	}, nil
}

// HardwareDeviceToRuntime converts a ProcessedHardwareDeviceManifest (HWD) into a ResourceTypeRuntime (RTR).
// This is the inverse of [RuntimeToHardwareDevice].
func HardwareDeviceToRuntime(ctx context.Context, hdm *hardwaredevicemanifestpb.ProcessedHardwareDeviceManifest, options ...ToRuntimeOption) (*resourcetyperuntimepb.ResourceTypeRuntime, error) {
	opts, err := resolveToRuntimeOptions(hdm, options)
	if err != nil {
		return nil, err
	}

	var assetTag assettagpb.AssetTag
	if len(hdm.GetMetadata().GetAssetTags()) > 0 {
		assetTag = hdm.GetMetadata().GetAssetTags()[0]
	}
	rtr := &resourcetyperuntimepb.ResourceTypeRuntime{
		Metadata: &metadatapb.Metadata{
			IdVersion: &idpb.IdVersion{
				Id:      hdm.GetMetadata().GetId(),
				Version: opts.version,
			},
			Vendor:        hdm.GetMetadata().GetVendor(),
			DisplayName:   hdm.GetMetadata().GetDisplayName(),
			Documentation: hdm.GetMetadata().GetDocumentation(),
			AssetType:     assettypepb.AssetType_ASSET_TYPE_HARDWARE_DEVICE,
			AssetTag:      assetTag,
		},
		InstallationOrigin: opts.installationOrigin,
	}

	rtr.ReferencedAssets = make(map[string]*assetpb.Asset)
	for _, asset := range hdm.GetAssets() {
		var a *assetpb.Asset
		var id string

		switch v := asset.GetVariant().(type) {
		case *hdmpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Catalog:
			a = &assetpb.Asset{
				Source: &assetpb.Asset_Catalog{
					Catalog: &referencepb.CatalogAsset{
						AssetType: v.Catalog.GetAssetType(),
						IdVersion: v.Catalog.GetIdVersion(),
					},
				},
			}
			id = idutils.IDFromProtoUnchecked(v.Catalog.GetIdVersion().GetId())
		case *hdmpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Data:
			a = &assetpb.Asset{
				Source: &assetpb.Asset_Local{
					Local: &processedassetpb.ProcessedAsset{
						Variant: &processedassetpb.ProcessedAsset_Data{
							Data: v.Data,
						},
					},
				},
			}
			id = idutils.IDFromProtoUnchecked(v.Data.GetMetadata().GetIdVersion().GetId())
		case *hdmpb.ProcessedHardwareDeviceManifest_ProcessedAsset_SceneObject:
			a = &assetpb.Asset{
				Source: &assetpb.Asset_Local{
					Local: &processedassetpb.ProcessedAsset{
						Variant: &processedassetpb.ProcessedAsset_SceneObject{
							SceneObject: v.SceneObject,
						},
					},
				},
			}
			id = idutils.IDFromProtoUnchecked(v.SceneObject.GetMetadata().GetId())
		case *hdmpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Service:
			a = &assetpb.Asset{
				Source: &assetpb.Asset_Local{
					Local: &processedassetpb.ProcessedAsset{
						Variant: &processedassetpb.ProcessedAsset_Service{
							Service: v.Service,
						},
					},
				},
			}
			id = idutils.IDFromProtoUnchecked(v.Service.GetMetadata().GetId())
		default:
			return nil, status.Errorf(codes.InvalidArgument, "invalid asset variant %T referenced in the hardware device", v)
		}
		rtr.ReferencedAssets[id] = a
	}

	assets, err := graph.CollectAssets(ctx, hdm, &graph.CollectAssetsOptions{
		ACIClient: opts.aciClient,
	})
	if err != nil {
		return nil, status.Errorf(codes.InvalidArgument, "unable to collect assets: %v", err)
	}

	// Process the SceneObject part of the HardwareDevice.
	// ProcessedResourceManifest cannot represent multiple scene objects, so we must disallow
	// HardwareDevices with multiple scene objects.
	if len(assets.SceneObjects()) != 1 {
		return nil, status.Errorf(codes.InvalidArgument, "expected exactly one scene object, got %d", len(assets.SceneObjects()))
	}
	sceneObject := assets.SceneObjects()[0]
	if sceneObjectAssets := sceneObject.GetAssets(); assets != nil {
		rtr.SceneObject = sceneObjectAssets.GetSceneObjectModel()
		rtr.DefaultSceneObjectConfig = sceneObjectAssets.GetDefaultSceneObjectConfig()
	}

	// Process the Service part of the HardwareDevice.
	// ProcessedResourceManifest cannot represent multiple services, so we must disallow
	// HardwareDevices with multiple services.
	if len(assets.Services()) != 1 {
		return nil, status.Errorf(codes.InvalidArgument, "expected exactly one service, got %d", len(assets.Services()))
	}
	service := assets.Services()[0]
	rsd, err := serviceDefToResourceServiceDef(service.GetServiceDef(), service.GetAssets().GetImages())
	if err != nil {
		return nil, status.Error(codes.InvalidArgument, "invalid service_def provided")
	}
	if rsd == nil {
		return nil, status.Error(codes.InvalidArgument, "service_def was not provided")
	}
	rtr.ServiceDef = rsd
	if serviceAssets := service.GetAssets(); serviceAssets != nil {
		rtr.DefaultConfiguration = serviceAssets.GetDefaultConfiguration()
	}

	if !opts.skipValidation {
		if err := sceneobjectvalidation.ValidateSceneObject(sceneObject.GetAssets().GetSceneObjectModel()); err != nil {
			// TODO - b/372759205: use extstatus APIs more directly when they can
			// set the status code and message more directly.
			status := status.Newf(codes.InvalidArgument, "scene object validation failed: %v", err)
			if withExt, err := status.WithDetails(
				es.New(errComponent, errCodeInvalidSceneObject,
					es.WithTitle("Scene Object validation failed"),
					es.WithUserMessage(fmt.Sprintf("Failed to install scene object: %v", sceneObject.GetMetadata().GetDisplayName())),
					es.WithDebugMessage(fmt.Sprintf("Scene Object: %v", sceneObject.GetAssets().GetSceneObjectModel())),
					es.WithContextFromError(err),
				).Proto(),
			); err == nil {
				status = withExt
			}
			return nil, status.Err()
		}

		if err := validateResourceTypeRuntime(rtr); err != nil {
			return nil, status.Errorf(codes.InvalidArgument, "invalid manifest: %v", err)
		}
	}

	if err := metadatautilsinternal.PopulateOutputOnlyFieldsForHardwareDevice(assets, rtr.GetMetadata()); err != nil {
		return nil, err
	}
	return rtr, nil
}

// SideloadedVersion returns a version string for a sideloaded asset. The version includes build
// metadata based on a hash of the manifest to avoid conflicting entries within the database.
func SideloadedVersion(m proto.Message) (string, error) {
	manifestBytes, err := proto.MarshalOptions{Deterministic: true}.Marshal(m)
	if err != nil {
		return "", fmt.Errorf("could not marshal manifest: %v", err)
	}
	return fmt.Sprintf("0.0.1+%x", sha256.Sum256(manifestBytes)), nil
}

// serviceDefToResourceServiceDef transforms a ServiceDef proto into a ResourceServiceDefinition.
// It maps core service metadata and converts the Real and Sim pod specifications by
// resolving their associated container images from the provided images map.
func serviceDefToResourceServiceDef(sd *servicemanifestpb.ServiceDef, images map[string]*imagepb.Image) (*resourceservicedefinitionpb.ResourceServiceDefinition, error) {
	if sd == nil {
		return nil, nil
	}
	rsd := &resourceservicedefinitionpb.ResourceServiceDefinition{
		ServiceProtoPrefixes:           sd.GetServiceProtoPrefixes(),
		ConfigMessageFullName:          sd.GetConfigMessageFullName(),
		HttpConfig:                     sd.GetHttpConfig(),
		DynamicReconfigurationConfig:   sd.GetDynamicReconfigurationConfig(),
		ServiceStateConfig:             sd.GetServiceStateConfig(),
		ServiceInspectionConfig:        sd.GetServiceInspectionConfig(),
		RecommendedConfigurationConfig: sd.GetRecommendedConfigurationConfig(),
		SupportsDynamicReconfiguration: sd.GetSupportsDynamicReconfiguration(),
		SupportsServiceState:           sd.GetSupportsServiceState(),
	}

	var err error
	if realImage := sd.GetRealSpec().GetImage(); realImage != nil {
		rsd.RealSpec, err = servicePodSpecToResourceSpec(sd.GetRealSpec(), images)
		if err != nil {
			return nil, err
		}
	}
	if simImage := sd.GetSimSpec().GetImage(); simImage != nil {
		rsd.SimSpec, err = servicePodSpecToResourceSpec(sd.GetSimSpec(), images)
		if err != nil {
			return nil, err
		}
	}

	return rsd, nil
}

// resourceServiceDefToServiceDef converts a ResourceServiceDefinition proto into a ServiceDef.
// It extracts image information into the returned map and handles the conversion of
// Real and Sim resource specifications into pod specifications.
func resourceServiceDefToServiceDef(rsd *resourceservicedefinitionpb.ResourceServiceDefinition) (*servicemanifestpb.ServiceDef, map[string]*imagepb.Image, error) {
	if rsd == nil {
		return nil, nil, nil
	}
	images := make(map[string]*imagepb.Image)
	sd := &servicemanifestpb.ServiceDef{
		ServiceProtoPrefixes:           rsd.GetServiceProtoPrefixes(),
		ConfigMessageFullName:          rsd.GetConfigMessageFullName(),
		HttpConfig:                     rsd.GetHttpConfig(),
		DynamicReconfigurationConfig:   rsd.GetDynamicReconfigurationConfig(),
		ServiceStateConfig:             rsd.GetServiceStateConfig(),
		ServiceInspectionConfig:        rsd.GetServiceInspectionConfig(),
		SupportsDynamicReconfiguration: rsd.GetSupportsDynamicReconfiguration(),
		SupportsServiceState:           rsd.GetSupportsServiceState(),
	}

	var err error
	if realImage := rsd.GetRealSpec().GetImage(); realImage != nil {
		sd.RealSpec, err = resourceSpecToServicePodSpec(rsd.GetRealSpec(), images)
		if err != nil {
			return nil, nil, err
		}
	}
	if simImage := rsd.GetSimSpec().GetImage(); simImage != nil {
		sd.SimSpec, err = resourceSpecToServicePodSpec(rsd.GetSimSpec(), images)
		if err != nil {
			return nil, nil, err
		}
	}
	if len(images) == 0 {
		images = nil
	}
	return sd, images, nil
}

func serviceToResourceCapabilities(s *servicepermissionspb.PosixCapabilities) (*resourcepermissionspb.Capabilities, error) {
	if s == nil {
		return nil, nil
	}
	var add []string
	for _, a := range s.GetAdd() {
		switch a {
		case servicepermissionspb.PosixCapability_POSIX_CAPABILITY_SYS_RAWIO:
			add = append(add, "SYS_RAWIO")
		case servicepermissionspb.PosixCapability_POSIX_CAPABILITY_SYS_NICE:
			add = append(add, "SYS_NICE")
		case servicepermissionspb.PosixCapability_POSIX_CAPABILITY_IPC_LOCK:
			add = append(add, "IPC_LOCK")
		default:
			return nil, fmt.Errorf("unknown service capability enum: %v", a)
		}
	}
	return &resourcepermissionspb.Capabilities{
		Add: add,
	}, nil
}

func resourceCapabilitiesToServiceCapabilities(r *resourcepermissionspb.Capabilities) (*servicepermissionspb.PosixCapabilities, error) {
	if r == nil {
		return nil, nil
	}
	var add []servicepermissionspb.PosixCapability
	for _, a := range r.GetAdd() {
		switch a {
		case "SYS_RAWIO":
			add = append(add, servicepermissionspb.PosixCapability_POSIX_CAPABILITY_SYS_RAWIO)
		case "SYS_NICE":
			add = append(add, servicepermissionspb.PosixCapability_POSIX_CAPABILITY_SYS_NICE)
		case "IPC_LOCK":
			add = append(add, servicepermissionspb.PosixCapability_POSIX_CAPABILITY_IPC_LOCK)
		default:
			return nil, fmt.Errorf("unknown resource capability: %v", a)
		}
	}
	return &servicepermissionspb.PosixCapabilities{
		Add: add,
	}, nil
}

func serviceToResourceResourceRequirements(s *servicepermissionspb.ResourceRequirements) *resourcepermissionspb.ResourceRequirements {
	if s == nil {
		return nil
	}
	limits := map[string]string{}
	maps.Copy(limits, s.GetLimits())
	requests := map[string]string{}
	maps.Copy(requests, s.GetRequests())
	return &resourcepermissionspb.ResourceRequirements{Limits: limits, Requests: requests}
}

func resourceResourceRequirementsToServiceResourceRequirements(r *resourcepermissionspb.ResourceRequirements) *servicepermissionspb.ResourceRequirements {
	if r == nil {
		return nil
	}
	limits := map[string]string{}
	maps.Copy(limits, r.GetLimits())
	requests := map[string]string{}
	maps.Copy(requests, r.GetRequests())
	return &servicepermissionspb.ResourceRequirements{Limits: limits, Requests: requests}
}

func serviceToResourceSecurityContext(s *servicepermissionspb.SecurityContext) (*resourcepermissionspb.SecurityContext, error) {
	if s == nil {
		return nil, nil
	}

	capabilities, err := serviceToResourceCapabilities(s.GetPosixCapabilities())
	if err != nil {
		return nil, fmt.Errorf("invalid capabilites: %w", err)
	}
	return &resourcepermissionspb.SecurityContext{
		Capabilities: capabilities,
		Privileged:   s.GetPrivileged(),
	}, nil
}

func resourceSecurityContextToServiceSecurityContext(r *resourcepermissionspb.SecurityContext) (*servicepermissionspb.SecurityContext, error) {
	if r == nil {
		return nil, nil
	}

	capabilities, err := resourceCapabilitiesToServiceCapabilities(r.GetCapabilities())
	if err != nil {
		return nil, fmt.Errorf("invalid capabilites: %w", err)
	}
	return &servicepermissionspb.SecurityContext{
		PosixCapabilities: capabilities,
		Privileged:        r.GetPrivileged(),
	}, nil
}

func serviceToResourceVolumeMount(s *servicevolumepb.VolumeMount) *resourcevolumepb.VolumeMount {
	if s == nil {
		return nil
	}
	return &resourcevolumepb.VolumeMount{
		Name:      s.GetName(),
		MountPath: s.GetMountPath(),
		ReadOnly:  s.GetReadOnly(),
	}
}

func resourceVolumeMountToServiceVolumeMount(r *resourcevolumepb.VolumeMount) *servicevolumepb.VolumeMount {
	if r == nil {
		return nil
	}
	return &servicevolumepb.VolumeMount{
		Name:      r.GetName(),
		MountPath: r.GetMountPath(),
		ReadOnly:  r.GetReadOnly(),
	}
}

func serviceImageToResourceImageSettings(s *servicemanifestpb.ServiceImage, imageMap map[string]*imagepb.Image) (*resourceservicedefinitionpb.ImageWithSettings, error) {
	if s == nil {
		return nil, nil
	}

	// Convert the service's volume mounts to resource volume mounts.
	var volumeMounts []*resourcevolumepb.VolumeMount
	for _, vm := range s.GetSettings().GetVolumeMounts() {
		volumeMounts = append(volumeMounts, serviceToResourceVolumeMount(vm))
	}

	// Convert the service's security context to a resource security context.
	securityContext, err := serviceToResourceSecurityContext(s.GetSettings().GetSecurityContext())
	if err != nil {
		return nil, fmt.Errorf("invalid security_context: %w", err)
	}

	// Convert the service's requirements to a resource requirement.
	resources := serviceToResourceResourceRequirements(s.GetSettings().GetResourceRequirements())

	var envVars []*resourceservicedefinitionpb.EnvVar
	for _, ev := range s.GetSettings().GetEnvVars() {
		envVars = append(envVars, &resourceservicedefinitionpb.EnvVar{
			Name:  ev.GetName(),
			Value: ev.GetValue(),
		})
	}

	image, exists := imageMap[s.GetArchiveFilename()]
	if !exists {
		return nil, fmt.Errorf("image %q not found in images map", s.GetArchiveFilename())
	}

	settings := &resourceservicedefinitionpb.ImageWithSettings{
		Image:            image,
		VolumeMounts:     volumeMounts,
		SecurityContext:  securityContext,
		Resources:        resources,
		RequiresRtpcNode: s.GetSettings().GetRequiresRtpcNode(),
		EnvVars:          envVars,
		Args:             s.GetSettings().GetArgs(),
	}
	if proto.Size(settings) == 0 {
		settings = nil
	}
	return settings, nil
}

func resourceImageSettingsToServiceImage(r *resourceservicedefinitionpb.ImageWithSettings) (*servicemanifestpb.ServiceImage, error) {
	if r == nil {
		return nil, nil
	}

	// Convert the resource's volume mounts to service volume mounts.
	var volumeMounts []*servicevolumepb.VolumeMount
	for _, vm := range r.GetVolumeMounts() {
		volumeMounts = append(volumeMounts, resourceVolumeMountToServiceVolumeMount(vm))
	}

	// Convert the resource's security context to a service security context.
	securityContext, err := resourceSecurityContextToServiceSecurityContext(r.GetSecurityContext())
	if err != nil {
		return nil, fmt.Errorf("invalid security_context: %w", err)
	}

	// Convert the resource's requirements to a service requirement.
	resources := resourceResourceRequirementsToServiceResourceRequirements(r.GetResources())

	var envVars []*envpb.EnvVar
	for _, ev := range r.GetEnvVars() {
		envVars = append(envVars, &envpb.EnvVar{
			Name:  ev.GetName(),
			Value: ev.GetValue(),
		})
	}

	settings := &servicemanifestpb.ServiceImageSettings{
		VolumeMounts:         volumeMounts,
		SecurityContext:      securityContext,
		ResourceRequirements: resources,
		RequiresRtpcNode:     r.GetRequiresRtpcNode(),
		Args:                 r.GetArgs(),
		EnvVars:              envVars,
	}
	if proto.Size(settings) == 0 {
		settings = nil
	}
	return &servicemanifestpb.ServiceImage{
		Settings:        settings,
		ArchiveFilename: r.GetImage().GetName(),
	}, nil
}

func serviceVolumeToResourceVolume(sv *servicevolumepb.Volume) (*resourcevolumepb.Volume, error) {
	if sv == nil {
		return nil, nil
	}
	switch s := sv.GetSource().(type) {
	case *servicevolumepb.Volume_HostPath:
		rv := &resourcevolumepb.Volume{
			Name: sv.GetName(),
			Source: &resourcevolumepb.Volume_HostPath{
				HostPath: &resourcevolumepb.HostPathVolumeSource{
					Path: s.HostPath.GetPath(),
				},
			},
		}
		switch s.HostPath.GetType() {
		case servicevolumepb.HostPathVolumeSourceType_HOST_PATH_VOLUME_SOURCE_TYPE_CHAR_DEVICE:
			rv.GetHostPath().Type = "CharDevice"
		case servicevolumepb.HostPathVolumeSourceType_HOST_PATH_VOLUME_SOURCE_TYPE_UNSPECIFIED:
			// Unset/unspecified type remains valid to stay compatible with the old behavior and proto.
		default:
			return nil, fmt.Errorf("unknown host path type in %v", s.HostPath.GetType())
		}
		return rv, nil
	case *servicevolumepb.Volume_EmptyDir:
		rv := &resourcevolumepb.Volume{
			Name: sv.GetName(),
			Source: &resourcevolumepb.Volume_EmptyDir{
				EmptyDir: &resourcevolumepb.EmptyDirVolumeSource{},
			},
		}
		switch s.EmptyDir.GetMedium() {
		case servicevolumepb.EmptyDirMedium_EMPTY_DIR_MEDIUM_MEMORY:
			rv.GetEmptyDir().Medium = "Memory"
		case servicevolumepb.EmptyDirMedium_EMPTY_DIR_MEDIUM_UNSPECIFIED:
		}
		return rv, nil
	default:
		return nil, fmt.Errorf("unknown volume source in %v", sv.GetSource())
	}
}

func resourceVolumeToServiceVolume(rv *resourcevolumepb.Volume) (*servicevolumepb.Volume, error) {
	if rv == nil {
		return nil, nil
	}

	switch s := rv.GetSource().(type) {
	case *resourcevolumepb.Volume_HostPath:
		sv := &servicevolumepb.Volume{
			Name: rv.GetName(),
			Source: &servicevolumepb.Volume_HostPath{
				HostPath: &servicevolumepb.HostPathVolumeSource{
					Path: s.HostPath.GetPath(),
				},
			},
		}
		switch s.HostPath.GetType() {
		case "CharDevice":
			sv.GetHostPath().Type = servicevolumepb.HostPathVolumeSourceType_HOST_PATH_VOLUME_SOURCE_TYPE_CHAR_DEVICE.Enum()
		case "":
			// Unset type remains valid to stay compatible with the old behavior and proto.
		default:
			return nil, fmt.Errorf("unknown host path type %q", s.HostPath.GetType())
		}
		return sv, nil
	case *resourcevolumepb.Volume_EmptyDir:
		sv := &servicevolumepb.Volume{
			Name: rv.GetName(),
			Source: &servicevolumepb.Volume_EmptyDir{
				EmptyDir: &servicevolumepb.EmptyDirVolumeSource{},
			},
		}
		switch s.EmptyDir.GetMedium() {
		case "Memory":
			sv.GetEmptyDir().Medium = servicevolumepb.EmptyDirMedium_EMPTY_DIR_MEDIUM_MEMORY
		case "":
		default:
			return nil, fmt.Errorf("unknown empty dir medium %v", s.EmptyDir.GetMedium())
		}
		return sv, nil
	default:
		return nil, fmt.Errorf("unknown volume source in %v", rv.GetSource())
	}
}

func serviceVolumesToResourceVolumes(svs []*servicevolumepb.Volume) ([]*resourcevolumepb.Volume, error) {
	if svs == nil {
		return nil, nil
	}

	var rvs []*resourcevolumepb.Volume
	for _, sv := range svs {
		rv, err := serviceVolumeToResourceVolume(sv)
		if err != nil {
			return nil, fmt.Errorf("invalid volume %q in pod settings: %v", sv.GetName(), err)
		}
		rvs = append(rvs, rv)
	}
	return rvs, nil
}

func settingsFromResourceSpec(r *resourceservicedefinitionpb.ResourceSpec) (*servicemanifestpb.ServicePodSettings, error) {
	if r == nil {
		return nil, nil
	}

	var svs []*servicevolumepb.Volume
	for _, rv := range r.GetVolumes() {
		sv, err := resourceVolumeToServiceVolume(rv)
		if err != nil {
			return nil, fmt.Errorf("invalid volume %q in pod settings: %v", rv.GetName(), err)
		}
		svs = append(svs, sv)
	}

	settings := &servicemanifestpb.ServicePodSettings{
		HostNetwork:     r.GetHostNetwork(),
		Volumes:         svs,
		SecurityContext: r.GetSecurityContext(),
	}
	if proto.Size(settings) == 0 {
		settings = nil
	}
	return settings, nil
}

func servicePodSpecToResourceSpec(s *servicemanifestpb.ServicePodSpec, imageMap map[string]*imagepb.Image) (*resourceservicedefinitionpb.ResourceSpec, error) {
	if s == nil {
		return nil, nil
	}
	rvs, err := serviceVolumesToResourceVolumes(s.GetSettings().GetVolumes())
	if err != nil {
		return nil, err
	}
	imageSettings, err := serviceImageToResourceImageSettings(s.GetImage(), imageMap)
	if err != nil {
		return nil, err
	}

	spec := &resourceservicedefinitionpb.ResourceSpec{
		Image:           []*resourceservicedefinitionpb.ImageWithSettings{imageSettings},
		Volumes:         rvs,
		HostNetwork:     s.GetSettings().GetHostNetwork(),
		SecurityContext: s.GetSettings().GetSecurityContext(),
	}
	for _, image := range s.GetExtraImages() {
		imageSettings, err := serviceImageToResourceImageSettings(image, imageMap)
		if err != nil {
			return nil, err
		}
		spec.Image = append(spec.Image, imageSettings)
	}
	for _, image := range s.GetInitContainers() {
		imageSettings, err := serviceImageToResourceImageSettings(image, imageMap)
		if err != nil {
			return nil, err
		}
		spec.InitContainers = append(spec.InitContainers, imageSettings)
	}

	return spec, nil
}

func resourceSpecToServicePodSpec(r *resourceservicedefinitionpb.ResourceSpec, imageMap map[string]*imagepb.Image) (*servicemanifestpb.ServicePodSpec, error) {
	if r == nil {
		return nil, nil
	}

	settings, err := settingsFromResourceSpec(r)
	if err != nil {
		return nil, err
	}
	spec := &servicemanifestpb.ServicePodSpec{
		Settings: settings,
	}
	if len(r.GetImage()) != 0 {
		serviceImage, err := resourceImageSettingsToServiceImage(r.GetImage()[0])
		if err != nil {
			return nil, err
		}
		spec.Image = serviceImage
		imageMap[serviceImage.GetArchiveFilename()] = r.GetImage()[0].GetImage()
	}
	if len(r.GetImage()) > 1 {
		for _, imageWithSettings := range r.GetImage()[1:] {
			serviceImage, err := resourceImageSettingsToServiceImage(imageWithSettings)
			if err != nil {
				return nil, err
			}
			spec.ExtraImages = append(spec.ExtraImages, serviceImage)
			imageMap[serviceImage.GetArchiveFilename()] = imageWithSettings.GetImage()
		}
	}
	for _, imageWithSettings := range r.GetInitContainers() {
		initImage, err := resourceImageSettingsToServiceImage(imageWithSettings)
		if err != nil {
			return nil, err
		}
		spec.InitContainers = append(spec.InitContainers, initImage)
		imageMap[initImage.GetArchiveFilename()] = imageWithSettings.GetImage()
	}
	return spec, nil
}

// inferRuntimeAssetType attempts to infer the AssetType of a ResourceTypeRuntime based on the
// presence of its component fields.
//
// It can be used to provide a helpful hint in error messages when the AssetType is unspecified in
// the metadata.
func inferRuntimeAssetType(rtr *resourcetyperuntimepb.ResourceTypeRuntime) assettypepb.AssetType {
	if at := rtr.GetMetadata().GetAssetType(); at != assettypepb.AssetType_ASSET_TYPE_UNSPECIFIED {
		return at
	}

	type component int
	const (
		cData component = iota
		cProcess
		cSceneObject
		cService
		cSkill
	)

	hasComponents := map[component]bool{
		cData:        rtr.GetData() != nil,
		cProcess:     rtr.GetProcess() != nil,
		cSceneObject: rtr.GetSceneObject() != nil,
		cService:     rtr.GetServiceDef() != nil,
		cSkill:       rtr.GetSkill() != nil,
	}

	if isExclusively(hasComponents, cData) {
		return assettypepb.AssetType_ASSET_TYPE_DATA
	}
	if isExclusively(removeKey(hasComponents, cData), cSceneObject, cService) {
		return assettypepb.AssetType_ASSET_TYPE_HARDWARE_DEVICE
	}
	if isExclusively(hasComponents, cProcess) {
		return assettypepb.AssetType_ASSET_TYPE_PROCESS
	}
	if isExclusively(hasComponents, cSceneObject) {
		return assettypepb.AssetType_ASSET_TYPE_SCENE_OBJECT
	}
	if isExclusively(hasComponents, cService) {
		return assettypepb.AssetType_ASSET_TYPE_SERVICE
	}
	if isExclusively(hasComponents, cSkill) {
		return assettypepb.AssetType_ASSET_TYPE_SKILL
	}

	return assettypepb.AssetType_ASSET_TYPE_UNSPECIFIED
}

func isExclusively[K comparable](has map[K]bool, comps ...K) bool {
	for c, b := range has {
		inComps := slices.Contains(comps, c)
		if (!b && inComps) || (b && !inComps) {
			return false
		}
	}
	return true
}

func removeKey[K comparable, V any](m map[K]V, k K) map[K]V {
	mc := maps.Clone(m)
	delete(mc, k)
	return mc
}

func findAssetID(rtr *resourcetyperuntimepb.ResourceTypeRuntime) string {
	id, err := idutils.IDFromProto(rtr.GetMetadata().GetIdVersion().GetId())
	if err == nil {
		return id
	}
	return "Asset"
}
