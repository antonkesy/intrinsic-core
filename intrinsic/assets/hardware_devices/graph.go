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

// Package graph contains utilities for working with HardwareDevice graphs.
package graph

import (
	"context"
	"fmt"

	"intrinsic/kubernetes/acl/clientcontext"
	"intrinsic/util/proto/names"

	"google.golang.org/grpc"
	"google.golang.org/protobuf/proto"

	acipb "intrinsic/assets/catalog/proto/v1/asset_catalog_internal_go_proto"
	dapb "intrinsic/assets/data/proto/v1/data_asset_go_proto"
	hdmpb "intrinsic/assets/hardware_devices/proto/v1/hardware_device_manifest_go_proto"
	atpb "intrinsic/assets/proto/asset_type_go_proto"
	idpb "intrinsic/assets/proto/id_go_proto"
	agpb "intrinsic/assets/proto/v1/asset_graph_go_proto"
	rpb "intrinsic/assets/proto/v1/reference_go_proto"
	avpb "intrinsic/assets/proto/view_go_proto"
	sompb "intrinsic/assets/scene_objects/proto/scene_object_manifest_go_proto"
	smpb "intrinsic/assets/services/proto/service_manifest_go_proto"
)

// AssetCatalogInternalClient is an interface for the part of the AssetCatalogInternal service that
// is used by this package.
type AssetCatalogInternalClient interface {
	BatchGetAssetsInternal(ctx context.Context, req *acipb.BatchGetAssetsRequest, opts ...grpc.CallOption) (*acipb.BatchGetAssetsResponse, error)
}

// CollectAssetsOptions contains options for a call to CollectAssets.
type CollectAssetsOptions struct {
	// ACIClient is the catalog client to use to resolve catalog references.
	ACIClient AssetCatalogInternalClient
	// IgnoreUnavailableAssets specifies whether to ignore unavailable referenced catalog Assets.
	//
	// If true, unavailable Assets will be omitted from the returned collection.
	IgnoreUnavailableAssets bool
}

// CollectedAssets represents the assets that have been extracted from a
// ProcessedHardwareDeviceManifest.
type CollectedAssets struct {
	// DataMap is a map of Data assets included in the HardwareDevice that did not take part in a
	// configuration edge.
	DataMap map[string]*dapb.DataAsset
	// SceneObjectsMap is a map of SceneObjects included in the HardwareDevice.
	SceneObjectsMap map[string]*sompb.ProcessedSceneObjectManifest
	// Services is a map of Service assets included in the HardwareDevice.
	// If a Data asset has a Configuration edge to a Service, then its payload will have replaced the
	// default configuration of the Service.
	ServicesMap map[string]*smpb.ProcessedServiceManifest
}

// Services returns the Services included in the HardwareDevice.
func (a *CollectedAssets) Services() []*smpb.ProcessedServiceManifest {
	var services []*smpb.ProcessedServiceManifest
	for _, service := range a.ServicesMap {
		services = append(services, service)
	}
	return services
}

// SceneObjects returns the SceneObjects included in the HardwareDevice.
func (a *CollectedAssets) SceneObjects() []*sompb.ProcessedSceneObjectManifest {
	var sceneObjects []*sompb.ProcessedSceneObjectManifest
	for _, sceneObject := range a.SceneObjectsMap {
		sceneObjects = append(sceneObjects, sceneObject)
	}
	return sceneObjects
}

// Data returns the Data assets included in the HardwareDevice.
func (a *CollectedAssets) Data() []*dapb.DataAsset {
	var data []*dapb.DataAsset
	for _, d := range a.DataMap {
		data = append(data, d)
	}
	return data
}

// CollectAssets collects the assets from a ProcessedHardwareDeviceManifest.
func CollectAssets(ctx context.Context, hdm *hdmpb.ProcessedHardwareDeviceManifest, opts *CollectAssetsOptions) (*CollectedAssets, error) {
	assets := &CollectedAssets{
		DataMap:         make(map[string]*dapb.DataAsset),
		SceneObjectsMap: make(map[string]*sompb.ProcessedSceneObjectManifest),
		ServicesMap:     make(map[string]*smpb.ProcessedServiceManifest),
	}

	// Collect the local assets first. Keep track of the catalog assets so we can fetch them as a
	// batch below.
	catalogAssets := make(map[string]*rpb.CatalogAsset)
	for key, asset := range hdm.GetAssets() {
		// Note that we make copies to avoid mutating the input.
		switch asset.Variant.(type) {
		case *hdmpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Catalog:
			catalogAssets[key] = proto.Clone(asset.GetCatalog()).(*rpb.CatalogAsset)
		case *hdmpb.ProcessedHardwareDeviceManifest_ProcessedAsset_SceneObject:
			assets.SceneObjectsMap[key] = proto.Clone(asset.GetSceneObject()).(*sompb.ProcessedSceneObjectManifest)
		case *hdmpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Service:
			assets.ServicesMap[key] = proto.Clone(asset.GetService()).(*smpb.ProcessedServiceManifest)
		case *hdmpb.ProcessedHardwareDeviceManifest_ProcessedAsset_Data:
			assets.DataMap[key] = proto.Clone(asset.GetData()).(*dapb.DataAsset)
		}
	}

	// Fetch the catalog assets.
	if len(catalogAssets) > 0 {
		if opts.ACIClient == nil {
			return nil, fmt.Errorf("cannot collect catalog assets, since no asset catalog client was provided")
		}

		outgoingCtx, err := clientcontext.ToContextFromIncoming(ctx)
		if err != nil {
			return nil, clientcontext.ErrGRPC(err)
		}
		keys := make([]string, 0, len(catalogAssets))
		idVersions := make([]*idpb.IdVersion, 0, len(catalogAssets))
		assetTypes := make([]atpb.AssetType, 0, len(catalogAssets))
		for key, asset := range catalogAssets {
			keys = append(keys, key)
			idVersions = append(idVersions, asset.GetIdVersion())
			assetTypes = append(assetTypes, asset.GetAssetType())
		}
		resp, err := opts.ACIClient.BatchGetAssetsInternal(outgoingCtx, &acipb.BatchGetAssetsRequest{
			IdVersions:      idVersions,
			View:            avpb.AssetViewType_ASSET_VIEW_TYPE_ALL,
			SkipUnavailable: opts.IgnoreUnavailableAssets,
		})
		if err != nil {
			return nil, fmt.Errorf("failed to get assets from catalog: %w", err)
		}
		for i, asset := range resp.GetAssets() {
			assetType := assetTypes[i]
			key := keys[i]
			switch assetType {
			case atpb.AssetType_ASSET_TYPE_SCENE_OBJECT:
				assets.SceneObjectsMap[key] = asset.GetDeploymentData().GetSceneObjectSpecificDeploymentData().GetManifest()
			case atpb.AssetType_ASSET_TYPE_SERVICE:
				assets.ServicesMap[key] = asset.GetDeploymentData().GetServiceSpecificDeploymentData().GetManifest()
			case atpb.AssetType_ASSET_TYPE_DATA:
				assets.DataMap[key] = asset.GetDeploymentData().GetDataSpecificDeploymentData().GetData()
			default:
				return nil, fmt.Errorf("asset has unknown type %v", assetType)
			}
		}
	}

	// Apply configuration edges to the Services.
	for _, edge := range hdm.GetGraph().GetEdges() {
		switch edge.GetEdgeType().(type) {
		case *agpb.AssetEdge_Configures:
			source := edge.GetSource()
			sourceNode, ok := hdm.GetGraph().GetNodes()[source]
			if !ok {
				return nil, fmt.Errorf("configuration edge from unknown source %q", source)
			}
			sourceNodeID := sourceNode.GetAsset()
			sourceData, ok := assets.DataMap[sourceNodeID]
			if !ok {
				return nil, fmt.Errorf("configuration edge from wrong type or unknown data asset %q", sourceNodeID)
			}
			target := edge.GetTarget()
			targetNode, ok := hdm.GetGraph().GetNodes()[target]
			if !ok {
				return nil, fmt.Errorf("configuration edge to unknown target %q", target)
			}
			targetNodeID := targetNode.GetAsset()
			targetService, ok := assets.ServicesMap[targetNodeID]
			if !ok {
				return nil, fmt.Errorf("configuration edge to wrong type or unknown service %q", targetNodeID)
			}

			// Verify that the Data asset is compatible with the Service's configuration.
			serviceConfigName := targetService.GetServiceDef().GetConfigMessageFullName()
			dataName, err := names.AnyToProtoName(sourceData.GetData())
			if err != nil {
				return nil, fmt.Errorf("failed to get payload proto name: %w", err)
			}
			if serviceConfigName != dataName {
				return nil, fmt.Errorf("configuration edge from Data asset %q to Service %q has unexpected configuration type (got %q, expected %q)", sourceNodeID, targetNodeID, dataName, serviceConfigName)
			}

			// Save the Data asset's payload as the Service's new default configuration.
			targetService.GetAssets().DefaultConfiguration = sourceData.GetData()
		default:
			return nil, fmt.Errorf("unsupported edge type %v", edge.GetEdgeType())
		}
	}

	return assets, nil
}
