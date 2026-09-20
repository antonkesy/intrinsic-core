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

// Package applicationasset implements conversion of catalog application assets to catalog assets, and vice-versa.
// The converted values of the assets are not stored, they are only used to
// output a `Solution` message and easily diff/merge two `Solution` versions.
package applicationasset

import (
	"context"
	"fmt"

	acpb "intrinsic/assets/catalog/proto/v1/asset_catalog_go_proto"
	assetpb "intrinsic/assets/proto/v1/asset_go_proto"
	processedassetpb "intrinsic/assets/proto/v1/processed_asset_go_proto"
	referencepb "intrinsic/assets/proto/v1/reference_go_proto"
	applicationpb "intrinsic/config/proto/application_go_proto"
)

// ApplicationAssetToAsset converts an application asset to an asset.
func ApplicationAssetToAsset(ctx context.Context, applicationAsset *applicationpb.Application_Asset, catalogC acpb.AssetCatalogClient) (*assetpb.Asset, error) {
	if applicationAsset == nil {
		return nil, fmt.Errorf("application asset is nil")
	}

	asset := &assetpb.Asset{}
	switch applicationAsset.GetVariant().(type) {
	case *applicationpb.Application_Asset_Catalog:
		req := &acpb.GetAssetRequest{
			AssetId: &acpb.GetAssetRequest_IdVersion{
				IdVersion: applicationAsset.GetCatalog(),
			},
		}
		assetFromCatalog, err := catalogC.GetAsset(ctx, req)
		if err != nil {
			return nil, fmt.Errorf("failed to get asset from catalog: %w", err)
		}
		asset.Source = &assetpb.Asset_Catalog{
			Catalog: &referencepb.CatalogAsset{
				IdVersion: applicationAsset.GetCatalog(),
				AssetType: assetFromCatalog.GetMetadata().GetAssetType(),
			},
		}
	case *applicationpb.Application_Asset_Service:
		asset.Source = &assetpb.Asset_Local{
			Local: &processedassetpb.ProcessedAsset{
				Variant: &processedassetpb.ProcessedAsset_Service{
					Service: applicationAsset.GetService(),
				},
			},
		}
	case *applicationpb.Application_Asset_Skill:
		asset.Source = &assetpb.Asset_Local{
			Local: &processedassetpb.ProcessedAsset{
				Variant: &processedassetpb.ProcessedAsset_Skill{
					Skill: applicationAsset.GetSkill(),
				},
			},
		}
	case *applicationpb.Application_Asset_Data:
		asset.Source = &assetpb.Asset_Local{
			Local: &processedassetpb.ProcessedAsset{
				Variant: &processedassetpb.ProcessedAsset_Data{
					Data: applicationAsset.GetData(),
				},
			},
		}
	case *applicationpb.Application_Asset_SceneObject:
		asset.Source = &assetpb.Asset_Local{
			Local: &processedassetpb.ProcessedAsset{
				Variant: &processedassetpb.ProcessedAsset_SceneObject{
					SceneObject: applicationAsset.GetSceneObject(),
				},
			},
		}
	case *applicationpb.Application_Asset_HardwareDevice:
		asset.Source = &assetpb.Asset_Local{
			Local: &processedassetpb.ProcessedAsset{
				Variant: &processedassetpb.ProcessedAsset_HardwareDevice{
					HardwareDevice: applicationAsset.GetHardwareDevice(),
				},
			},
		}
	case *applicationpb.Application_Asset_Process:
		asset.Source = &assetpb.Asset_Local{
			Local: &processedassetpb.ProcessedAsset{
				Variant: &processedassetpb.ProcessedAsset_Process{
					Process: applicationAsset.GetProcess(),
				},
			},
		}
	default:
		return nil, fmt.Errorf("unknown application asset variant")
	}

	return asset, nil
}

// AssetToApplicationAsset converts an asset to an application asset.
// This is the inverse of the ApplicationAssetToAsset() function.
func AssetToApplicationAsset(asset *assetpb.Asset) (*applicationpb.Application_Asset, error) {
	if asset == nil {
		return nil, fmt.Errorf("asset is nil")
	}

	applicationAsset := &applicationpb.Application_Asset{}
	switch src := asset.GetSource().(type) {
	case *assetpb.Asset_Catalog:
		applicationAsset.Variant = &applicationpb.Application_Asset_Catalog{
			Catalog: src.Catalog.GetIdVersion(),
		}
	case *assetpb.Asset_Local:
		processed := src.Local
		if processed == nil {
			return nil, fmt.Errorf("local asset has no processed variant")
		}

		switch v := processed.GetVariant().(type) {
		case *processedassetpb.ProcessedAsset_Data:
			applicationAsset.Variant = &applicationpb.Application_Asset_Data{
				Data: v.Data,
			}

		case *processedassetpb.ProcessedAsset_HardwareDevice:
			applicationAsset.Variant = &applicationpb.Application_Asset_HardwareDevice{
				HardwareDevice: v.HardwareDevice,
			}
		case *processedassetpb.ProcessedAsset_Process:
			applicationAsset.Variant = &applicationpb.Application_Asset_Process{
				Process: v.Process,
			}
		case *processedassetpb.ProcessedAsset_SceneObject:
			applicationAsset.Variant = &applicationpb.Application_Asset_SceneObject{
				SceneObject: v.SceneObject,
			}
		case *processedassetpb.ProcessedAsset_Service:
			applicationAsset.Variant = &applicationpb.Application_Asset_Service{
				Service: v.Service,
			}
		case *processedassetpb.ProcessedAsset_Skill:
			applicationAsset.Variant = &applicationpb.Application_Asset_Skill{
				Skill: v.Skill,
			}
		default:
			return nil, fmt.Errorf("unknown processed asset variant")
		}
	default:
		return nil, fmt.Errorf("unknown asset source")
	}

	return applicationAsset, nil
}
