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

// Package assetprocessor contains a function to process the assets in an
// application. inctl commands can use it populate the assets and instances
// fields before app deployment or release.
package assetprocessor

import (
	"context"
	"fmt"
	"strings"

	"intrinsic/assets/bundle"
	"intrinsic/assets/idutils"
	"intrinsic/util/proto/registryutil"

	log "github.com/golang/glog"
	"go.opencensus.io/trace" 
	"golang.org/x/sync/errgroup"
	"google.golang.org/protobuf/encoding/prototext"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/reflect/protoregistry"

	assetpb "intrinsic/assets/build_defs/asset_go_proto"
	ipb "intrinsic/assets/proto/id_go_proto"
	icpb "intrinsic/assets/proto/v1/instance_config_go_proto"
	apppb "intrinsic/config/proto/application_go_proto"
	commonpb "intrinsic/config/proto/common_go_proto"
	processpb "intrinsic/config/proto/process_go_proto"
	rspb "intrinsic/config/proto/resource_set_go_proto"

	anypb "google.golang.org/protobuf/types/known/anypb"
)

type processedAsset struct {
	id    string
	asset *apppb.Application_Asset
	types *protoregistry.Types
}

func processAsset(ctx context.Context, a *assetpb.LocalSolution_Asset, proc *Processor) (*processedAsset, error) {

	ctx, span := trace.StartSpan(ctx, "assetprocessor.processAsset")
	defer span.End()

	switch a.GetVariant().(type) {
	case *assetpb.LocalSolution_Asset_Local:
		localInfo := a.GetLocal()

		span.AddAttributes(trace.StringAttribute("asset_id", idutils.IDFromProtoUnchecked(localInfo.GetId())))
		span.AddAttributes(trace.StringAttribute("asset_type", localInfo.GetAssetType().String()))


		id, err := idutils.IDFromProto(localInfo.GetId())
		if err != nil {
			return nil, fmt.Errorf("failed to parse local Asset ID: %w", err)
		}
		p, err := proc.PathResolver(localInfo.GetBundleRunfilesPath())
		if err != nil {
			return nil, fmt.Errorf("failed to resolve path prefix for Asset %s: %w", id, err)
		}

		processedBundle, err := proc.Processor.ProcessFile(ctx, p)
		if err != nil {
			return nil, fmt.Errorf("failed to process Asset %s: %w", id, err)
		}
		types, err := registryutil.NewTypesFromFileDescriptorSet(processedBundle.FileDescriptorSet())
		if err != nil {
			return nil, fmt.Errorf("cannot parse file descriptor set protos: %w", err)
		}
		return &processedAsset{
			id:    id,
			asset: processedBundle.DeployApp(),
			types: types,
		}, nil

	case *assetpb.LocalSolution_Asset_Catalog:
		catalogInfo := a.GetCatalog()

		span.AddAttributes(trace.StringAttribute("asset_id", idutils.IDFromProtoUnchecked(catalogInfo.GetIdVersion().GetId())))
		span.AddAttributes(trace.StringAttribute("asset_type", catalogInfo.GetAssetType().String()))

		idVersion := catalogInfo.GetIdVersion()
		id, err := idutils.IDFromProto(idVersion.GetId())
		if err != nil {
			return nil, fmt.Errorf("failed to parse catalog Asset ID: %w", err)
		}
		return &processedAsset{
			id: id,
			asset: &apppb.Application_Asset{
				Variant: &apppb.Application_Asset_Catalog{
					Catalog: idVersion,
				},
			},
		}, nil
	default:
		return nil, fmt.Errorf("unsupported solution asset variant type %T", a.GetVariant())
	}
}

func convertAssets(ctx context.Context, solutionAssets []*assetpb.LocalSolution_Asset, proc *Processor) (map[string]*processedAsset, error) {

	ctx, span := trace.StartSpan(ctx, "assetprocessor.convertAssets")
	defer span.End()

	// NOTE: Pre-allocate all results to avoid data races in the goroutines below.
	results := make([]*processedAsset, len(solutionAssets))
	g, gCtx := errgroup.WithContext(ctx)
	g.SetLimit(15)
	for i, a := range solutionAssets {
		g.Go(func() error {
			pa, err := processAsset(gCtx, a, proc)
			if err != nil {
				return err
			}
			results[i] = pa
			return nil
		})
	}
	if err := g.Wait(); err != nil {
		return nil, err
	}

	assets := map[string]*processedAsset{}
	for _, r := range results {
		if _, exists := assets[r.id]; exists {
			return nil, fmt.Errorf("multiple assets with the same asset ID %q", r.id)
		}
		assets[r.id] = r
	}
	if len(assets) == 0 {
		assets = nil
	}

	return assets, nil
}

func convertInstances(instanceInfos []*assetpb.AssetInstanceInfo, assets map[string]*processedAsset) (map[string]*apppb.Application_Instance, error) {
	instances := map[string]*apppb.Application_Instance{}
	for _, i := range instanceInfos {
		asset := i.GetAsset()
		pa, exists := assets[asset]
		if !exists {
			return nil, fmt.Errorf("instance references unknown asset %q", asset)
		}

		var serviceConfig *anypb.Any
		switch s := i.GetConfig().(type) {
		case *assetpb.AssetInstanceInfo_TextProto:
			if pa.types == nil {
				return nil, fmt.Errorf("cannot find types for instance %q", i.GetInstanceName())
			}

			serviceConfig = &anypb.Any{}
			options := &prototext.UnmarshalOptions{Resolver: pa.types}
			if err := options.Unmarshal([]byte(s.TextProto), serviceConfig); err != nil {
				return nil, fmt.Errorf("failed to parse asset configuration: %v", err)
			}
		case *assetpb.AssetInstanceInfo_Parsed:
			serviceConfig = s.Parsed
		}

		var scheduledNodeHostname *string
		if i.GetRequiredNodeHostname() != "" {
			scheduledNodeHostname = proto.String(i.GetRequiredNodeHostname())
		}

		var config *icpb.InstanceConfig
		if serviceConfig != nil || scheduledNodeHostname != nil {
			if strings.Contains(string(serviceConfig.GetTypeUrl()), "SceneObjectConfig") {
				return nil, fmt.Errorf("instance %q has a SceneObjectConfig in config", i.GetInstanceName())
			}
			config = &icpb.InstanceConfig{
				Variant: &icpb.InstanceConfig_Service{
					Service: &icpb.InstanceConfig_ServiceInstanceConfig{
						ServiceConfig:         serviceConfig,
						ScheduledNodeHostname: scheduledNodeHostname,
					},
				},
			}
		}

		instances[i.GetInstanceName()] = &apppb.Application_Instance{
			Asset:  asset,
			Config: config,
		}
	}
	if len(instances) == 0 {
		instances = nil
	}
	return instances, nil
}

// SolutionAssetPreprocessor is a function that preprocesses a solution asset before it is processed
// by Process.
//
// If the preprocessor returns nil, the original asset is used.
type SolutionAssetPreprocessor func(*assetpb.LocalSolution_Asset) (*assetpb.LocalSolution_Asset, error)

// RewriteLocalToCatalogOptions contains the options for RewriteLocalToCatalog.
type RewriteLocalToCatalogOptions struct {
	// If non-nil, these default versions will be used instead of the provided version parameter if a match is found.
	Manifest map[string]string
}

// RewriteLocalToCatalogOption is an option for RewriteLocalToCatalog.
type RewriteLocalToCatalogOption func(*RewriteLocalToCatalogOptions)

// WithManifest returns an option that supplies a manifest of default catalog item versions.
func WithManifest(manifest map[string]string) RewriteLocalToCatalogOption {
	return func(opts *RewriteLocalToCatalogOptions) {
		opts.Manifest = manifest
	}
}

// RewriteLocalToCatalog returns a SolutionAssetPreprocessor that rewrites local assets to catalog
// assets with versions from the provided manifest.
func RewriteLocalToCatalog(options ...RewriteLocalToCatalogOption) SolutionAssetPreprocessor {
	opts := &RewriteLocalToCatalogOptions{}
	for _, opt := range options {
		opt(opts)
	}

	if opts.Manifest == nil {
		return nil
	}

	return func(a *assetpb.LocalSolution_Asset) (*assetpb.LocalSolution_Asset, error) {
		switch a.GetVariant().(type) {
		case *assetpb.LocalSolution_Asset_Local:
			key := idutils.IDFromProtoUnchecked(a.GetLocal().GetId())
			v, ok := opts.Manifest[key]
			if !ok {
				// We don't error out here, instead we allow data and process assets
				// to remain local. Downstream validators can handle missing catalog references.
				return nil, nil
			}
			log.Infof("Pinning local asset %q to manifest version %q", key, v)

			return &assetpb.LocalSolution_Asset{
				Variant: &assetpb.LocalSolution_Asset_Catalog{
					Catalog: &assetpb.AssetCatalogRefInfo{
						IdVersion: &ipb.IdVersion{
							Id:      a.GetLocal().GetId(),
							Version: v,
						},
					},
				},
			}, nil
		default:
			return nil, nil
		}
	}
}

// PathResolver is a function that resolves the path to an asset bundle.
type PathResolver func(path string) (string, error)

// Processor provides a way to process solution assets.
type Processor struct {
	SolutionAssetPreprocessor
	bundle.Processor
	PathResolver
}

// Process processes the solution assets and returns the corresponding
// application assets and instances.
func (proc *Processor) Process(ctx context.Context, sa *assetpb.LocalSolution) (*apppb.Application, error) {

	ctx, span := trace.StartSpan(ctx, "assetprocessor.Process")
	defer span.End()

	var preprocessedLocalSolution []*assetpb.LocalSolution_Asset
	if proc.SolutionAssetPreprocessor != nil {
		preprocessedLocalSolution = make([]*assetpb.LocalSolution_Asset, len(sa.GetAssets()))
		for i, a := range sa.GetAssets() {
			pa, err := proc.SolutionAssetPreprocessor(a)
			if err != nil {
				return nil, fmt.Errorf("failed to preprocess asset %q: %w", i, err)
			}
			if pa == nil {
				pa = a
			}
			preprocessedLocalSolution[i] = pa
		}
	} else {
		preprocessedLocalSolution = sa.GetAssets()
	}

	processedAssets, err := convertAssets(ctx, preprocessedLocalSolution, proc)
	if err != nil {
		return nil, err
	}
	instances, err := convertInstances(sa.GetInstances(), processedAssets)
	if err != nil {
		return nil, err
	}

	var assets map[string]*apppb.Application_Asset
	if len(processedAssets) > 0 {
		assets = make(map[string]*apppb.Application_Asset, len(processedAssets))
		for id, pa := range processedAssets {
			assets[id] = pa.asset
		}
	}

	return &apppb.Application{
		Metadata: &commonpb.Metadata{
			DisplayName: sa.GetDisplayName(),
			BuildTarget: sa.GetBuildTarget(),
			Category:    commonpb.Metadata_TEMPLATE,
		},
		// TODO: b/394317777 - Leave process unspecified if not provided.
		Process: &processpb.Process{},
		// TODO: b/394317777 - Leave resources unspecified.
		Resources:          &rspb.ResourceSet{},
		Assets:             assets,
		Instances:          instances,
		ObjectWorldUpdates: sa.GetObjectWorldUpdates(),
		OperationMode:      sa.GetDefaultOperationMode(),
	}, nil
}
