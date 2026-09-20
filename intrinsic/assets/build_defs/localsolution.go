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

// Package localsolution constructs a LocalSolution proto from a set of assets
// and asset instances. It does some light validation.
package localsolution

import (
	"fmt"
	"maps"
	"slices"

	"intrinsic/assets/idutils"
	"intrinsic/util/proto/registryutil"

	"google.golang.org/protobuf/encoding/prototext"
	"google.golang.org/protobuf/reflect/protoregistry"

	assetpb "intrinsic/assets/build_defs/asset_go_proto"
	opmodepb "intrinsic/config/proto/operation_mode_go_proto"
	owupb "intrinsic/world/public/proto/object_world_updates_go_proto"

	anypb "google.golang.org/protobuf/types/known/anypb"
)

// Options contains options for creating a LocalSolution proto.
type Options struct {
	AssetInfos           []*assetpb.AssetInfo
	AssetLocalInfos      []*assetpb.AssetLocalInfo
	AssetCatalogRefInfos []*assetpb.AssetCatalogRefInfo
	AssetInstanceInfos   []*assetpb.AssetInstanceInfo
	ObjectWorldUpdates   []*owupb.ObjectWorldUpdate
	DefaultOperationMode opmodepb.OperationMode
	DisplayName          string
	BuildTarget          string
	Version              string
}

// New creates a new LocalSolution proto.
func New(opts Options) (*assetpb.LocalSolution, error) {
	types := map[string]*protoregistry.Types{}
	for _, ai := range opts.AssetInfos {
		id, err := idutils.IDFromProto(ai.GetId())
		if err != nil {
			return nil, fmt.Errorf("AssetInfo contains an invalid asset id: %v", err)
		}
		if ai.GetFileDescriptorSet() == nil {
			continue
		}
		t, err := registryutil.NewTypesFromFileDescriptorSet(ai.GetFileDescriptorSet())
		if err != nil {
			return nil, fmt.Errorf("cannot parse file descriptor set protos for %q: %v", id, err)
		}
		types[id] = t
	}

	assets := map[string]*assetpb.LocalSolution_Asset{}
	for _, la := range opts.AssetLocalInfos {
		asset, err := idutils.IDFromProto(la.GetId())
		if err != nil {
			return nil, fmt.Errorf("AssetLocalInfo contains an invalid asset id: %v", err)
		}
		if _, exists := assets[asset]; exists {
			return nil, fmt.Errorf("solution contains multiple %q assets", asset)
		}
		assets[asset] = &assetpb.LocalSolution_Asset{
			Variant: &assetpb.LocalSolution_Asset_Local{
				Local: la,
			},
		}
	}
	for _, ca := range opts.AssetCatalogRefInfos {
		asset, err := idutils.IDFromProto(ca.GetIdVersion().GetId())
		if err != nil {
			return nil, fmt.Errorf("AssetCatalogRefInfo contains an invalid asset id: %v", err)
		}
		if _, exists := assets[asset]; exists {
			return nil, fmt.Errorf("solution contains multiple %q assets", asset)
		}
		assets[asset] = &assetpb.LocalSolution_Asset{
			Variant: &assetpb.LocalSolution_Asset_Catalog{
				Catalog: ca,
			},
		}
	}

	// instanceNames is a set of asset instance names.
	instanceNames := map[string]struct{}{}
	var instances []*assetpb.AssetInstanceInfo
	for _, ai := range opts.AssetInstanceInfos {
		asset := ai.GetAsset()
		if _, exists := assets[asset]; !exists {
			return nil, fmt.Errorf("solution contains instance with no corresponding asset type %q", asset)
		}
		if _, exists := instanceNames[ai.GetInstanceName()]; exists {
			return nil, fmt.Errorf("solution contains multiple asset instances named %q", ai.GetInstanceName())
		}

		// If the instance has a config then we parse it with the available file
		// descriptors and put it in the instance. For now we produce an error if
		// there are no descriptors available as no downstream components will be
		// able to parse it. That might change in the future.
		if ai.GetTextProto() != "" {
			t, exists := types[asset]
			if !exists {
				return nil, fmt.Errorf("cannot find file descriptor set for instance %q", asset)
			}
			config := &anypb.Any{}
			options := new(prototext.UnmarshalOptions)
			options.Resolver = t
			if err := options.Unmarshal([]byte(ai.GetTextProto()), config); err != nil {
				return nil, fmt.Errorf("failed to parse asset configuration: %v", err)
			}
			ai.Config = &assetpb.AssetInstanceInfo_Parsed{
				Parsed: config,
			}
		}

		instanceNames[ai.GetInstanceName()] = struct{}{}
		instances = append(instances, ai)
	}

	return &assetpb.LocalSolution{
		Assets:               slices.Collect(maps.Values(assets)),
		Instances:            instances,
		ObjectWorldUpdates:   opts.ObjectWorldUpdates,
		DefaultOperationMode: opts.DefaultOperationMode,
		DisplayName:          opts.DisplayName,
		BuildTarget:          opts.BuildTarget,
		Version:              opts.Version,
	}, nil
}
