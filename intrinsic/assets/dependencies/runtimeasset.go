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

// Package runtimeasset provides access to the Intrinsic Runtime Asset protos.
package runtimeasset

import (
	_ "embed"
	"sync"

	log "github.com/golang/glog"
	"google.golang.org/protobuf/encoding/prototext"

	iapb "intrinsic/assets/proto/installed_assets_go_proto"
	aipb "intrinsic/assets/proto/v1/asset_instances_go_proto"
	resourcepb "intrinsic/resources/proto/resource_registry_go_proto"
)

//go:embed runtime_installed_asset.pbtxt
var runtimeInstalledAssetPbtxt []byte

//go:embed runtime_asset_instance.pbtxt
var runtimeAssetInstancePbtxt []byte

//go:embed runtime_resource_instance.pbtxt
var runtimeResourceInstancePbtxt []byte

var (
	runtimeInstalledAssetOnce sync.Once
	runtimeInstalledAsset     *iapb.InstalledAsset

	runtimeAssetInstanceOnce sync.Once
	runtimeAssetInstance     *aipb.AssetInstance

	runtimeResourceInstanceOnce sync.Once
	runtimeResourceInstance     *resourcepb.ResourceInstance
)

// InstalledAsset returns the runtime asset as an InstalledAsset.
func InstalledAsset() *iapb.InstalledAsset {
	runtimeInstalledAssetOnce.Do(func() {
		ia := &iapb.InstalledAsset{}
		if err := prototext.Unmarshal(runtimeInstalledAssetPbtxt, ia); err != nil {
			log.Fatalf("failed to unmarshal runtime installed asset pbtxt: %v", err)
		}
		runtimeInstalledAsset = ia
	})
	return runtimeInstalledAsset
}

// AssetInstance returns the runtime asset as an AssetInstance.
func AssetInstance() *aipb.AssetInstance {
	runtimeAssetInstanceOnce.Do(func() {
		ai := &aipb.AssetInstance{}
		if err := prototext.Unmarshal(runtimeAssetInstancePbtxt, ai); err != nil {
			log.Fatalf("failed to unmarshal runtime asset instance pbtxt: %v", err)
		}
		runtimeAssetInstance = ai
	})
	return runtimeAssetInstance
}

// ResourceInstance returns the platform as a ResourceInstance.
func ResourceInstance() *resourcepb.ResourceInstance {
	runtimeResourceInstanceOnce.Do(func() {
		ri := &resourcepb.ResourceInstance{}
		if err := prototext.Unmarshal(runtimeResourceInstancePbtxt, ri); err != nil {
			log.Fatalf("failed to unmarshal runtime resource instance pbtxt: %v", err)
		}
		runtimeResourceInstance = ri
	})
	return runtimeResourceInstance
}
