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

// Package main is a generator for runtime InstalledAsset.
package main

import (
	"flag"

	"intrinsic/assets/dependencies/build_defs/runtimeasset"
	"intrinsic/production/intrinsic"
	"intrinsic/util/proto/protoio"

	log "github.com/golang/glog"
)

var (
	outputInstalledAsset = flag.String("output_installed_asset", "", "Output path for the serialized InstalledAsset proto.")
	versionFile          = flag.String("version_file", "", "Path to a file containing the version string (e.g. bazel-out/stable-status.txt).")
)

func main() {
	intrinsic.Init()

	version, err := runtimeasset.GetVersion(*versionFile)
	if err != nil {
		log.Exitf("failed to get asset version: %v", err)
	}

	if *outputInstalledAsset == "" {
		log.Exit("output_installed_asset must be provided")
	}

	asset, err := runtimeasset.GenerateInstalledAsset(version)
	if err != nil {
		log.Exitf("failed to generate runtime installed asset: %v", err)
	}

	if err := protoio.WriteStableTextProto(*outputInstalledAsset, asset); err != nil {
		log.Exitf("failed to write runtime installed asset: %v", err)
	}
}
