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

// Package resourcevalidate provides utils for validating resource representations.
package resourcevalidate

import (
	"context"
	"fmt"

	"intrinsic/assets/assetvalidate"
	"intrinsic/assets/conversion/runtime"
	"intrinsic/assets/data/datavalidate"
	"intrinsic/assets/errors/report"
	"intrinsic/assets/hardware_devices/hardwaredevicevalidate"
	"intrinsic/assets/metadatautils"
	"intrinsic/assets/processes/processvalidate"
	"intrinsic/assets/scene_objects/sceneobjectvalidate"
	"intrinsic/assets/services/servicevalidate"
	"intrinsic/skills/skillvalidate"

	assetpb "intrinsic/assets/proto/v1/asset_go_proto"
	rtrpb "intrinsic/resources/proto/resource_type_runtime_go_proto"
)

type resourceTypeRuntimeOptions struct {
	dataOptions           []datavalidate.DataAssetOption
	hardwareDeviceOptions []hardwaredevicevalidate.ProcessedHardwareDeviceManifestOption
	processOptions        []processvalidate.ProcessAssetOption
	report                *report.Report
	sceneObjectOptions    []sceneobjectvalidate.ProcessedSceneObjectManifestOption
	serviceOptions        []servicevalidate.ProcessedServiceManifestOption
	skillOptions          []skillvalidate.ProcessedSkillManifestOption
}

// ResourceTypeOption is an option for validating a ResourceTypeRuntime.
type ResourceTypeOption func(*resourceTypeRuntimeOptions)

// WithDataOptions appends options to use for validating Data Assets.
func WithDataOptions(options ...datavalidate.DataAssetOption) ResourceTypeOption {
	return func(opts *resourceTypeRuntimeOptions) {
		opts.dataOptions = append(opts.dataOptions, options...)
	}
}

// WithHardwareDeviceOptions appends options to use for validating HardwareDevices.
func WithHardwareDeviceOptions(options ...hardwaredevicevalidate.ProcessedHardwareDeviceManifestOption) ResourceTypeOption {
	return func(opts *resourceTypeRuntimeOptions) {
		opts.hardwareDeviceOptions = append(opts.hardwareDeviceOptions, options...)
	}
}

// WithProcessOptions appends options to use for validating Processes.
func WithProcessOptions(options ...processvalidate.ProcessAssetOption) ResourceTypeOption {
	return func(opts *resourceTypeRuntimeOptions) {
		opts.processOptions = append(opts.processOptions, options...)
	}
}

// WithSceneObjectOptions appends options to use for validating SceneObjects.
func WithSceneObjectOptions(options ...sceneobjectvalidate.ProcessedSceneObjectManifestOption) ResourceTypeOption {
	return func(opts *resourceTypeRuntimeOptions) {
		opts.sceneObjectOptions = append(opts.sceneObjectOptions, options...)
	}
}

// WithServiceOptions appends options to use for validating Services.
func WithServiceOptions(options ...servicevalidate.ProcessedServiceManifestOption) ResourceTypeOption {
	return func(opts *resourceTypeRuntimeOptions) {
		opts.serviceOptions = append(opts.serviceOptions, options...)
	}
}

func WithSkillOptions(options ...skillvalidate.ProcessedSkillManifestOption) ResourceTypeOption {
	return func(opts *resourceTypeRuntimeOptions) {
		opts.skillOptions = append(opts.skillOptions, options...)
	}
}

// WithReport sets the shared validation Report to use for collecting warnings.
func WithReport(report *report.Report) ResourceTypeOption {
	return func(opts *resourceTypeRuntimeOptions) {
		opts.report = report
		WithDataOptions(datavalidate.WithReport(report))(opts)
		WithHardwareDeviceOptions(hardwaredevicevalidate.WithReport(report))(opts)
		WithProcessOptions(processvalidate.WithReport(report))(opts)
		WithSceneObjectOptions(sceneobjectvalidate.WithReport(report))(opts)
		WithServiceOptions(servicevalidate.WithReport(report))(opts)
		WithSkillOptions(skillvalidate.WithReport(report))(opts)
	}
}

// ResourceTypeRuntime validates a ResourceTypeRuntime.
func ResourceTypeRuntime(ctx context.Context, rtr *rtrpb.ResourceTypeRuntime, options ...ResourceTypeOption) error {
	opts := &resourceTypeRuntimeOptions{}
	WithReport(report.New())(opts)
	for _, opt := range options {
		opt(opts)
	}

	if rtr == nil {
		return fmt.Errorf("ResourceTypeRuntime must not be nil")
	}

	if err := metadatautils.ValidateMetadata(
		rtr.GetMetadata(),
		metadatautils.WithRuntimeOptions(),
	); err != nil {
		return err
	}

	processed, err := runtime.RuntimeToProcessedAsset(rtr)
	if err != nil {
		return err
	}
	asset := &assetpb.Asset{
		Source: &assetpb.Asset_Local{
			Local: processed,
		},
	}

	return assetvalidate.Asset(ctx, asset,
		assetvalidate.WithReport(opts.report),
		assetvalidate.WithDataOptions(opts.dataOptions...),
		assetvalidate.WithHardwareDeviceOptions(opts.hardwareDeviceOptions...),
		assetvalidate.WithProcessOptions(opts.processOptions...),
		assetvalidate.WithSceneObjectOptions(opts.sceneObjectOptions...),
		assetvalidate.WithServiceOptions(opts.serviceOptions...),
		assetvalidate.WithSkillOptions(opts.skillOptions...),
	)
}
