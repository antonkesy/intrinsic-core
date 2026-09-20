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

#include "intrinsic/simulation/gazebo/plugins/cameras/simulation_camera_settings.h"

#include "intrinsic/perception/cameras/genicam/feature_names.h"
#include "intrinsic/perception/proto/v1/camera_settings.pb.h"
#include "intrinsic/perception/proto/v1/intrinsic_params.pb.h"
#include "intrinsic/perception/proto/v1/settings.pb.h"

namespace intrinsic::perception {

intrinsic_proto::perception::v1::CameraSetting GetWidthSetting(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params) {
  intrinsic_proto::perception::v1::CameraSetting setting;
  setting.set_name(genicam::kWidth);
  setting.set_integer_value(intrinsic_params.dimensions().cols());
  return setting;
}

intrinsic_proto::perception::v1::CameraSettingProperties
GetWidthSettingProperties(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params) {
  intrinsic_proto::perception::v1::CameraSettingProperties setting_properties;
  setting_properties.set_name(genicam::kWidth);
  auto& integer_properties = *setting_properties.mutable_integer_properties();
  integer_properties.mutable_range()->set_minimum(
      intrinsic_params.dimensions().cols());
  integer_properties.mutable_range()->set_maximum(
      intrinsic_params.dimensions().cols());
  integer_properties.set_increment(1);
  return setting_properties;
}

intrinsic_proto::perception::v1::CameraSetting GetHeightSetting(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params) {
  intrinsic_proto::perception::v1::CameraSetting setting;
  setting.set_name(genicam::kHeight);
  setting.set_integer_value(intrinsic_params.dimensions().rows());
  return setting;
}

intrinsic_proto::perception::v1::CameraSettingProperties
GetHeightSettingProperties(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params) {
  intrinsic_proto::perception::v1::CameraSettingProperties setting_properties;
  setting_properties.set_name(genicam::kHeight);
  auto& integer_properties = *setting_properties.mutable_integer_properties();
  integer_properties.mutable_range()->set_minimum(
      intrinsic_params.dimensions().rows());
  integer_properties.mutable_range()->set_maximum(
      intrinsic_params.dimensions().rows());
  integer_properties.set_increment(1);
  return setting_properties;
}

intrinsic_proto::perception::v1::CameraSetting GetSensorWidthSetting(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params) {
  auto setting = GetWidthSetting(intrinsic_params);
  setting.set_name(genicam::kSensorWidth);
  return setting;
}

intrinsic_proto::perception::v1::CameraSettingProperties
GetSensorWidthSettingProperties(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params) {
  auto setting_props = GetWidthSettingProperties(intrinsic_params);
  setting_props.set_name(genicam::kSensorWidth);
  return setting_props;
}

intrinsic_proto::perception::v1::CameraSetting GetSensorHeightSetting(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params) {
  auto setting = GetHeightSetting(intrinsic_params);
  setting.set_name(genicam::kSensorHeight);
  return setting;
}

intrinsic_proto::perception::v1::CameraSettingProperties
GetSensorHeightSettingProperties(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params) {
  auto setting_props = GetHeightSettingProperties(intrinsic_params);
  setting_props.set_name(genicam::kSensorHeight);
  return setting_props;
}

}  // namespace intrinsic::perception
