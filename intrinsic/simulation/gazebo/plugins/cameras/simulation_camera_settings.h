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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_SIMULATION_CAMERA_SETTINGS_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_SIMULATION_CAMERA_SETTINGS_H_

#include "intrinsic/perception/proto/v1/camera_settings.pb.h"
#include "intrinsic/perception/proto/v1/intrinsic_params.pb.h"

namespace intrinsic::perception {

// Returns settings for GenICam feature "Width".
intrinsic_proto::perception::v1::CameraSetting GetWidthSetting(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params);

intrinsic_proto::perception::v1::CameraSettingProperties
GetWidthSettingProperties(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params);

// Returns settings for GenICam feature "Height".
intrinsic_proto::perception::v1::CameraSetting GetHeightSetting(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params);

intrinsic_proto::perception::v1::CameraSettingProperties
GetHeightSettingProperties(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params);

// Returns settings for GenICam feature "SensorWidth".
intrinsic_proto::perception::v1::CameraSetting GetSensorWidthSetting(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params);

intrinsic_proto::perception::v1::CameraSettingProperties
GetSensorWidthSettingProperties(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params);

// Returns settings for GenICam feature "SensorHeight".
intrinsic_proto::perception::v1::CameraSetting GetSensorHeightSetting(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params);

intrinsic_proto::perception::v1::CameraSettingProperties
GetSensorHeightSettingProperties(
    const intrinsic_proto::perception::v1::IntrinsicParams& intrinsic_params);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_SIMULATION_CAMERA_SETTINGS_H_
