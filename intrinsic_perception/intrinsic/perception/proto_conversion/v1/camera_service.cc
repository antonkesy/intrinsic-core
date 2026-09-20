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

#include "intrinsic/perception/proto_conversion/v1/camera_service.h"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/perception/cameras/sensor_information.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/pixel_type.h"
#include "intrinsic/perception/proto/v1/camera_service.pb.h"
#include "intrinsic/perception/proto/v1/image_buffer.pb.h"
#include "intrinsic/perception/proto_conversion/v1/camera_params.h"
#include "intrinsic/perception/proto_conversion/v1/dimensions.h"
#include "intrinsic/perception/proto_conversion/v1/image_buffer.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic_proto::perception::v1 {

absl::StatusOr<intrinsic::perception::SensorInformation> FromProto(
    const SensorInformation& sensor_info) {
  const int64_t id = sensor_info.id();
  std::optional<intrinsic::perception::CameraParams> factory_camera_params;
  if (sensor_info.has_factory_camera_params()) {
    factory_camera_params = FromProto(sensor_info.factory_camera_params());
  }
  std::optional<intrinsic::Pose> camera_t_sensor;
  if (sensor_info.has_camera_t_sensor()) {
    INTR_ASSIGN_OR_RETURN(camera_t_sensor, intrinsic_proto::FromProto(
                                               sensor_info.camera_t_sensor()));
  }
  std::vector<intrinsic::perception::PixelType> supported_pixel_types;
  if (!sensor_info.supported_pixel_types().empty()) {
    std::transform(
        sensor_info.supported_pixel_types().cbegin(),
        sensor_info.supported_pixel_types().cend(),
        std::back_inserter(supported_pixel_types),
        [](const int pixel_type) { return FromProto(PixelType(pixel_type)); });
  }
  const intrinsic::perception::Dimensions dimensions =
      FromProto(sensor_info.dimensions());
  return intrinsic::perception::SensorInformation(
      id, sensor_info.display_name(), factory_camera_params, camera_t_sensor,
      supported_pixel_types, dimensions, sensor_info.disabled());
}

SensorInformation ToProto(
    const intrinsic::perception::SensorInformation& sensor_info) {
  SensorInformation sensor_info_proto;
  sensor_info_proto.set_id(sensor_info.id());
  sensor_info_proto.set_display_name(sensor_info.display_name());
  if (sensor_info.factory_camera_params().has_value()) {
    *sensor_info_proto.mutable_factory_camera_params() =
        ToProto(sensor_info.factory_camera_params().value());
  }
  if (sensor_info.camera_t_sensor().has_value()) {
    *sensor_info_proto.mutable_camera_t_sensor() =
        intrinsic::ToProto(sensor_info.camera_t_sensor().value());
  }
  for (const intrinsic::perception::PixelType& pixel_type :
       sensor_info.supported_pixel_types()) {
    sensor_info_proto.add_supported_pixel_types(ToProto(pixel_type));
  }
  *sensor_info_proto.mutable_dimensions() = ToProto(sensor_info.dimensions());
  sensor_info_proto.set_disabled(sensor_info.disabled());
  return sensor_info_proto;
}

}  // namespace intrinsic_proto::perception::v1
