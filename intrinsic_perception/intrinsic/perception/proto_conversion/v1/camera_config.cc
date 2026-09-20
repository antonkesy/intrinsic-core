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

#include "intrinsic/perception/proto_conversion/v1/camera_config.h"

#include <cstdint>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/perception/cameras/camera_config.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/cameras/camera_setting.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/proto/v1/camera_config.pb.h"
#include "intrinsic/perception/proto/v1/sensor_config.pb.h"
#include "intrinsic/perception/proto_conversion/v1/camera_identifier.h"
#include "intrinsic/perception/proto_conversion/v1/camera_params.h"
#include "intrinsic/perception/proto_conversion/v1/camera_settings.h"
#include "intrinsic/util/proto/repeated_field_util.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic_proto::perception::v1 {

absl::StatusOr<std::pair<intrinsic::perception::CameraConfig,
                         intrinsic::perception::CameraIdentifier>>
FromProto(const CameraConfig& camera_config) {
  std::vector<intrinsic::perception::CameraSetting> camera_settings;
  camera_settings.reserve(camera_config.camera_settings_size());
  for (const auto& setting : camera_config.camera_settings()) {
    camera_settings.push_back(FromProto(setting));
  }
  intrinsic::perception::CameraParamsBySensorId camera_params_by_sensor_id;
  for (const auto& sensor_config : camera_config.sensor_configs()) {
    if (!sensor_config.has_camera_params()) {
      continue;
    }
    camera_params_by_sensor_id.emplace(
        sensor_config.id(), FromProto(sensor_config.camera_params()));
  }
  absl::flat_hash_map<int64_t, intrinsic::Pose3d> camera_t_sensor_by_sensor_id;
  for (const auto& sensor_config : camera_config.sensor_configs()) {
    if (!sensor_config.has_camera_t_sensor()) {
      continue;
    }
    INTR_ASSIGN_OR_RETURN(intrinsic::Pose3d camera_t_sensor,
                          FromProtoNormalized(sensor_config.camera_t_sensor()));
    camera_t_sensor_by_sensor_id.emplace(sensor_config.id(),
                                         std::move(camera_t_sensor));
  }
  return std::make_pair(
      intrinsic::perception::CameraConfig{
          .camera_settings = std::move(camera_settings),
          .camera_params_by_sensor_id = std::move(camera_params_by_sensor_id),
          .camera_t_sensor_by_sensor_id =
              std::move(camera_t_sensor_by_sensor_id),
      },
      FromProto(camera_config.identifier()));
}

CameraConfig ToProto(
    const intrinsic::perception::CameraConfig& camera_config,
    const intrinsic::perception::CameraIdentifier& camera_identifier) {
  CameraConfig proto;
  *proto.mutable_identifier() = ToProto(camera_identifier);
  for (const auto& camera_setting : camera_config.camera_settings) {
    *proto.add_camera_settings() = ToProto(camera_setting);
  }

  const auto get_sensor_config = [&](int64_t id) -> SensorConfig& {
    if (auto it = absl::c_find_if(*proto.mutable_sensor_configs(),
                                  [&](const SensorConfig& sensor_config) {
                                    return sensor_config.id() == id;
                                  });
        it != proto.sensor_configs().end()) {
      return *it;
    }
    SensorConfig* sensor_config = proto.add_sensor_configs();
    sensor_config->set_id(id);
    return *sensor_config;
  };

  for (const auto& [id, camera_params] :
       camera_config.camera_params_by_sensor_id) {
    *get_sensor_config(id).mutable_camera_params() = ToProto(camera_params);
  }
  for (const auto& [id, camera_t_sensor] :
       camera_config.camera_t_sensor_by_sensor_id) {
    *get_sensor_config(id).mutable_camera_t_sensor() = ToProto(camera_t_sensor);
  }
  intrinsic::Sort(proto.mutable_sensor_configs(),
                  [](const SensorConfig* a, const SensorConfig* b) {
                    return a->id() < b->id();
                  });
  return proto;
}

}  // namespace intrinsic_proto::perception::v1
