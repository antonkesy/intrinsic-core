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

#ifndef INTRINSIC_SIMULATION_GAZEBO_GZ_TOPIC_INFO_H_
#define INTRINSIC_SIMULATION_GAZEBO_GZ_TOPIC_INFO_H_

#include <cstdint>
#include <optional>
#include <string>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/simulation/gazebo/proto/v1/gazebo_service.pb.h"

namespace intrinsic {
namespace simulation {

// Metadata for a Gz Transport topic in Gazebo simulation.
// See related `TopicInfo` message definition in
// intrinsic/simulation/gazebo/proto/v1/gazebo_service.proto for more details.
struct GzTopicInfo {
  // Metadata about the object associated with a topic.
  struct ObjectInfo {
    std::string name;
  };

  // Metadata about a sensor associated with a topic.
  struct SensorInfo {
    std::string name;
    std::string type;
    bool is_trigger_topic = false;
    int64_t update_rate = 0;
  };

  // Metadata about a Gazebo System plugin associated with a topic.
  struct PluginInfo {
    std::string name;
    std::string filename;
  };

  // Metadata about a joint associated with a topic.
  struct JointInfo {
    std::string name;
    // Will be either 0 or 1 if set. If unset, the topic is not associated with
    // a particular joint axis.
    std::optional<uint32_t> axis_index;
  };

  // Fully qualified topic name.
  std::string topic_name;
  // Object info, specified if the topic is associated with an object.
  std::optional<ObjectInfo> object_info;
  // Sensor info, only specified if the topic is associated with a sensor.
  std::optional<SensorInfo> sensor_info;
  // Plugin info, only specified if the topic is associated with a plugin.
  std::optional<PluginInfo> plugin_info;
  // Joint info, only specified if the topic is associated with a joint.
  std::optional<JointInfo> joint_info;
  // Whether the topic is advertised by the sim server. If false, the topic is
  // subscribed by the sim server.
  bool is_sim_server_advertised = false;
  // Whether the topic is associated with a Gz Transport Service. If false, it
  // is a Gz Transport pubsub topic.
  bool is_service_topic = false;
  // Additional server-implementation specific metadata about this topic.
  std::string metadata;
};

// ---------------------- Proto conversion methods ------------------------

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo::ObjectInfo> ToProto(
    const GzTopicInfo::ObjectInfo& object_info);

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo::SensorInfo> ToProto(
    const GzTopicInfo::SensorInfo& sensor_info);

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo::PluginInfo> ToProto(
    const GzTopicInfo::PluginInfo& plugin_info);

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo::JointInfo> ToProto(
    const GzTopicInfo::JointInfo& joint_info);

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo> ToProto(
    const GzTopicInfo& topic_info);

absl::StatusOr<intrinsic_proto::simulation::v1::Topics> ToProto(
    absl::Span<const GzTopicInfo> topic_infos);

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_GZ_TOPIC_INFO_H_
