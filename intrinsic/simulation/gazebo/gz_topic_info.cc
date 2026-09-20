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

#include "intrinsic/simulation/gazebo/gz_topic_info.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/simulation/gazebo/proto/v1/gazebo_service.pb.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace simulation {

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo::ObjectInfo> ToProto(
    const GzTopicInfo::ObjectInfo& object_info) {
  if (object_info.name.empty()) {
    return intrinsic::InvalidArgumentErrorBuilder()
           << "Object name must not be empty.";
  }

  intrinsic_proto::simulation::v1::TopicInfo::ObjectInfo proto;
  proto.set_name(object_info.name);
  return proto;
}

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo::SensorInfo> ToProto(
    const GzTopicInfo::SensorInfo& sensor_info) {
  if (sensor_info.name.empty()) {
    return intrinsic::InvalidArgumentErrorBuilder()
           << "Sensor name must not be empty.";
  }
  if (sensor_info.update_rate < 0) {
    return intrinsic::InvalidArgumentErrorBuilder()
           << "Sensor update_rate must be non-negative, got "
           << sensor_info.update_rate;
  }

  intrinsic_proto::simulation::v1::TopicInfo::SensorInfo proto;
  proto.set_name(sensor_info.name);
  if (!sensor_info.type.empty()) {
    proto.set_type(sensor_info.type);
  }
  proto.set_is_trigger_topic(sensor_info.is_trigger_topic);
  proto.set_update_rate(sensor_info.update_rate);
  return proto;
}

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo::PluginInfo> ToProto(
    const GzTopicInfo::PluginInfo& plugin_info) {
  if (plugin_info.name.empty()) {
    return intrinsic::InvalidArgumentErrorBuilder()
           << "Plugin name must not be empty.";
  }

  intrinsic_proto::simulation::v1::TopicInfo::PluginInfo proto;
  proto.set_name(plugin_info.name);
  if (!plugin_info.filename.empty()) {
    proto.set_filename(plugin_info.filename);
  }
  return proto;
}

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo::JointInfo> ToProto(
    const GzTopicInfo::JointInfo& joint_info) {
  if (joint_info.name.empty()) {
    return intrinsic::InvalidArgumentErrorBuilder()
           << "Joint name must not be empty.";
  }

  intrinsic_proto::simulation::v1::TopicInfo::JointInfo proto;
  proto.set_name(joint_info.name);
  if (joint_info.axis_index.has_value()) {
    proto.set_axis_index(*joint_info.axis_index);
  }
  return proto;
}

absl::StatusOr<intrinsic_proto::simulation::v1::TopicInfo> ToProto(
    const GzTopicInfo& topic_info) {
  if (topic_info.topic_name.empty()) {
    return intrinsic::InvalidArgumentErrorBuilder()
           << "Topic name must not be empty.";
  }

  if (topic_info.sensor_info.has_value() &&
      topic_info.sensor_info->is_trigger_topic &&
      topic_info.is_sim_server_advertised && !topic_info.is_service_topic) {
    return intrinsic::InvalidArgumentErrorBuilder()
           << "Trigger topic cannot be advertised by the simulation server, "
              "unless it is a service. Topic: "
           << topic_info.topic_name;
  }

  intrinsic_proto::simulation::v1::TopicInfo proto;
  proto.set_topic_name(topic_info.topic_name);
  proto.set_is_sim_server_advertised(topic_info.is_sim_server_advertised);
  proto.set_is_service_topic(topic_info.is_service_topic);

  if (topic_info.object_info.has_value()) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_object_info(),
                          ToProto(*topic_info.object_info),
                          _ << "Topic: " << topic_info.topic_name);
  }

  if (topic_info.sensor_info.has_value()) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_sensor_info(),
                          ToProto(*topic_info.sensor_info),
                          _ << "Topic: " << topic_info.topic_name);
  }

  if (topic_info.plugin_info.has_value()) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_plugin_info(),
                          ToProto(*topic_info.plugin_info),
                          _ << "Topic: " << topic_info.topic_name);
  }

  if (topic_info.joint_info.has_value()) {
    INTR_ASSIGN_OR_RETURN(*proto.mutable_joint_info(),
                          ToProto(*topic_info.joint_info),
                          _ << "Topic: " << topic_info.topic_name);
  }

  if (!topic_info.metadata.empty()) {
    proto.set_metadata(topic_info.metadata);
  }

  return proto;
}

absl::StatusOr<intrinsic_proto::simulation::v1::Topics> ToProto(
    absl::Span<const GzTopicInfo> topic_infos) {
  if (topic_infos.empty()) {
    return intrinsic::InvalidArgumentErrorBuilder() << "Topic list is empty.";
  }
  intrinsic_proto::simulation::v1::Topics proto_topics;
  for (const auto& info : topic_infos) {
    INTR_ASSIGN_OR_RETURN(*proto_topics.add_topics(), ToProto(info));
  }
  return proto_topics;
}

}  // namespace simulation
}  // namespace intrinsic
