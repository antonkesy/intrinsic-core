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

#ifndef INTRINSIC_SIMULATION_WORLD_GRIPPER_PLUGIN_SPEC_H_
#define INTRINSIC_SIMULATION_WORLD_GRIPPER_PLUGIN_SPEC_H_

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.pb.h"

namespace intrinsic {
namespace simulation {

// Specification for generating FixedJointGripperPlugin SDF XML.
// Typically used for suction grippers where grasping creates a fixed joint
// between the gripper link and the grasped object.
struct FixedJointGripperPluginSpec {
  static constexpr char kPluginName[] = "FixedJointGripperPlugin";

  // Object-local link name for the gripper attachment link in the World.
  std::string gripper_link_name;

  // Gz Transport pubsub topic on which the gripper plugin will listen for
  // commands.
  std::string gripper_command_topic;

  // Gz Transport pubsub topic on which the gripper plugin will publish gripper
  // status updates.
  std::string gripper_status_topic;

  // Configuration for simulating an EOAT suction gripper's GPIO signals.
  std::optional<intrinsic_proto::eoat::SuctionGripperConfig>
      suction_gripper_config;

  // Serializes this spec into an SDFormat XML string.
  // The generated plugin's name attribute will be
  // `<kPluginName><plugin_name_suffix>`.
  std::string ToSdformatXmlString(
      std::string_view plugin_name_suffix = "") const;
};

// Specification for generating ActuatedStickyGripperPlugin SDF XML.
// Typically used for pinch grippers with actuated joints where grasping creates
// an attachment on contact with a designated finger link.
struct ActuatedGripperPluginSpec {
  static constexpr char kPluginName[] = "ActuatedStickyGripperPlugin";

  // Object-local joint names for the actuated gripper finger joints.
  std::vector<std::string> joint_names;

  // Object-local link name for the gripper contact link (e.g. finger link)
  // where grasped objects attach.
  std::string sticky_link_name;

  // Intrinsic pubsub topic on which the plugin should publish status updates.
  std::string status_topic;

  // If this field is set, this actuated gripper will configure itself to work
  // with the GPIO server in simulation.
  std::optional<intrinsic_proto::eoat::PinchGripperConfig> pinch_gripper_config;

  // Configuration for the SimulatedPinchGripperService (gRPC & PubSub).
  std::optional<intrinsic_proto::gripper::PinchGripperConfig>
      service_pinch_gripper_config;

  // Identifier used for identifying this gripper in the simulated
  // PinchGripperService.
  // Should only be specified if `pinch_gripper_config` is unset or
  // `pinch_gripper_config` is set, but does not include a `name`.
  std::string pinch_gripper_handle;

  // Whether the gripper is closed when all joints are at their lower limit.
  // Leave it unset to use the plugin's default behavior.
  std::optional<bool> is_default_closed;

  // Serializes this spec into an SDFormat XML string.
  // The generated plugin's name attribute will be
  // `<kPluginName><plugin_name_suffix>`.
  std::string ToSdformatXmlString(
      std::string_view plugin_name_suffix = "") const;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_WORLD_GRIPPER_PLUGIN_SPEC_H_
