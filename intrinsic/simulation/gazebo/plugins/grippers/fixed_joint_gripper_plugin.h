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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_FIXED_JOINT_GRIPPER_PLUGIN_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_FIXED_JOINT_GRIPPER_PLUGIN_H_

#include <memory>
#include <optional>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "gz/msgs/MessageTypes.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "gz/transport/Node.hh"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/fixed_joint_gripper.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/gpio_gripper_plugin_connection.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/gripper.pb.h"
#include "sdf/Element.hh"

namespace intrinsic {
namespace simulation {

// An implementation of a Gripper in Gazebo that "welds" the grasped object to
// the gripper with a fixed joint when commanded to grip.
class FixedJointGripperPlugin : public ::gz::sim::System,
                                public ::gz::sim::ISystemConfigure,
                                public ::gz::sim::ISystemPreUpdate,
                                public ::gz::sim::ISystemPostUpdate {
 public:
  ~FixedJointGripperPlugin() override;

  void Configure(const ::gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 ::gz::sim::EntityComponentManager& ecm,
                 ::gz::sim::EventManager& eventMgr) override;

  void PreUpdate(const ::gz::sim::UpdateInfo& info,
                 ::gz::sim::EntityComponentManager& ecm) override;

  void PostUpdate(const ::gz::sim::UpdateInfo& info,
                  const ::gz::sim::EntityComponentManager& ecm) override;

  bool DidReceiveCommandGrip() const;
  bool DidReceiveCommandRelease() const;

  bool IsGrasping() const;

  // returns kNullEntity if no entity is currently grasped
  ::gz::sim::Entity GraspedLink() const;

  absl::Status GetInitializationStatus();

 protected:
  std::string plugin_name_;
  ::gz::sim::Entity parent_entity_;

  absl::Status LoadSDF(sdf::Element* sdf);
  void OnDioCommand(const ::gz::msgs::UInt32& command);
  absl::Status OnGpioCommand(
      const intrinsic_proto::simulation::gazebo::GripperCommand& command);
  void OnCommand(
      const intrinsic_proto::simulation::gazebo::GripperCommand& command);
  void PublishStatus();

  std::string command_topic_;
  std::string status_topic_;
  // TODO(qingyou): Remove int32 based dio pub sub when dio command from
  // dio_device are no longer needed.
  std::string dio_command_topic_;
  std::string dio_status_topic_;

  std::string gripper_link_name_;

  ::gz::transport::Node node_;
  ::gz::transport::Node::Publisher dio_status_pub_;
  ::gz::transport::Node::Publisher status_pub_;

 private:
  std::unique_ptr<FixedJointGripper> fixed_joint_gripper_;

  static constexpr char kCommandElementName[] = "gripper_command_topic";
  static constexpr char kStatusElementName[] = "gripper_status_topic";

  // TODO(b/190554052): Use sdformat's support for parsing from template instead
  static constexpr char kDioCommandElementName[] = "dio_command_topic";
  static constexpr char kDioStatusElementName[] = "dio_status_topic";

  intrinsic_proto::simulation::gazebo::GripperCommand FromDioCommand(
      int dio_command) const;

  // The grip and release commands need to match the bit pattern set for the dio
  // device of the physical gripper I/O interface for the gripper to work
  //  properly with ICON.
  int dio_command_grip_;
  static constexpr char kDioCommandGripElementName[] = "dio_command_grip";

  int dio_command_release_;
  static constexpr char kDioCommandReleaseElementName[] = "dio_command_release";

  int GetDioStatus() const;

  int dio_status_attached_;
  static constexpr char kDioStatusAttachedElementName[] = "dio_status_attached";

  int dio_status_detached_;
  static constexpr char kDioStatusDetachedElementName[] = "dio_status_detached";

  // Defines what link elements hold children (collisions) that are contact
  // points for a grip.
  static constexpr char kGripperLinkElementName[] = "gripper_link";

  // Block the Attach command after a Detach to give the object some time to
  // move away. A value of 0 results in some detachment failures for simulated
  // detachments.
  static constexpr char kGraspDebounceTimeSecondsElementName[] =
      "seconds_to_detach";
  static constexpr double kDefaultGraspDebounceTimeSeconds = 0.5;
  double grasp_debounce_time_seconds_;

  // Optionally stores the gpio signals needed to simulate a EOAT suction
  // gripper. The signals are used to configure gpio_connection_
  std::optional<intrinsic_proto::eoat::SuctionGripperConfig>
      suction_gripper_config_;
  static constexpr char kSuctionGripperConfigElementName[] =
      "suction_gripper_config";

  // Initializes gpio_connection_ with the given suction_gripper_config and
  // registers this plugin with the simulated GPIO gRPC service.
  void InitGPIOGripperConnection(
      const intrinsic_proto::eoat::SuctionGripperConfig&
          suction_gripper_config);
  // Destroys gpio_connection_ if any and de-registers this plugin from the
  // simulated GPIO gRPC service.
  void DestroyGPIOGripperConnection();
  // Bridge between this plugin and the simulated GPIO gRPC service. The
  // connection will be initialized with callbacks for the simulated GPIO gRPC
  // service to send commands to this plugin without depending on
  // FixedJointGripperPlugin
  std::unique_ptr<GPIOGripperPluginConnection> gpio_connection_;
  std::string gpio_connection_handle_;

  // Time the last attach or detach occurred.
  absl::Time time_last_action_change_ = absl::InfinitePast();

  absl::Mutex initialization_status_mutex_;
  absl::Status initialization_status_
      ABSL_GUARDED_BY(initialization_status_mutex_) = absl::UnavailableError(
          "FixedJointGripperPlugin hasn't finished initializing.");
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_FIXED_JOINT_GRIPPER_PLUGIN_H_
