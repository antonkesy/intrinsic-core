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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GENERIC_ACTION_GENERIC_ACTION_PLUGIN_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GENERIC_ACTION_GENERIC_ACTION_PLUGIN_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "gtest/gtest_prod.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "intrinsic/simulation/gazebo/plugins/generic_action/generic_action_gpio_connection.h"
#include "intrinsic/world/proto/generic_action.pb.h"
#include "sdf/Element.hh"

namespace intrinsic {
namespace simulation {

class GenericActionPlugin final : public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate,
                                  public gz::sim::ISystemPostUpdate {
 public:
  using Entity = gz::sim::Entity;
  using EntityComponentManager = gz::sim::EntityComponentManager;
  using EventManager = gz::sim::EventManager;
  using UpdateInfo = gz::sim::UpdateInfo;

  void Configure(const Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 EntityComponentManager& ecm, EventManager& eventMgr) override;

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
      ABSL_LOCKS_EXCLUDED(joint_position_cmds_mutex_);

  void PostUpdate(const UpdateInfo& info,
                  const EntityComponentManager& ecm) override
      ABSL_LOCKS_EXCLUDED(joint_positions_mutex_);

  ~GenericActionPlugin() override;

 private:
  void LoadSDF(const sdf::Element& sdf);
  void ConfigureConnection();

  void SetJointPositions(gz::sim::Entity joint_entity,
                         const std::vector<double>& values)
      ABSL_LOCKS_EXCLUDED(joint_position_cmds_mutex_);

  void ReadJointPositions(gz::sim::Entity joint_entity,
                          std::vector<double>& values)
      ABSL_LOCKS_EXCLUDED(joint_positions_mutex_);

  void RegisterJointPositionCommand(
      const Entity& entity,
      const intrinsic_proto::world::generic_action::JointPositionCommand&
          joint_position_command);

  void RegisterMatchJointPosition(
      const Entity& entity,
      const intrinsic_proto::world::generic_action::MatchJointPosition&
          match_joint_position);

  void RegisterDummyReadAction(
      const Entity& entity,
      const intrinsic_proto::world::generic_action::DummyReadSignals&
          dummy_read_signals);

  void RegisterDummyWriteAction(
      const Entity& entity,
      const intrinsic_proto::world::generic_action::DummyWriteSignals&
          dummy_write_signals);

  void RegisterReadJointPosition(
      const Entity& entity,
      const intrinsic_proto::world::generic_action::ReadJointPosition&
          read_joint_position);

  std::string plugin_name_;
  Entity parent_entity_;

  // A pointer to a EntityComponentManager. Remains valid until the simulation
  // server is reset/destroyed.
  gz::sim::EntityComponentManager* ecm_ = nullptr;

  // The connection this plugin registers with the simulated GPIO server
  std::unique_ptr<GPIOGenericActionPluginConnection> gpio_connection_ = nullptr;

  std::optional<intrinsic_proto::world::generic_action::GenericActionConfig>
      generic_action_config_ = std::nullopt;

  absl::flat_hash_map<gz::sim::Entity, std::vector<double>> joint_position_cmds_
      ABSL_GUARDED_BY(joint_position_cmds_mutex_);
  absl::flat_hash_map<gz::sim::Entity, std::vector<double>> joint_positions_
      ABSL_GUARDED_BY(joint_positions_mutex_);

  absl::Mutex joint_positions_mutex_;
  absl::Mutex joint_position_cmds_mutex_;

  FRIEND_TEST(GenericActionPluginTest, ValidConfigure);
  FRIEND_TEST(GenericActionPluginTest, SetCommandValues);
  FRIEND_TEST(GenericActionPluginTest, GetSignalValues);
  FRIEND_TEST(GenericActionPluginTest, RegisterAndUnregister);
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GENERIC_ACTION_GENERIC_ACTION_PLUGIN_H_
