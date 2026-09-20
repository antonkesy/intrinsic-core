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

#include "intrinsic/simulation/gazebo/plugins/generic_action/generic_action_plugin.h"

#include <chrono>  // NOLINT
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "absl/synchronization/mutex.h"
#include "gloop/util/gtl/iterator_adaptors.h"
#include "google/protobuf/text_format.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointVelocityLimitsCmd.hh"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/simulation/gazebo/plugins/generic_action/generic_action_gpio_connection.h"
#include "intrinsic/simulation/gazebo/plugins/gpio/gpio_service.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/world/proto/generic_action.pb.h"
#include "sdf/Element.hh"

namespace intrinsic {
namespace simulation {

using gz::sim::components::JointPosition;
using gz::sim::components::JointVelocityCmd;
using gz::sim::components::JointVelocityLimitsCmd;
using ::intrinsic_proto::world::generic_action::DummyReadSignals;
using ::intrinsic_proto::world::generic_action::DummyWriteSignals;
using ::intrinsic_proto::world::generic_action::GenericAction;
using ::intrinsic_proto::world::generic_action::JointPositionCommand;
using ::intrinsic_proto::world::generic_action::ReadJointPosition;

namespace {
constexpr char kGenericActionElementName[] = "generic_action";

// Max target vel cmd to apply to joint.
// Consider making this parameter configurable
constexpr double kVelCmdMax = 1000;
}  // namespace

GenericActionPlugin::~GenericActionPlugin() {
  if (gpio_connection_ == nullptr) return;
  GPIOService& gpio_service = GPIOService::StartSingleton();
  if (!gpio_service
           .UnregisterPluginConnection(gpio_connection_->plugin_handle())
           .ok()) {
    LOG(ERROR) << "Failed to unregister gpio connection for plugin "
               << gpio_connection_->plugin_handle();
  }
  gpio_connection_ = nullptr;
}

void GenericActionPlugin::Configure(
    const Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& eventMgr) {
  ecm_ = &ecm;
  plugin_name_ = sdf->Get<std::string>("name");
  parent_entity_ = entity;
  LoadSDF(*sdf);
  ConfigureConnection();
}

void GenericActionPlugin::PreUpdate(const UpdateInfo& info,
                                    EntityComponentManager& ecm) {
  // Get time in seconds
  auto dt = std::chrono::duration<double>(info.dt).count();

  // Get the maximum amount that this joint may move
  double max_movement = kVelCmdMax * dt;

  absl::MutexLock lock(joint_position_cmds_mutex_);
  for (const auto& [entity, pos_cmd_vec] : joint_position_cmds_) {
    auto joint_comp = ecm.Component<JointPosition>(entity);
    if (!joint_comp) {
      LOG(WARNING) << "Unable to find joint positions for entity " << entity;
    }

    // Propotional controller - adapted from gazebo's joint position
    // controlller system.
    std::vector<double> pos_vec = joint_comp->Data();
    std::vector<double> target_vel_cmd_vec;
    for (unsigned int i = 0; i < pos_vec.size(); ++i) {
      double error = pos_vec[i] - pos_cmd_vec[i];
      double target_vel = 0.0;
      if (abs(error) > max_movement) {
        target_vel = (error < 0) ? kVelCmdMax : -kVelCmdMax;
      } else {
        target_vel = -error;
      }
      target_vel_cmd_vec.push_back(target_vel);
    }
    auto joint_vel_cmd_comp = ecm.Component<JointVelocityCmd>(entity);
    if (!joint_vel_cmd_comp) {
      ecm.CreateComponent(entity, JointVelocityCmd(target_vel_cmd_vec));
    } else {
      joint_vel_cmd_comp->Data() = target_vel_cmd_vec;
    }
  }
}

void GenericActionPlugin::PostUpdate(const UpdateInfo& info,
                                     const EntityComponentManager& ecm) {
  // Populate joint positions for match and read actions
  absl::MutexLock lock(joint_positions_mutex_);
  for (auto& [entity, positions] : joint_positions_) {
    auto joint_comp = ecm.Component<JointPosition>(entity);
    if (joint_comp) {
      positions = joint_comp->Data();
    } else {
      LOG(WARNING) << "Unable to find joint positions for entity " << entity;
    }
  }
}

void GenericActionPlugin::LoadSDF(const sdf::Element& sdf) {
  if (sdf.HasElement(kGenericActionElementName)) {
    std::string generic_action_string =
        sdf.Get<std::string>(kGenericActionElementName);
    intrinsic_proto::world::generic_action::GenericActionConfig config;
    if (!::google::protobuf::TextFormat::ParseFromString(generic_action_string,
                                                         &config)) {
      LOG(ERROR)
          << "Failed to parse plugin field " << kGenericActionElementName
          << " as a "
             "intrinsic_proto::world::generic_action::GenericActionConfig";
    }
    generic_action_config_.emplace(config);
  } else {
    LOG(ERROR) << absl::Substitute(
        "Cannot find sdf element <$0> for GenericActionPlugin",
        kGenericActionElementName);
  }
}

void GenericActionPlugin::ConfigureConnection() {
  if (!generic_action_config_.has_value()) {
    LOG(ERROR) << "Cannot configure gpio generic action plugin connection with "
                  "a valid generic action config";
    return;
  }

  QCHECK_EQ(gpio_connection_, nullptr)
      << "Cannot initialize gpio generic action connection twice for plugin "
      << plugin_name_ << ".";

  std::string plugin_handle =
      absl::Substitute("$0[$1]", plugin_name_, parent_entity_);
  gpio_connection_ =
      std::make_unique<GPIOGenericActionPluginConnection>(plugin_handle);
  for (const GenericAction& action : generic_action_config_->actions()) {
    switch (action.action_case()) {
      case GenericAction::kJointPositionCommand: {
        RegisterJointPositionCommand(parent_entity_,
                                     action.joint_position_command());
        break;
      }
      case GenericAction::kReadJointPosition: {
        RegisterReadJointPosition(parent_entity_, action.read_joint_position());
        break;
      }
      case GenericAction::kMatchJointPosition: {
        RegisterMatchJointPosition(parent_entity_,
                                   action.match_joint_position());
        break;
      }
      case GenericAction::kDummyReadSignals: {
        RegisterDummyReadAction(parent_entity_, action.dummy_read_signals());
        break;
      }
      case GenericAction::kDummyWriteSignals: {
        RegisterDummyWriteAction(parent_entity_, action.dummy_write_signals());
        break;
      }
      default: {
        CHECK(false) << "Unhandled generic action case " << action;
      }
    }
  }

  GPIOService& gpio_service = GPIOService::StartSingleton();
  if (auto status =
          gpio_service.RegisterPluginConnection(gpio_connection_.get());
      !status.ok()) {
    LOG(ERROR) << absl::Substitute(
        "Generic action plugin $0 failed to register its plugin connection "
        "with the simulated gpio service.",
        parent_entity_);
  }
}

void GenericActionPlugin::SetJointPositions(gz::sim::Entity joint_entity,
                                            const std::vector<double>& values) {
  absl::MutexLock lock(joint_position_cmds_mutex_);
  joint_position_cmds_[joint_entity] = values;
  LOG(INFO) << "Set joint positions for entity [" << joint_entity
            << "] to values: [" << absl::StrJoin(values, ", ") << "]";
}

void GenericActionPlugin::ReadJointPositions(gz::sim::Entity joint_entity,
                                             std::vector<double>& values) {
  absl::MutexLock lock(joint_positions_mutex_);
  if (!joint_positions_.contains(joint_entity)) {
    LOG(WARNING) << "Unable to read joint positions for entity "
                 << joint_entity;
    return;
  }
  values = joint_positions_[joint_entity];
}

void GenericActionPlugin::RegisterJointPositionCommand(
    const Entity& entity, const JointPositionCommand& joint_position_command) {
  gz::sim::Model model(entity);
  const std::string& joint_name =
      joint_position_command.joint_reference().joint_name();
  Entity joint = model.JointByName(*ecm_, joint_name);
  if (joint == gz::sim::kNullEntity) {
    LOG(ERROR) << "No joint with name " << joint_name << " found in entity "
               << entity;
  }

  auto joint_position_component = ecm_->Component<JointPosition>(joint);
  if (joint_position_component) {
    joint_position_component = ecm_->CreateComponent(joint, JointPosition());
  }
  QCHECK_NE(nullptr, joint_position_component);

  GPIOGenericActionPluginConnection::SetCommandCallback set_position_cb =
      [this, joint, &joint_position_command](
          const intrinsic_proto::gpio::v1::SignalValueSet& command_values) {
        std::vector<double> joint_positions;
        if (joint_position_command.has_target_position_command()) {
          const auto& target_pos_cmd =
              joint_position_command.target_position_command();
          for (unsigned int i = 0; i < target_pos_cmd.joint_positions_size();
               ++i) {
            joint_positions.push_back(target_pos_cmd.joint_positions(i));
          }
        } else {
          for (const auto& [name, value] : command_values.values()) {
            joint_positions.push_back(value.double_value());
          }
        }
        SetJointPositions(joint, joint_positions);
      };
  if (joint_position_command.has_generic_position_command()) {
    const auto& gpio_commands =
        joint_position_command.generic_position_command().gpio_commands();
    std::vector<intrinsic_proto::gpio::v1::SignalDescription> descriptions{
        gpio_commands.begin(), gpio_commands.end()};
    if (!gpio_connection_
             ->RegisterSetCommandCallback(descriptions, set_position_cb)
             .ok()) {
      LOG(ERROR) << "Failed to register gpio connection callback for "
                 << joint_position_command;
    }
  } else if (joint_position_command.has_target_position_command()) {
    const auto& gpio_signals =
        joint_position_command.target_position_command().gpio_signals();
    if (!gpio_connection_
             ->RegisterSetCommandCallback(gpio_signals, set_position_cb)
             .ok()) {
      LOG(ERROR) << "Failed to register gpio connection callback for "
                 << joint_position_command;
    }
  } else {
    LOG(ERROR) << "Unhandled registration of joint position command "
               << joint_position_command;
  }
}

void GenericActionPlugin::RegisterMatchJointPosition(
    const Entity& entity,
    const intrinsic_proto::world::generic_action::MatchJointPosition&
        match_joint_position) {
  gz::sim::Model model(entity);
  const std::string& joint_name =
      match_joint_position.joint_reference().joint_name();
  Entity joint = model.JointByName(*ecm_, joint_name);
  if (joint == gz::sim::kNullEntity) {
    LOG(ERROR) << "No joint with name " << joint_name << " found in entity "
               << entity;
    return;
  }

  // Create joint position component if one doesn't exist
  auto joint_position_component = ecm_->Component<JointPosition>(joint);
  if (joint_position_component == nullptr) {
    joint_position_component = ecm_->CreateComponent(joint, JointPosition());
  }
  QCHECK_NE(nullptr, joint_position_component);
  joint_position_component->Data().resize(
      match_joint_position.expected_joint_positions_size());

  GPIOGenericActionPluginConnection::GetStatusCallback match_status_cb =
      [match_joint_position, this, joint]() {
        std::vector<double> joint_positions;
        ReadJointPositions(joint, joint_positions);
        if (joint_positions.size() !=
            match_joint_position.expected_joint_positions_size()) {
          LOG(WARNING)
              << "Joint position size mismatch, returning non matching signals";
          return match_joint_position.non_matching_signals();
        }
        eigenmath::VectorXd expected = RepeatedDoubleToVectorXd(
            match_joint_position.expected_joint_positions());
        eigenmath::VectorXd actual = VectorToVectorXd(joint_positions);

        // Allow higher tol: 1 degree (~0.174 rad)
        if (!actual.isApprox(expected, 0.174)) {
          return match_joint_position.non_matching_signals();
        }
        return match_joint_position.matching_signals();
      };
  absl::flat_hash_set<std::string> signal_names;
  for (const auto& name :
       gtl::key_view(match_joint_position.matching_signals().values())) {
    signal_names.emplace(name);
  }
  auto register_status = gpio_connection_->RegisterGetStatusCallback(
      signal_names, match_status_cb);

  CHECK_OK(register_status)
      << "Failed to register gpio connection callback for " << register_status;
}

void GenericActionPlugin::RegisterDummyReadAction(
    const Entity& entity, const DummyReadSignals& dummy_read_signals) {
  CHECK_NE(gpio_connection_, nullptr);
  const auto& signals = dummy_read_signals.gpio_signals();

  absl::flat_hash_set<std::string> signal_names;
  for (const auto& [name, _] : signals.values()) {
    signal_names.emplace(name);
  }
  GPIOGenericActionPluginConnection::GetStatusCallback dummy_cb =
      [&signals]() -> intrinsic_proto::gpio::v1::SignalValueSet {
    return signals;
  };
  auto register_status =
      gpio_connection_->RegisterGetStatusCallback(signal_names, dummy_cb);
  CHECK_OK(register_status)
      << "Failed to register gpio connection callback for " << register_status;
}

void GenericActionPlugin::RegisterDummyWriteAction(
    const Entity& entity, const DummyWriteSignals& dummy_write_signals) {
  CHECK_NE(gpio_connection_, nullptr);

  GPIOGenericActionPluginConnection::SetCommandCallback set_position_cb =
      [](const intrinsic_proto::gpio::v1::SignalValueSet& command_values) {
        LOG(INFO) << "Dummy write ignoring command " << command_values;
      };

  const auto& gpio_signals = dummy_write_signals.gpio_signals();
  if (!gpio_connection_
           ->RegisterSetCommandCallback(gpio_signals, set_position_cb)
           .ok()) {
    LOG(ERROR) << "Failed to register gpio connection callback for "
               << dummy_write_signals;
  }
}

void GenericActionPlugin::RegisterReadJointPosition(
    const Entity& entity, const ReadJointPosition& read_joint_position) {
  CHECK_NE(gpio_connection_, nullptr);

  gz::sim::Model model(entity);
  const std::string& joint_name =
      read_joint_position.joint_reference().joint_name();
  Entity joint = model.JointByName(*ecm_, joint_name);
  if (joint == gz::sim::kNullEntity) {
    LOG(ERROR) << "No joint with name " << joint_name << " found in entity "
               << entity;
    return;
  }

  // Create joint position component if one doesn't exist
  auto joint_position_component = ecm_->Component<JointPosition>(joint);
  if (joint_position_component == nullptr) {
    joint_position_component = ecm_->CreateComponent(joint, JointPosition());
  }

  {
    absl::MutexLock lock(joint_positions_mutex_);
    joint_positions_[joint] = {};
  }

  const auto& gpio_output = read_joint_position.gpio_output();
  absl::flat_hash_set<std::string> signal_names;
  for (const auto& output : gpio_output) {
    signal_names.emplace(output.signal_name());
  }

  GPIOGenericActionPluginConnection::GetStatusCallback read_joint_cb =
      [joint_name, this, joint]() {
        intrinsic_proto::gpio::v1::SignalValueSet read_values;
        std::vector<double> joint_positions;
        ReadJointPositions(joint, joint_positions);
        auto& read_values_map = *read_values.mutable_values();
        for (unsigned int i = 0; i < joint_positions.size(); ++i) {
          std::string name =
              absl::StrCat("read_joint_position_", joint_name, "_", i);
          read_values_map[name].set_double_value(joint_positions[i]);
        }
        return read_values;
      };

  auto register_status =
      gpio_connection_->RegisterGetStatusCallback(signal_names, read_joint_cb);
  CHECK_OK(register_status)
      << "Failed to register gpio connection callback for " << register_status;
}

}  // namespace simulation
}  // namespace intrinsic
