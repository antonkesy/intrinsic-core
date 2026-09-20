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

#include "intrinsic/simulation/gazebo/plugins/grippers/fixed_joint_gripper.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "absl/base/log_severity.h"
#include "absl/container/flat_hash_set.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "intrinsic/simulation/gazebo/components/articulation_component.h"
#include "intrinsic/simulation/gazebo/components/interaction_component.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/gripper.pb.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/link_articulation_util.h"
#include "intrinsic/simulation/gazebo/plugins/link_device_util.h"
#include "intrinsic/util/status/return.h"
#include "intrinsic/util/status/status_macros.h"

// Flag to enable / disable removal of the ContactSensorData component once
// contact data are not needed. i.e. when gripper is not in GRASP mode.
// Setting this to true will let the simulator continue to retrieve and fill
// contact data every iteration which can potentially be expensive. On the
// other hand, if contact data are already in use, e.g. by other plugins, the
// gripper plugin should not remove the component.
// TODO(b/353307051) Once contact data retrieval performance is improved, we
// can consider removing this flag and always keep the contact sensor data.
ABSL_FLAG(bool, fixed_joint_gripper_keep_contact_sensor_data, false,
          "If true, FixedJointGripper does not remove the "
          "ContactSensorData component once enabled.");

namespace intrinsic {
namespace simulation {

using gz::sim::Entity;
using gz::sim::kNullEntity;
using gz::sim::components::ChildLinkName;
using gz::sim::components::ContactSensorData;
using gz::sim::components::JointType;
using gz::sim::components::ParentLinkName;
using NameComponent = gz::sim::components::Name;
using ::intrinsic_proto::simulation::gazebo::GripperCommand;
using ::intrinsic_proto::simulation::gazebo::GripperStatus;

namespace {
// Log helper function to get an entity's name. If it does not have a name
// component, the entity id is returned as a string.
std::string EntityName(Entity entity,
                       const gz::sim::EntityComponentManager& ecm) {
  const auto* name_component = ecm.Component<NameComponent>(entity);
  return name_component == nullptr ? std::to_string(entity)
                                   : name_component->Data();
}

constexpr int kVerboseLogIntervalSeconds = 5;
}  // namespace

FixedJointGripper::FixedJointGripper(
    const std::string& plugin_name, const std::string& gripper_link_name,
    double grasp_debounce_time_seconds, gz::sim::Link gripper_link,
    std::vector<::gz::sim::Entity> collision_test_entities)
    : plugin_name_(plugin_name),
      gripper_link_(gripper_link),
      collision_test_entities_(std::move(collision_test_entities)),
      gripper_link_name_(gripper_link_name),
      grasp_debounce_time_seconds_(grasp_debounce_time_seconds) {
  status_.set_grasp_status(GripperStatus::DETACHED);
  status_.set_blowoff_status(GripperStatus::BLOWOFF_OFF);
  command_.set_command(GripperCommand::RELEASE);
}

bool FixedJointGripper::IsGrasping() const {
  return status_.grasp_status() == GripperStatus::ATTACHED;
}

// The gripper link name is used to obtain the corresponding Link entity from
// the ECM. We also obtain the collision entity associated with the test volume
// that we use to determine which object the gripper is about to grasp. Finally,
// we add a ContactSensorData component to the collision entity to indicate to
// the Physics System that it should populate contacts after the time
// integration step for this entity.
absl::StatusOr<std::unique_ptr<FixedJointGripper>> FixedJointGripper::Create(
    const gz::sim::Entity& plugin_parent, absl::string_view plugin_name,
    absl::string_view gripper_link_name, double grasp_debounce_time_seconds,
    gz::sim::EntityComponentManager& ecm) {
  INTR_ASSIGN_OR_RETURN(auto link_entity, GetLinkEntityFromRelativeScopedName(
                                              gripper_link_name,
                                              kLinkDeviceLinkNameScopeSeparator,
                                              plugin_parent, ecm));

  gz::sim::Link gripper_link(link_entity);

  if (!gripper_link.Valid(ecm)) {
    LOG(ERROR) << "Could not get a valid entity for gripper link with name "
               << gripper_link_name << ". Gripper will not work for "
               << plugin_name;
  }
  // add ContactSensorData component so that the Physics engine populates it
  // with the contact information
  std::vector<gz::sim::Entity> collision_test_entities =
      gripper_link.Collisions(ecm);
  if (collision_test_entities.empty()) {
    LOG(ERROR) << "Gripper link " << gripper_link_name << " has no collisions."
               << " Gripper will not work for " << plugin_name;
  }

  // Add Interaction component so that we know this is a robot tip entity.
  // TODO(b/296597289): Move this to a separate annotation plugin.
  Interaction* interaction_component = ecm.CreateComponent(
      gripper_link.Entity(), Interaction(InteractionType::kRobotTip));
  if (interaction_component == nullptr) {
    LOG(ERROR) << "Couldn't create Interaction component for gripper link "
               << gripper_link_name;
  }

  return std::make_unique<FixedJointGripper>(
      std::string(plugin_name), std::string(gripper_link_name),
      grasp_debounce_time_seconds, gripper_link,
      std::move(collision_test_entities));
}

void FixedJointGripper::OnCommand(const GripperCommand& command_msg) {
  auto command = command_msg.command();
  if (command == command_.command()) return;

  if (command == GripperCommand::UNKNOWN) {
    LOG_EVERY_N_SEC(WARNING, kVerboseLogIntervalSeconds)
        << "Ignoring unknown command for " << plugin_name_;
    return;
  }

  auto current_time = absl::Now();
  auto earliest_next_action_time =
      time_last_action_change_ + absl::Seconds(grasp_debounce_time_seconds_);
  if (current_time < earliest_next_action_time) {
    LOG_EVERY_N_SEC(WARNING, kVerboseLogIntervalSeconds)
        << "Dropping gripper command for " << plugin_name_
        << " because it was received too soon from the previous command."
        << " Last command time: " << time_last_action_change_;
  } else {
    if (command_.command() == GripperCommand::GRASP &&
        (command == GripperCommand::RELEASE ||
         command == GripperCommand::BLOWOFF_ON)) {
      LOG_IF(WARNING, !IsGrasping())
          << "Switching from grasp to release "
             "command before a successful grasp for gripper "
          << plugin_name_;
    }

    time_last_action_change_ = current_time;
    command_.set_command(command);
  }
}

void FixedJointGripper::ProcessCommand(gz::sim::EntityComponentManager& ecm) {
  if (command_.command() == GripperCommand::GRASP && !IsGrasping()) {
    if (grasped_object_link_entity_ != kNullEntity) {
      absl::StatusOr<Entity> detachable_joint = AddDetachableJoint(
          gripper_link_.Entity(), grasped_object_link_entity_, ecm);
      if (!detachable_joint.ok()) {
        LOG_EVERY_N_SEC(ERROR, kVerboseLogIntervalSeconds)
            << "Failed to add fixed joint between " << gripper_link_name_
            << " and " << EntityName(grasped_object_link_entity_, ecm)
            << " for gripper " << plugin_name_;
      } else {
        detachable_joint_entity_ = *detachable_joint;
        status_.set_grasp_status(GripperStatus::ATTACHED);
        LOG(INFO) << "Grasped " << EntityName(grasped_object_link_entity_, ecm)
                  << " with gripper " << plugin_name_;
      }
    } else {
      LOG_EVERY_POW_2(INFO)
          << "Received grasp command but no object is present.";
    }
  }

  // TODO(b/244454115): Treat blowoff on as a release request. Implement a
  // physical blowoff force when needed.
  if ((command_.command() == GripperCommand::RELEASE ||
       command_.command() == GripperCommand::BLOWOFF_ON) &&
      IsGrasping()) {
    const std::string grasped_object_name =
        EntityName(grasped_object_link_entity_, ecm);
    if (detachable_joint_entity_ == kNullEntity) {
      LOG_EVERY_N_SEC(ERROR, kVerboseLogIntervalSeconds)
          << "Received grasp release command but " << plugin_name_
          << " is not grasping anything.";
    } else {
      if (!RemoveDetachableJoint(detachable_joint_entity_, ecm).ok()) {
        LOG_EVERY_N_SEC(ERROR, kVerboseLogIntervalSeconds)
            << "Could not detach object " << grasped_object_name
            << " for gripper " << plugin_name_;
      } else {
        detachable_joint_entity_ = kNullEntity;
        grasped_object_link_entity_ = kNullEntity;
      }
    }
    // conservatively set detached status so we don't block execution
    status_.set_grasp_status(GripperStatus::DETACHED);
    LOG(INFO) << "Detached " << grasped_object_name << " with gripper "
              << plugin_name_;
  }

  // Update contact sensor data component to signal to the Physics System
  // whether or not contact sensor data is required for this gripper. This is an
  // optimization to avoid unnecessary data copy from the physics engine.
  UpdateContactSensorDataComponent(ecm);

  // Update ECM values of ArticulationType for links that the gripper has
  // been in contact with.
  UpdateArticulationComponent(ecm);
}

void FixedJointGripper::UpdateGraspedObject(
    const gz::sim::EntityComponentManager& ecm) {
  if (collision_test_entities_.empty()) return;
  if (IsGrasping()) return;

  // Use a hash set to merge contacts together based on the parent link element.
  // multiple contacts might occur between two links. we only care about
  // grasping the link in contact, not about all the different collision
  // elements that are in contact.
  absl::flat_hash_set<Entity> colliding_links;
  bool any_contact_sensor_data_enabled = false;

  // TODO(b/515016200): Refine contact selection for identifying graspable
  // objects. Currently, the following criteria is quite loose and might result
  // in an unexpected object being grasped in a cluttered workspace.
  for (auto collision_entity : collision_test_entities_) {
    std::optional<gz::msgs::Contacts> contact_data_optional =
        ecm.ComponentData<ContactSensorData>(collision_entity);

    if (!contact_data_optional.has_value()) {
      continue;
    }
    any_contact_sensor_data_enabled = true;

    const auto& contact_data = contact_data_optional.value();
    for (size_t i = 0; i < contact_data.contact_size(); i++) {
      Entity parent_link = kNullEntity;
      auto& collision1 = contact_data.contact(i).collision1();
      if (collision1.id() != collision_entity) {
        parent_link = ecm.ParentEntity(collision1.id());
      }
      auto& collision2 = contact_data.contact(i).collision2();
      if (collision2.id() != collision_entity) {
        // safe to overwrite since only one of the parent links can be different
        // from gripper_link_.
        parent_link = ecm.ParentEntity(collision2.id());
      }
      if (parent_link == gripper_link_.Entity()) {
        LOG_EVERY_N_SEC(ERROR, kVerboseLogIntervalSeconds)
            << "Gripper link is in collision itself for " << plugin_name_;
      } else {
        const bool already_has_link = !colliding_links.empty();
        LOG_IF_EVERY_N_SEC(INFO, already_has_link, kVerboseLogIntervalSeconds)
            << "Gripper link is in collision with multiple links for "
            << plugin_name_;

        colliding_links.insert(parent_link);
      }
    }
  }

  // If no contact sensor data has been enabled yet, don't reset
  // grasped_object_link_entity_. This ensures that we don't report that no
  // object is present prematurely in `ProcessCommand`.
  if (!any_contact_sensor_data_enabled) {
    return;
  }

  grasped_object_link_entity_ = kNullEntity;

  // grasp the first non-static, floating object.
  for (const auto& link_entity : colliding_links) {
    gz::sim::Link link(link_entity);
    if (link.Valid(ecm)) {
      INTR_ASSIGN_OR_RETURN(
          ArticulationType articulation, GetArticulationType(link, ecm),
          _.LogEvery(absl::LogSeverity::kError,
                     absl::Seconds(kVerboseLogIntervalSeconds))
              .With(ReturnVoid()));
      link_to_articulation_type_cache_[link_entity] = articulation;
      if (articulation == ArticulationType::kFloating &&
          !link.ParentModel(ecm)->Static(ecm)) {
        grasped_object_link_entity_ = link_entity;
        break;
      }
    }
  }
}

gz::sim::Entity FixedJointGripper::GraspedLink() const {
  return IsGrasping() ? grasped_object_link_entity_ : kNullEntity;
}

void FixedJointGripper::UpdateContactSensorDataComponent(
    gz::sim::EntityComponentManager& ecm) const {
  if (collision_test_entities_.empty()) return;
  const bool need_contact_sensor_data =
      command_.command() == GripperCommand::GRASP && !IsGrasping();
  if (absl::GetFlag(FLAGS_fixed_joint_gripper_keep_contact_sensor_data) &&
      !need_contact_sensor_data) {
    return;
  }
  for (auto entity : collision_test_entities_) {
    gz::sim::enableComponent<ContactSensorData>(
        ecm, entity, /*_enable=*/need_contact_sensor_data);
  }
}

void FixedJointGripper::UpdateArticulationComponent(
    gz::sim::EntityComponentManager& ecm) {
  for (const auto& [link_entity, articulation_type] :
       link_to_articulation_type_cache_) {
    if (std::optional<ArticulationType> ecm_articulation =
            ecm.ComponentData<Articulation>(link_entity);
        ecm_articulation.has_value()) {
      LOG_IF(ERROR, ecm_articulation != articulation_type)
          << "Computed articulation type for link '"
          << EntityName(link_entity, ecm)
          << "' does not match value in the ECM!! Overwriting value.";
    }
    ecm.SetComponentData<Articulation>(link_entity, articulation_type);
  }
  link_to_articulation_type_cache_.clear();
}

}  // namespace simulation
}  // namespace intrinsic
