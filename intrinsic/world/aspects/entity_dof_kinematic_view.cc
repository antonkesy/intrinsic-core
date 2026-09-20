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

#include "intrinsic/world/aspects/entity_dof_kinematic_view.h"

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/configuration_validation.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/labels.h"

namespace intrinsic {
namespace entity_kinematic_world_details {

ConstEntityDofKinematicView::ConstEntityDofKinematicView(
    const entity_aspect_world_details::EntityWorld* world,
    const std::vector<JointEntityId>& joints)
    : const_world_(world), joint_handles_(joints) {}

absl::Status ConstEntityDofKinematicView::SetDofValues(
    const eigenmath::VectorXd& dof_values, bool enforce_limits,
    std::optional<absl::Time> timestamp, bool enforce_monotonic_time) {
  LOG(FATAL)
      << "ConstEntityDofKinematicView::SetDofValues() should never be called";
}

eigenmath::VectorXd ConstEntityDofKinematicView::GetDofValues() const {
  eigenmath::VectorXd ret(joint_handles_.size());
  for (int i = 0; i < joint_handles_.size(); i++) {
    ret[i] = GetKinematicsComponent(i)->GetRawValue();
  }
  return ret;
}

size_t ConstEntityDofKinematicView::GetDofCount() const {
  return joint_handles_.size();
}

std::vector<DofLabel> ConstEntityDofKinematicView::GetDofLabels() const {
  std::vector<DofLabel> ret;
  ret.reserve(joint_handles_.size());
  for (int i = 0; i < joint_handles_.size(); i++) {
    ret.push_back(GetKinematicsComponent(i)->GetDofLabel());
  }
  return ret;
}

std::vector<DofId> ConstEntityDofKinematicView::GetAllDofIds() const {
  std::vector<DofId> ret;
  ret.reserve(joint_handles_.size());
  for (auto handle : joint_handles_) {
    ASSIGN_OR_DIE(DofId dof_id, const_world_->GetDofIdByEntityId(handle));
    ret.push_back(dof_id);
  }
  return ret;
}

const std::vector<JointEntityId>&
ConstEntityDofKinematicView::GetJointEntityIds() const {
  return joint_handles_;
}

eigenmath::VectorXd
ConstEntityDofKinematicView::GetLowerDofValueApplicationLimits() const {
  eigenmath::VectorXd ret(joint_handles_.size());
  for (int i = 0; i < joint_handles_.size(); i++) {
    ret[i] =
        GetKinematicsComponent(i)->GetApplicationRawValueFixedLimits().first;
  }
  return ret;
}

eigenmath::VectorXd
ConstEntityDofKinematicView::GetUpperDofValueApplicationLimits() const {
  eigenmath::VectorXd ret(joint_handles_.size());
  for (int i = 0; i < joint_handles_.size(); i++) {
    ret[i] =
        GetKinematicsComponent(i)->GetApplicationRawValueFixedLimits().second;
  }
  return ret;
}

std::pair<eigenmath::VectorXd, eigenmath::VectorXd>
ConstEntityDofKinematicView::GetDofValueApplicationLimits() const {
  eigenmath::VectorXd lower(joint_handles_.size());
  eigenmath::VectorXd upper(joint_handles_.size());
  for (int i = 0; i < joint_handles_.size(); i++) {
    const auto limits =
        GetKinematicsComponent(i)->GetApplicationRawValueFixedLimits();
    lower[i] = limits.first;
    upper[i] = limits.second;
  }
  return std::make_pair(std::move(lower), std::move(upper));
}

eigenmath::VectorXd ConstEntityDofKinematicView::GetLowerDofValueSystemLimits()
    const {
  eigenmath::VectorXd ret(joint_handles_.size());
  for (int i = 0; i < joint_handles_.size(); i++) {
    ret[i] = GetKinematicsComponent(i)->GetSystemRawValueFixedLimits().first;
  }
  return ret;
}

eigenmath::VectorXd ConstEntityDofKinematicView::GetUpperDofValueSystemLimits()
    const {
  eigenmath::VectorXd ret(joint_handles_.size());
  for (int i = 0; i < joint_handles_.size(); i++) {
    ret[i] = GetKinematicsComponent(i)->GetSystemRawValueFixedLimits().second;
  }
  return ret;
}

JointLimitsXd ConstEntityDofKinematicView::GetDofApplicationLimits() const {
  JointLimitsXd ret;
  ret.SetSize(joint_handles_.size());
  auto [lower_limits, upper_limits] = GetDofValueApplicationLimits();

  ret.min_position = std::move(lower_limits);
  ret.max_position = std::move(upper_limits);
  for (int i = 0; i < joint_handles_.size(); i++) {
    const auto* component = GetKinematicsComponent(i);
    ret.max_velocity[i] = component->GetApplicationVelocityLimit();
    ret.max_acceleration[i] = component->GetApplicationAccelerationLimit();
    ret.max_jerk[i] = component->GetApplicationJerkLimit();
    ret.max_torque[i] = component->GetApplicationEffortLimit();
  }
  return ret;
}

JointLimitsXd ConstEntityDofKinematicView::GetDofSystemLimits() const {
  JointLimitsXd ret;
  ret.SetSize(joint_handles_.size());
  ret.min_position = GetLowerDofValueSystemLimits();
  ret.max_position = GetUpperDofValueSystemLimits();
  for (int i = 0; i < joint_handles_.size(); i++) {
    const auto* component = GetKinematicsComponent(i);
    ret.max_velocity[i] = component->GetSystemVelocityLimit();
    ret.max_acceleration[i] = component->GetSystemAccelerationLimit();
    ret.max_jerk[i] = component->GetSystemJerkLimit();
    ret.max_torque[i] = component->GetSystemEffortLimit();
  }
  return ret;
}

absl::Status ConstEntityDofKinematicView::SetDofApplicationLimits(
    const JointLimitsXd& limits, bool enforce_limits) {
  LOG(FATAL)
      << "ConstEntityDofKinematicView::SetDofApplicationLimits() should never "
         "be called";
  return absl::UnimplementedError(
      "ConstEntityDofKinematicView::SetDofApplicationLimits() should never be "
      "called");
}

absl::Status ConstEntityDofKinematicView::SetDofSystemLimits(
    const JointLimitsXd& limits, bool enforce_limits) {
  LOG(FATAL) << "ConstEntityDofKinematicView::SetDofSystemLimits() should "
                "never be called";
  return absl::UnimplementedError(
      "ConstEntityDofKinematicView::SetDofSystemLimits() should never be "
      "called");
}

ConfigurationValidator
ConstEntityDofKinematicView::GetConfigurationLimitsValidator(
    ConfigurationValidatorOptions options) const {
  if (options & kUseRcsLimits) {
    // TODO(b/151856172): Reimplement RCS validation of joint values in a
    // formalized, data-driven way.
    LOG(WARNING) << "RCS limit validation of joint values currently disabled, "
                    "see b/151856172.";
  }
  if (options & kUseVariableLimits) {
    auto [lower_limits, upper_limits] = GetDofValueApplicationLimits();
    return [lower_limits,
            upper_limits](const eigenmath::VectorXd& new_values) -> bool {
      // Note: We cannot check that new_values and the limits are the same
      // length because the "6+1 DoF" robots were implemented by appending the
      // additional DoF's value without updating the ConfigurationValidator (see
      // intrinsic/choreographer/transformers/discrete_ik_transformer.cc;rcl=317644541;l=205).
      // Thus we can only check that new_values is not shorter than the limits.
      if (new_values.size() < lower_limits.size()) {
        return false;
      }
      for (int i = 0; i < lower_limits.size(); i++) {
        if (new_values[i] < lower_limits[i] ||
            new_values[i] > upper_limits[i]) {
          return false;
        }
      }
      return true;
    };
  }

  // If kUseVariableLimits is not set, return an "always true" validator.
  return [](const eigenmath::VectorXd& new_values) -> bool { return true; };
}

const KinematicsComponent* ConstEntityDofKinematicView::GetKinematicsComponent(
    int joint_handles_index) const {
  CHECK_GE(joint_handles_index, 0);
  CHECK_LT(joint_handles_index, joint_handles_.size());
  ASSIGN_OR_DIE(
      const WorldEntity* ent,
      const_world_->GetEntityById(joint_handles_[joint_handles_index]));
  ASSIGN_OR_DIE(auto component, ent->GetComponent<KinematicsComponent>());
  CHECK(component != nullptr);
  return component;
}

EntityDofKinematicView::EntityDofKinematicView(
    entity_aspect_world_details::EntityWorld* world,
    const std::vector<JointEntityId>& joints)
    : ConstEntityDofKinematicView(world, joints), world_(world) {}

absl::Status EntityDofKinematicView::SetDofValues(
    const eigenmath::VectorXd& dof_values, bool enforce_limits,
    std::optional<absl::Time> timestamp, bool enforce_monotonic_time) {
  if (dof_values.size() != joint_handles_.size()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "dof_values contains " << dof_values.size()
           << " values, expected " << joint_handles_.size();
  }

  if (enforce_monotonic_time) {
    if (timestamp.has_value()) {
      for (int i = 0; i < joint_handles_.size(); i++) {
        const auto* component = GetKinematicsComponent(i);
        auto current_timestamp = component->GetTimestamp();
        if (current_timestamp.has_value() && *timestamp < *current_timestamp) {
          return absl::InvalidArgumentError(
              "Timestamp is earlier than current timestamp for joint.");
        }
      }
    } else {
      bool all_nullopt = true;
      for (int i = 0; i < joint_handles_.size(); i++) {
        const auto* component = GetKinematicsComponent(i);
        if (component->GetTimestamp().has_value()) {
          all_nullopt = false;
          break;
        }
      }
      if (!all_nullopt) {
        return absl::InvalidArgumentError(
            "Timestamp is required when some joints already have timestamps "
            "and enforce_monotonic_time is true.");
      }
    }
  }

  for (int i = 0; i < dof_values.size(); i++) {
    INTR_RETURN_IF_ERROR(world_->SetDofRawValue(
        joint_handles_[i], dof_values[i], enforce_limits, timestamp));
  }
  return absl::OkStatus();
}

absl::Status EntityDofKinematicView::SetDofApplicationLimits(
    const JointLimitsXd& limits, bool enforce_limits) {
  if (!limits.IsValid()) {
    return absl::InvalidArgumentError("limit are invalid.");
  }

  int dof_count = GetDofCount();
  if (dof_count != limits.size()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "DoF count mismatch: provided limits cover " << limits.size()
           << " DoFs, world's robot expects " << dof_count << " DoFs";
  }

  // Before modifying any limits, make sure that the current positions are
  // within the new limits.
  eigenmath::VectorXd current_values = GetDofValues();
  for (int i = 0; i < dof_count; ++i) {
    if (current_values[i] < limits.min_position[i] ||
        current_values[i] > limits.max_position[i]) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "DoF at index " << i << " has value " << current_values[i]
             << " outside of new limits [" << limits.min_position[i] << ", "
             << limits.max_position[i] << "]";
    }
  }

  for (int i = 0; i < joint_handles_.size(); i++) {
    auto component = GetKinematicsComponent(i);
    if (std::isfinite(limits.min_position[i]) &&
        std::isfinite(limits.max_position[i])) {
      INTR_RETURN_IF_ERROR(component->SetApplicationRawValueFixedLimits(
          limits.min_position[i], limits.max_position[i], enforce_limits));
    }
    if (std::isfinite(limits.max_velocity[i])) {
      INTR_RETURN_IF_ERROR(component->SetApplicationVelocityLimit(
          limits.max_velocity[i], enforce_limits));
    }
    if (std::isfinite(limits.max_acceleration[i])) {
      INTR_RETURN_IF_ERROR(component->SetApplicationAccelerationLimit(
          limits.max_acceleration[i], enforce_limits));
    }
    if (std::isfinite(limits.max_jerk[i])) {
      INTR_RETURN_IF_ERROR(component->SetApplicationJerkLimit(
          limits.max_jerk[i], enforce_limits));
    }
    if (std::isfinite(limits.max_torque[i])) {
      INTR_RETURN_IF_ERROR(component->SetApplicationEffortLimit(
          limits.max_torque[i], enforce_limits));
    }
  }
  return absl::OkStatus();
}

absl::Status EntityDofKinematicView::SetDofSystemLimits(
    const JointLimitsXd& limits, bool enforce_limits) {
  if (!limits.IsValid()) {
    return absl::InvalidArgumentError("limits are invalid.");
  }

  int dof_count = GetDofCount();
  if (dof_count != limits.size()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "DoF count mismatch: provided limits cover " << limits.size()
           << " DoFs, world's robot expects " << dof_count << " DoFs";
  }

  // Before modifying any limits, make sure that the current positions are
  // within the new limits.
  eigenmath::VectorXd current_values = GetDofValues();
  for (int i = 0; i < dof_count; ++i) {
    if (current_values[i] < limits.min_position[i] ||
        current_values[i] > limits.max_position[i]) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "DoF at index " << i << " has value " << current_values[i]
             << " outside of new limits [" << limits.min_position[i] << ", "
             << limits.max_position[i] << "]";
    }
  }

  for (int i = 0; i < joint_handles_.size(); i++) {
    auto component = GetKinematicsComponent(i);
    if (std::isfinite(limits.min_position[i]) &&
        std::isfinite(limits.max_position[i])) {
      INTR_RETURN_IF_ERROR(component->SetSystemRawValueFixedLimits(
          limits.min_position[i], limits.max_position[i], enforce_limits));
    }
    if (std::isfinite(limits.max_velocity[i])) {
      INTR_RETURN_IF_ERROR(component->SetSystemVelocityLimit(
          limits.max_velocity[i], enforce_limits));
    }
    if (std::isfinite(limits.max_acceleration[i])) {
      INTR_RETURN_IF_ERROR(component->SetSystemAccelerationLimit(
          limits.max_acceleration[i], enforce_limits));
    }
    if (std::isfinite(limits.max_jerk[i])) {
      INTR_RETURN_IF_ERROR(
          component->SetSystemJerkLimit(limits.max_jerk[i], enforce_limits));
    }
    if (std::isfinite(limits.max_torque[i])) {
      INTR_RETURN_IF_ERROR(component->SetSystemEffortLimit(limits.max_torque[i],
                                                           enforce_limits));
    }
  }
  return absl::OkStatus();
}

KinematicsComponent* EntityDofKinematicView::GetKinematicsComponent(
    int joint_handles_index) {
  CHECK_GE(joint_handles_index, 0);
  CHECK_LT(joint_handles_index, joint_handles_.size());
  ASSIGN_OR_DIE(WorldEntity * ent,
                world_->GetEntityById(joint_handles_[joint_handles_index]));
  ASSIGN_OR_DIE(auto component, ent->GetComponent<KinematicsComponent>());
  CHECK(component != nullptr);
  return component;
}

}  // namespace entity_kinematic_world_details
}  // namespace intrinsic
