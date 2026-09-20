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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_limits.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_limits_constants.h"
#include "intrinsic/icon/control/parts/proto/v1/mode_of_safe_operation_limits_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/flatbuffers/transform_view.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_limits_utils.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"

namespace intrinsic::icon {

namespace {
using ::intrinsic_fbs::EnumNameModeOfSafeOperation;
using ::intrinsic_fbs::EnumValuesModeOfSafeOperation;
using ::intrinsic_fbs::ModeOfSafeOperation;

// Returns the largest valid application limits, given `system_limits`.
JointLimits GetMaxAdmissibleApplicationLimits(
    const JointLimits& system_limits) {
  JointLimits max_admissible_application_limits = system_limits;
  max_admissible_application_limits.max_velocity *=
      kMaxApplicationLimitsMultiplier;
  max_admissible_application_limits.max_acceleration *=
      kMaxApplicationLimitsMultiplier;
  max_admissible_application_limits.max_jerk *= kMaxApplicationLimitsMultiplier;
  return max_admissible_application_limits;
}

icon::RealtimeStatusOr<
    JointLimitsInterface::JointAccelerationLimitsFromDynamics>
ComputeJointAccelerationLimitsFromDynamics(
    RigidBodyInterface* rigid_body_dynamics,
    const HardwareInterfaceHandle<intrinsic_fbs::JointPositionCommand>&
        joint_position_command_hardware_interface,
    const JointLimits& system_limits) {
  // Update previous setpoint.
  Eigen::Map<const intrinsic::eigenmath::VectorNd> joint_position =
      intrinsic_fbs::ViewAs<::intrinsic::eigenmath::VectorNd>(
          joint_position_command_hardware_interface->position());
  Eigen::Map<const intrinsic::eigenmath::VectorNd> joint_velocity =
      intrinsic_fbs::ViewAs<::intrinsic::eigenmath::VectorNd>(
          joint_position_command_hardware_interface->velocity_feedforward());
  // Update min/max acceleration limits.
  JointLimitsInterface::JointAccelerationLimitsFromDynamics
      joint_acceleration_limits_from_dynamics =
          JointLimitsInterface::JointAccelerationLimitsFromDynamics();
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      joint_acceleration_limits_from_dynamics
          .joint_acceleration_limits_at_min_torque,
      rigid_body_dynamics->ComputeForwardDynamics(
          joint_position, joint_velocity, -system_limits.max_torque));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      joint_acceleration_limits_from_dynamics
          .joint_acceleration_limits_at_max_torque,
      rigid_body_dynamics->ComputeForwardDynamics(
          joint_position, joint_velocity, system_limits.max_torque));

  return joint_acceleration_limits_from_dynamics;
}

}  // namespace

absl::StatusOr<JointLimitsFeature> JointLimitsFeature::Create(
    JointLimits system_limits, JointLimits application_limits,
    const intrinsic_proto::icon::v1::ModeOfSafeOperationLimitsConfig& config,
    std::optional<JointPositionCommandHardwareInterface>
        joint_position_command_hardware_interface,
    std::unique_ptr<RigidBodyInterface> rigid_body_dynamics,
    std::optional<MutableJointLimitHardwareInterface>
        joint_system_limits_hardware_interface) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LimitCheckResult limit_check_result,
      IsWithinLimits(application_limits,
                     GetMaxAdmissibleApplicationLimits(system_limits)));
  if (!limit_check_result.v_ok) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Provided joint limits are inconsistent. The application velocity "
        "limits must be at least 5% less than the system velocity limits. "
        "Limit check returns ",
        ToFixedString(limit_check_result), "\nApplication velocity limits: \n",
        intrinsic::ToProto(application_limits).max_velocity(),
        "\nSystem velocity limits: \n",
        intrinsic::ToProto(system_limits).max_velocity()));
  }
  if (!limit_check_result.a_ok) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Provided joint limits are inconsistent. The application "
        "acceleration limits must be at least 5% less than the maximum "
        "acceleration limits. Limit check returns ",
        ToFixedString(limit_check_result),
        "\nApplication acceleration limits: \n",
        intrinsic::ToProto(application_limits).max_acceleration(),
        "\nSystem acceleration limits: \n",
        intrinsic::ToProto(system_limits).max_acceleration()));
  }
  if (!limit_check_result.j_ok) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Provided joint limits are inconsistent. The application jerk limits "
        "must be at least 5% less than the maximum jerk limits. Limit check "
        "returns ",
        ToFixedString(limit_check_result), "\nApplication jerk limits: \n",
        intrinsic::ToProto(application_limits).max_jerk(),
        "\nSystem jerk limits: \n",
        intrinsic::ToProto(system_limits).max_jerk()));
  }

  // All limit errors not related to VAJ.
  if (!limit_check_result) {
    return absl::InvalidArgumentError(
        absl::StrCat("Application limits do not leave a sufficient margin. "
                     "They violate the maximum admissible limits: ",
                     ToFixedString(limit_check_result)));
  }
  absl::flat_hash_map<ModeOfSafeOperation, JointLimitBundle> limits = {
      {ModeOfSafeOperation::UNKNOWN,
       {.application_limits = application_limits,
        .system_limits = system_limits}}};

  if (config.has_t1_limits()) {
    // Adjusts the application limits for Teaching 1.
    // Sets the system joint velocity limits at most to the value defined in
    // `t1_limit_params` and the application limits to
    // kMaxApplicationLimitsMultiplier times that. Adjusts a copy of the
    // "fallback" limits that is later inserted into the map.
    JointLimitBundle t1_limits = limits.at(ModeOfSafeOperation::UNKNOWN);
    // Assigns the minimal value of the max_velocity and the t1_limit for every
    // dof. Assumes that max_velocity is positive.
    t1_limits.system_limits.max_velocity =
        t1_limits.system_limits.max_velocity.cwiseMin(
            config.t1_limits().joint_max_velocity());
    // The application velocity is at most kMaxApplicationLimitsMultiplier *
    // max_velocity for a joint.
    t1_limits.application_limits.max_velocity =
        t1_limits.application_limits.max_velocity.cwiseMin(
            t1_limits.system_limits.max_velocity *
            kMaxApplicationLimitsMultiplier);
    limits[ModeOfSafeOperation::TEACH_PENDANT_1] = t1_limits;
  }
  JointLimitBundle fallback_limits = limits.at(ModeOfSafeOperation::UNKNOWN);
  // Fill in the remaining limits for all 'ModeOfSafeOperation's in
  // intrinsic/icon/control/safety/safety_messages.fbs
  for (const auto mode : EnumValuesModeOfSafeOperation()) {
    auto it = limits.find(mode);
    if (it == limits.end()) {
      LOG(WARNING) << "No explicit limits defined for ModeOfSafeOperation::"
                   << EnumNameModeOfSafeOperation(mode)
                   << " Using fallback limits.";
      limits[mode] = fallback_limits;
    } else {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          LimitCheckResult limit_check_result,
          IsWithinLimits(
              it->second.application_limits,
              GetMaxAdmissibleApplicationLimits(it->second.system_limits)));
      if (!limit_check_result) {
        return absl::InvalidArgumentError(
            absl::StrCat("Application limits for safety mode '",
                         EnumNameModeOfSafeOperation(mode),
                         "' do not leave a sufficient margin. "
                         "They violate the maximum admissible limits: ",
                         ToFixedString(limit_check_result)));
      }
    }
  }

  std::vector<JointLimitBundle> limits_vector(limits.size());
  for (const auto mode : EnumValuesModeOfSafeOperation()) {
    limits_vector[static_cast<uint8_t>(mode)] = limits[mode];
  }

  // Ensures that all cases of ModeOfSafeOperation in
  // intrinsic/icon/control/safety/safety_messages.fbs are covered.
  // This check should never trigger, as the JointLimitsFeature factory
  // populates limits_in_enum_order.
  QCHECK(limits_vector.size() ==
         static_cast<uint8_t>(ModeOfSafeOperation::MAX) + 1)
      << "There needs to be a LimitBundle for every ModeOfSafeOperation. And "
         "the enum values need to be sequential starting with zero.";

  return JointLimitsFeature(
      /*limits_in_enum_order=*/absl::MakeSpan(limits_vector),
      std::move(joint_position_command_hardware_interface),
      std::move(rigid_body_dynamics),
      std::move(joint_system_limits_hardware_interface));
}

JointLimitsFeature::JointLimitsFeature(
    absl::Span<JointLimitBundle> limits_in_enum_order,
    std::optional<JointPositionCommandHardwareInterface>
        joint_position_command_hardware_interface,
    std::unique_ptr<icon::RigidBodyInterface> rigid_body_dynamics,
    std::optional<MutableJointLimitHardwareInterface>
        joint_system_limits_hardware_interface)
    : limits_in_enum_order_(
          {limits_in_enum_order.begin(), limits_in_enum_order.end()}),
      joint_acceleration_limits_from_dynamics_(std::nullopt),
      joint_position_command_hardware_interface_(
          std::move(joint_position_command_hardware_interface)),
      joint_system_limits_hardware_interface_(
          std::move(joint_system_limits_hardware_interface)),
      rigid_body_dynamics_(std::move(rigid_body_dynamics)) {}

JointLimits JointLimitsFeature::GetApplicationLimits() const {
  return GetLimitBundleForModeOfSafeOperation(
             current_safety_status_.mode_of_safe_operation)
      .application_limits;
}
JointLimits JointLimitsFeature::GetSystemLimits() const {
  return GetLimitBundleForModeOfSafeOperation(
             current_safety_status_.mode_of_safe_operation)
      .system_limits;
}

icon::RealtimeStatusOr<
    std::optional<JointLimitsInterface::JointAccelerationLimitsFromDynamics>>
JointLimitsFeature::GetJointAccelerationLimitsFromDynamics() const {
  std::optional<JointLimitsInterface::JointAccelerationLimitsFromDynamics>
      joint_acceleration_limits_from_dynamics =
          joint_acceleration_limits_from_dynamics_;
  if (rigid_body_dynamics_ &&
      joint_position_command_hardware_interface_.has_value() &&
      !joint_acceleration_limits_from_dynamics_.has_value()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        joint_acceleration_limits_from_dynamics,
        ComputeJointAccelerationLimitsFromDynamics(
            rigid_body_dynamics_.get(),
            joint_position_command_hardware_interface_.value(),
            GetSystemLimits()));
    joint_acceleration_limits_from_dynamics_ =
        joint_acceleration_limits_from_dynamics;
  }
  return joint_acceleration_limits_from_dynamics;
}

RealtimeStatus JointLimitsFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  current_safety_status_ = params.safety_status;
  joint_acceleration_limits_from_dynamics_ = std::nullopt;

  return OkStatus();
}

RealtimeStatus JointLimitsFeature::ApplyCommand(
    RealtimePartInterface::ApplyCommandParameters params) {
  if (joint_system_limits_hardware_interface_.has_value()) {
    INTRINSIC_RT_RETURN_IF_ERROR(
        CopyTo(GetSystemLimits(), ***joint_system_limits_hardware_interface_));
  }
  return OkStatus();
}

const JointLimitsFeature::JointLimitBundle&
JointLimitsFeature::GetLimitBundleForModeOfSafeOperation(
    intrinsic_fbs::ModeOfSafeOperation mode) const {
  // Construction ensures that there is a LimitBundle for every
  // ModeOfSafeOperation.
  return limits_in_enum_order_.at(static_cast<uint8_t>(mode));
}

}  // namespace intrinsic::icon
