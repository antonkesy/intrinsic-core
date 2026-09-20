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

#include "intrinsic/icon/control/primitives/force_control/action_builder.h"

#include <array>
#include <vector>

#include "Eigen/Eigenvalues"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_admittance_info.h"
#include "intrinsic/icon/actions/cartesian_impedance.pb.h"
#include "intrinsic/icon/actions/cartesian_impedance_reference_generators.pb.h"
#include "intrinsic/icon/actions/force_primitive_info.h"
#include "intrinsic/icon/control/algorithms/compute_critical_damping.h"
#include "intrinsic/icon/control/algorithms/directional_stiffness.h"
#include "intrinsic/icon/control/algorithms/overdamping.h"
#include "intrinsic/icon/control/algorithms/selection_matrix_helpers.h"
#include "intrinsic/icon/control/primitives/force_control/force_primitive_utils.h"
#include "intrinsic/icon/control/primitives/force_control/proto/controller_params.pb.h"
#include "intrinsic/icon/control/primitives/force_control/proto/force_primitives.pb.h"
#include "intrinsic/icon/equipment/force_control_settings.pb.h"
#include "intrinsic/icon/proto/cart_space.pb.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/kinematics/types/joint_limits.pb.h"
#include "intrinsic/manipulation/skills/force/contact_stiffness.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto/pose.pb.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/math/twist.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon::force_primitive {

using ::intrinsic_proto::force::AlignRotationWithFrame;
using ::intrinsic_proto::force::ApplyForce;
using ::intrinsic_proto::force::AxisAndTorque;
using ::intrinsic_proto::force::ControllerParams;
using ::intrinsic_proto::force::DirectionAndForce;
using ::intrinsic_proto::force::ForcePrimitive;
using ::intrinsic_proto::force::Hold;
using ::intrinsic_proto::force::MakeContact;
using ::intrinsic_proto::icon::ForceControlSettings;
using ::intrinsic_proto::icon::actions::proto::CartesianImpedanceParameters;
using ::intrinsic_proto::icon::actions::proto::ReferenceGenerators;
using ::intrinsic_proto::manipulation::skills::ContactStiffnessParams;
using ::intrinsic_proto::manipulation::skills::ContactStiffnessValues;

// Translational stiffness orthogonal to the applied force.
inline constexpr double kDefaultTranslationalStiffnessOrthogonal = 300.0;
// Rotational stiffness orthogonal to the axis in which a force is applied.
inline constexpr double kDefaultRotationalStiffnessOrthogonal = 300.0;
// Rotational stiffness used for aligning the orientation of the tool.
inline constexpr double kDefaultRotationalStiffnessForOrientationAlignment =
    100.0;
// Setting the lowpass filter constant to 1.0 disables the lowpass filter.
inline constexpr double kLowpassFilterDisabled = 1.0;
// Default duration used for latch detection.
inline constexpr double kDefaultLatchDetectionDurationSeconds = 1.0;

const std::array<eigenmath::Vector3d, 3> kAllDirections = {
    eigenmath::Vector3d::UnitX(), eigenmath::Vector3d::UnitY(),
    eigenmath::Vector3d::UnitZ()};

namespace {
void ApplyDefaultControllerParams(ControllerParams& controller_params) {
  if (!controller_params
           .has_translational_stiffness_orthogonal_to_direction()) {
    controller_params.set_translational_stiffness_orthogonal_to_direction(
        kDefaultTranslationalStiffnessOrthogonal);
  }
  if (!controller_params.has_rotational_stiffness_orthogonal_to_direction()) {
    controller_params.set_rotational_stiffness_orthogonal_to_direction(
        kDefaultRotationalStiffnessOrthogonal);
  }
}

void ApplyDefaultControllerParams(
    AlignRotationWithFrame::ControllerParams& controller_params) {
  if (!controller_params.has_rotational_stiffness_rx()) {
    controller_params.set_rotational_stiffness_rx(
        kDefaultRotationalStiffnessForOrientationAlignment);
  }
  if (!controller_params.has_rotational_stiffness_ry()) {
    controller_params.set_rotational_stiffness_ry(
        kDefaultRotationalStiffnessForOrientationAlignment);
  }
  if (!controller_params.has_rotational_stiffness_rz()) {
    controller_params.set_rotational_stiffness_rz(
        kDefaultRotationalStiffnessForOrientationAlignment);
  }
}

// This method takes a stiffness matrix and replaces near-zero eigenvalues with
// the highest value for directions which have stiffness or returns a diagonal
// stiffness matrix with the `fallback_stiffness` value.
absl::StatusOr<eigenmath::MatrixNd> ModifyStiffnessForDampingCalculation(
    const eigenmath::MatrixNd& stiffness_matrix, double fallback_stiffness) {
  const double kMinimumEigenvalue = 1e-10;

  if (fallback_stiffness <= 0.0) {
    return absl::InvalidArgumentError(
        "The `fallback_stiffness` should be greater than 0.");
  }
  // We set any missing stiffness components to the eigenvalue which is greater
  // than the threshold.
  const Eigen::SelfAdjointEigenSolver<eigenmath::MatrixNd> solver(
      stiffness_matrix);
  eigenmath::VectorNd evals = solver.eigenvalues();
  eigenmath::MatrixNd V = solver.eigenvectors();
  // If we don't have any positive eigenvalues this is becasue we have no
  // stiffness in any direction. In this case use the fallback stiffness.
  if (evals.maxCoeff() < kMinimumEigenvalue) {
    return fallback_stiffness *
           eigenmath::MatrixNd::Identity(stiffness_matrix.rows(),
                                         stiffness_matrix.cols());
  }
  for (auto& val : evals) {
    if (val < kMinimumEigenvalue) {
      val = evals.maxCoeff();
    }
  }
  // Reconstruct matrix based on updated eigenvalues.
  eigenmath::MatrixNd new_stiffness_matrix =
      eigenmath::MatrixNd(V * evals.asDiagonal() * V.inverse());
  return new_stiffness_matrix;
}

absl::Status SetDampingForAdmittanceControl(
    const eigenmath::Matrix6d inertia,
    const ContactStiffnessParams& stiffness_params,
    CartesianImpedanceParameters& params) {
  INTR_ASSIGN_OR_RETURN(const double environment_stiffness,
                        GetEnvironmentStiffness(stiffness_params));

  INTR_ASSIGN_OR_RETURN(const eigenmath::Matrix6d damping,
                        ComputeOverdamping(inertia, environment_stiffness));
  *params.mutable_cartesian_target()->mutable_cartesian_damping() =
      ToProto(damping);

  return absl::OkStatus();
}

absl::Status SetDampingForImpedanceControl(
    const eigenmath::Matrix6d inertia, CartesianImpedanceParameters& params) {
  INTR_ASSIGN_OR_RETURN(
      eigenmath::Matrix6d target_stiffness,
      FromProto(params.cartesian_target().cartesian_stiffness()));
  INTR_ASSIGN_OR_RETURN(target_stiffness.topLeftCorner(3, 3),
                        ModifyStiffnessForDampingCalculation(
                            target_stiffness.topLeftCorner(3, 3),
                            kDefaultTranslationalStiffnessOrthogonal));
  INTR_ASSIGN_OR_RETURN(target_stiffness.bottomRightCorner(3, 3),
                        ModifyStiffnessForDampingCalculation(
                            target_stiffness.bottomRightCorner(3, 3),
                            kDefaultRotationalStiffnessOrthogonal));
  INTR_ASSIGN_OR_RETURN(const eigenmath::Matrix6d damping_critical,
                        ComputeCriticalDamping(inertia, target_stiffness));
  *params.mutable_cartesian_target()->mutable_cartesian_damping() =
      ToProto(damping_critical);

  return absl::OkStatus();
}

void SetConstraints(const ForceControlSettings& force_control_settings,
                    CartesianImpedanceParameters& params) {
  if (force_control_settings.has_enforce_cartesian_and_joint_limits()) {
    // This enables the experimental Jerk limited admittance controller.
    params.mutable_constraints()->set_enforce_cartesian_and_joint_limits(
        force_control_settings.enforce_cartesian_and_joint_limits());
  }
}

absl::Status ApplyMakeContact(const ForcePrimitiveInfo::FixedParams& params,
                              CartesianImpedanceParameters& output_params) {
  const MakeContact& make_contact = params.force_primitive().make_contact();
  INTR_ASSIGN_OR_RETURN(
      eigenmath::Vector3d motion_direction_in_task_frame,
      UnitDirectionFromProto(make_contact.motion_direction()));
  Wrench wrench = Wrench::ZERO;
  wrench.head(3) =
      motion_direction_in_task_frame * make_contact.max_contact_force();
  *output_params.mutable_cartesian_target()->mutable_tool_reference_wrench() =
      ToProto(wrench);

  ControllerParams controller_params =
      params.force_primitive().make_contact().controller_params();

  std::vector<eigenmath::Vector3d> controlled_translational_directions(
      kAllDirections.begin(), kAllDirections.end());
  std::vector<eigenmath::Vector3d> controlled_rotational_directions(
      kAllDirections.begin(), kAllDirections.end());
  if (controller_params.has_no_translational_compliance()) {
    controlled_translational_directions = {motion_direction_in_task_frame};
  }
  if (controller_params.has_no_rotational_compliance()) {
    controlled_rotational_directions = {};
  }

  INTR_ASSIGN_OR_RETURN(
      eigenmath::Matrix6d wrench_selection_matrix,
      ComputeWrenchSelectionMatrixForCompliantMotionInSubspaceSpannedBy(
          controlled_translational_directions,
          controlled_rotational_directions));

  *output_params.mutable_cartesian_target()->mutable_wrench_selection_matrix() =
      ToProto(wrench_selection_matrix);

  ApplyDefaultControllerParams(controller_params);

  INTR_ASSIGN_OR_RETURN(
      eigenmath::Matrix6d stiffness,
      ComputeDirectionalStiffness(
          {motion_direction_in_task_frame}, /*rotational_directions=*/{},
          {controller_params.translational_stiffness_orthogonal_to_direction(),
           controller_params.rotational_stiffness_orthogonal_to_direction()}));
  *output_params.mutable_cartesian_target()->mutable_cartesian_stiffness() =
      ToProto(stiffness);

  *output_params.mutable_state_variable_configuration()
       ->mutable_force_direction_1_in_task() =
      ToVectorProto(motion_direction_in_task_frame);

  return absl::OkStatus();
}

absl::Status SetApplyForce(const ForcePrimitiveInfo::FixedParams& params,
                           CartesianImpedanceParameters& output_params) {
  const ApplyForce& apply_force = params.force_primitive().apply_force();
  if (apply_force.forces().size() > 3) {
    return absl::InvalidArgumentError(
        "ApplyForce must have at most three forces.");
  }
  if (apply_force.torques().size() > 3) {
    return absl::InvalidArgumentError(
        "ApplyForce must have at most three torques.");
  }
  if (apply_force.forces().empty() && apply_force.torques().empty()) {
    return absl::InvalidArgumentError(
        "ApplyForce must have at least one force or torque.");
  }
  std::vector<eigenmath::Vector3d> motion_directions_in_task;
  motion_directions_in_task.reserve(apply_force.forces().size());
  eigenmath::Vector3d force_sum_in_task = eigenmath::Vector3d::Zero();
  for (const DirectionAndForce& force : apply_force.forces()) {
    INTR_ASSIGN_OR_RETURN(eigenmath::Vector3d motion_direction_in_task,
                          UnitDirectionFromProto(force.direction()));
    motion_directions_in_task.push_back(motion_direction_in_task);
    force_sum_in_task += motion_direction_in_task.normalized() * force.force();
  }
  std::vector<eigenmath::Vector3d> torque_axes_in_task;
  torque_axes_in_task.reserve(apply_force.torques().size());
  eigenmath::Vector3d torque_sum_in_task = eigenmath::Vector3d::Zero();
  for (const AxisAndTorque& torque : apply_force.torques()) {
    INTR_ASSIGN_OR_RETURN(eigenmath::Vector3d torque_axis_in_task,
                          UnitDirectionFromProto(torque.axis()));
    torque_axes_in_task.push_back(torque_axis_in_task);
    torque_sum_in_task += torque_axis_in_task.normalized() * torque.torque();
  }

  Wrench wrench = Wrench::ZERO;
  wrench.head(3) = force_sum_in_task;
  wrench.tail<3>() = torque_sum_in_task;
  *output_params.mutable_cartesian_target()->mutable_tool_reference_wrench() =
      ToProto(wrench);

  ControllerParams controller_params =
      params.force_primitive().apply_force().controller_params();

  std::vector<eigenmath::Vector3d> controlled_translational_directions(
      kAllDirections.begin(), kAllDirections.end());
  std::vector<eigenmath::Vector3d> controlled_rotational_directions(
      kAllDirections.begin(), kAllDirections.end());
  if (controller_params.has_no_translational_compliance()) {
    controlled_translational_directions = motion_directions_in_task;
  }
  if (controller_params.has_no_rotational_compliance()) {
    controlled_rotational_directions = torque_axes_in_task;
  }

  INTR_ASSIGN_OR_RETURN(
      eigenmath::Matrix6d wrench_selection_matrix,
      ComputeWrenchSelectionMatrixForCompliantMotionInSubspaceSpannedBy(
          controlled_translational_directions,
          controlled_rotational_directions));

  *output_params.mutable_cartesian_target()->mutable_wrench_selection_matrix() =
      ToProto(wrench_selection_matrix);

  ApplyDefaultControllerParams(controller_params);

  INTR_ASSIGN_OR_RETURN(
      eigenmath::Matrix6d stiffness,
      ComputeDirectionalStiffness(
          motion_directions_in_task, torque_axes_in_task,
          {controller_params.translational_stiffness_orthogonal_to_direction(),
           controller_params.rotational_stiffness_orthogonal_to_direction()}));
  *output_params.mutable_cartesian_target()->mutable_cartesian_stiffness() =
      ToProto(stiffness);

  if (!motion_directions_in_task.empty()) {
    *output_params.mutable_state_variable_configuration()
         ->mutable_force_direction_1_in_task() =
        ToVectorProto(motion_directions_in_task[0]);
  }
  if (motion_directions_in_task.size() > 1) {
    *output_params.mutable_state_variable_configuration()
         ->mutable_force_direction_2_in_task() =
        ToVectorProto(motion_directions_in_task[1]);
  }
  if (motion_directions_in_task.size() > 2) {
    *output_params.mutable_state_variable_configuration()
         ->mutable_force_direction_3_in_task() =
        ToVectorProto(motion_directions_in_task[2]);
  }

  if (!torque_axes_in_task.empty()) {
    *output_params.mutable_state_variable_configuration()
         ->mutable_torque_axis_1_in_task() =
        ToVectorProto(torque_axes_in_task[0]);
  }
  if (torque_axes_in_task.size() > 1) {
    *output_params.mutable_state_variable_configuration()
         ->mutable_torque_axis_2_in_task() =
        ToVectorProto(torque_axes_in_task[1]);
  }
  if (torque_axes_in_task.size() > 2) {
    *output_params.mutable_state_variable_configuration()
         ->mutable_torque_axis_3_in_task() =
        ToVectorProto(torque_axes_in_task[2]);
  }

  return absl::OkStatus();
}

absl::StatusOr<Pose3d> GetTaskToReference(
    const AlignRotationWithFrame& align_with_frame) {
  if (align_with_frame.reference_frame().has_task_frame_t_reference_frame()) {
    return FromProto(
        align_with_frame.reference_frame().task_frame_t_reference_frame());
  }
  if (align_with_frame.reference_frame().has_task_frame()) {
    return Pose3d::Identity();
  }
  if (align_with_frame.reference_frame().has_reference()) {
    return absl::InvalidArgumentError(
        "AlignRotationWithFrame has a reference frame defined in the world, "
        "provide a reference pose instead.");
  }

  return absl::InvalidArgumentError(
      "AlignRotationWithFrame does not have a reference frame.");
}

absl::Status SetAlignRotationWithFrame(
    const ForcePrimitiveInfo::FixedParams& params,
    CartesianImpedanceParameters& output_params) {
  const AlignRotationWithFrame& align_with_frame =
      params.force_primitive().align_rotation_with_frame();
  INTR_ASSIGN_OR_RETURN(const Pose3d task_t_reference,
                        GetTaskToReference(align_with_frame));
  // Set the reference position and orientation. The position is ignored ,
  // because the translational stiffness is zero. This is needed because there
  // is no option to set only a desired orientation for the Admittance control
  // action.
  *output_params.mutable_cartesian_target()
       ->mutable_task_t_tool_reference_pose() = icon::ToProto(task_t_reference);

  eigenmath::Matrix6d stiffness = eigenmath::Matrix6d::Zero();
  Wrench desired_wrench = Wrench::ZERO;

  if (align_with_frame.has_force()) {
    INTR_ASSIGN_OR_RETURN(
        const eigenmath::Vector3d motion_direction_in_task_frame,
        UnitDirectionFromProto(align_with_frame.force().direction()));
    desired_wrench.head(3) =
        motion_direction_in_task_frame * align_with_frame.force().force();
  }

  AlignRotationWithFrame::ControllerParams controller_params =
      params.force_primitive().align_rotation_with_frame().controller_params();
  ApplyDefaultControllerParams(controller_params);

  stiffness.diagonal().tail<3>() =
      eigenmath::Vector3d{controller_params.rotational_stiffness_rx(),
                          controller_params.rotational_stiffness_ry(),
                          controller_params.rotational_stiffness_rz()};

  *output_params.mutable_cartesian_target()->mutable_cartesian_stiffness() =
      ToProto(stiffness);
  *output_params.mutable_cartesian_target()->mutable_tool_reference_wrench() =
      ToProto(desired_wrench);

  return absl::OkStatus();
}

absl::Status SetHold(const ForcePrimitiveInfo::FixedParams& params,
                     CartesianImpedanceParameters& output_params) {
  ControllerParams controller_params =
      params.force_primitive().hold().controller_params();

  // We use all directions to be sensitive to external wrenches.
  std::vector<eigenmath::Vector3d> controlled_translational_directions(
      kAllDirections.begin(), kAllDirections.end());
  std::vector<eigenmath::Vector3d> controlled_rotational_directions(
      kAllDirections.begin(), kAllDirections.end());
  if (controller_params.has_no_translational_compliance()) {
    controlled_translational_directions = {};
  }
  if (controller_params.has_no_rotational_compliance()) {
    controlled_rotational_directions = {};
  }

  ApplyDefaultControllerParams(controller_params);

  INTR_ASSIGN_OR_RETURN(
      const eigenmath::Matrix6d wrench_selection_matrix,
      ComputeWrenchSelectionMatrixForCompliantMotionInSubspaceSpannedBy(
          controlled_translational_directions,
          controlled_rotational_directions));

  *output_params.mutable_cartesian_target()->mutable_wrench_selection_matrix() =
      ToProto(wrench_selection_matrix);

  *output_params.mutable_cartesian_target()->mutable_tool_reference_wrench() =
      ToProto(Wrench::ZERO);

  // The translational_directions and rotational_directions are empty here so
  // that the controller has stiffness in all directions.
  INTR_ASSIGN_OR_RETURN(
      eigenmath::Matrix6d stiffness,
      ComputeDirectionalStiffness(
          /*translational_directions=*/{}, /*rotational_directions=*/{},
          {controller_params.translational_stiffness_orthogonal_to_direction(),
           controller_params.rotational_stiffness_orthogonal_to_direction()}));
  *output_params.mutable_cartesian_target()->mutable_cartesian_stiffness() =
      ToProto(stiffness);

  return absl::OkStatus();
}

void SetReferenceGenerators(const ForcePrimitiveInfo::FixedParams& params,
                            ReferenceGenerators& reference_generators) {
  // Use the default simple lowpass reference generator.
  reference_generators.mutable_cartesian()->mutable_simple_lowpass();
  reference_generators.mutable_nullspace()->mutable_simple_lowpass();
  if (params.disable_reference_lowpass_filter()) {
    reference_generators.mutable_cartesian()
        ->mutable_simple_lowpass()
        ->set_lowpass_filter_constant(kLowpassFilterDisabled);
    reference_generators.mutable_nullspace()
        ->mutable_simple_lowpass()
        ->set_lowpass_filter_constant(kLowpassFilterDisabled);
  }
}

}  // namespace

absl::StatusOr<ActionParams> BuildParams(
    const ForcePrimitiveInfo::FixedParams& input_params) {
  ForcePrimitiveInfo::FixedParams params = input_params;

  CartesianAdmittanceInfo::FixedParams fixed_params;
  CartesianImpedanceParameters& impedance_params =
      *fixed_params.mutable_cartesian_impedance_parameters();

  SetConstraints(params.force_control_settings(), impedance_params);

  INTR_ASSIGN_OR_RETURN(
      Pose3d tip_t_tool,
      FromProto(params.task_params().robot_tip_t_robot_tool()));

  *impedance_params.mutable_cartesian_target()
       ->mutable_robot_tip_t_robot_tool() = icon::ToProto(tip_t_tool);

  INTR_ASSIGN_OR_RETURN(Pose3d base_t_task,
                        FromProto(params.task_params().robot_base_t_task()));
  *impedance_params.mutable_cartesian_target()->mutable_robot_base_t_task() =
      icon::ToProto(base_t_task);

  if (params.task_params().has_joint_limits()) {
    *impedance_params.mutable_constraints()->mutable_joint_limits() =
        params.task_params().joint_limits();
  }
  if (params.task_params().has_cartesian_limits()) {
    *impedance_params.mutable_constraints()->mutable_cartesian_limits() =
        params.task_params().cartesian_limits();
  }

  SetReferenceGenerators(params,
                         *impedance_params.mutable_reference_generators());

  impedance_params.mutable_state_variable_configuration()
      ->set_time_window_for_maximum_displacement_sec(
          kDefaultLatchDetectionDurationSeconds);

  *impedance_params.mutable_algorithm_configuration()
       ->mutable_sensed_wrench_deadband() =
      params.force_control_settings().sensed_wrench_deadband();

  switch (params.force_primitive().primitive_case()) {
    case ForcePrimitive::kMakeContact: {
      INTR_RETURN_IF_ERROR(ApplyMakeContact(params, impedance_params));
      break;
    }
    case ForcePrimitive::kApplyForce: {
      INTR_RETURN_IF_ERROR(SetApplyForce(params, impedance_params));
      break;
    }
    case ForcePrimitive::kAlignRotationWithFrame: {
      INTR_RETURN_IF_ERROR(SetAlignRotationWithFrame(params, impedance_params));
      break;
    }
    case ForcePrimitive::kHold: {
      INTR_RETURN_IF_ERROR(SetHold(params, impedance_params));
      break;
    }
    case ForcePrimitive::PRIMITIVE_NOT_SET:
      return absl::InvalidArgumentError("Force primitive not set.");
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported force primitive: ",
                       params.force_primitive().primitive_case()));
  }

  eigenmath::Matrix6d inertia =
      VirtualInertiaFromSettings(params.force_control_settings());
  *impedance_params.mutable_cartesian_target()
       ->mutable_virtual_cartesian_inertia() = ToProto(inertia);

  // Damping calculation for admittance and impedance control schemes differs
  // due to requirements for stabilility at contact. For admittance control we
  // need to considerably overdamp the system and follow the approach proposed
  // by D. Surdilovic, "Contact stability issues in position based impedance
  // control: theory and experiments," , 1996, , doi: 10.1109/ROBOT.1996.506953.
  // On the other hand for impedance control this can actually be problematic
  // due to the velocity feedback and the significant noise which is introduced.
  // In this case the classical critically damped approach proposed in the
  // original impedance control papers is sufficient: N. Hogan, "Impedance
  // Control: An Approach to Manipulation,"1984, pp. 304-313,
  // doi: 10.23919/ACC.1984.4788393.
  if (params.force_control_settings().has_use_impedance_control() &&
      params.force_control_settings().use_impedance_control()) {
    if (!(params.task_params()
                  .environment_stiffness()
                  .contact_stiffness_case() ==
              ContactStiffnessParams::kDefaultContactStiffness &&
          params.task_params()
                  .environment_stiffness()
                  .default_contact_stiffness() ==
              ContactStiffnessValues::CONTACT_STIFFNESS_UNSPECIFIED)) {
      return absl::InvalidArgumentError(
          "The setting of environmental stiffness is only applicable to "
          "admittance control but the robot is using impedance control. Please "
          "set the environment stiffness to unspecified.");
    }
    INTR_RETURN_IF_ERROR(
        SetDampingForImpedanceControl(inertia, impedance_params));
  } else {
    INTR_RETURN_IF_ERROR(SetDampingForAdmittanceControl(
        inertia, params.task_params().environment_stiffness(),
        impedance_params));
  }

  return fixed_params;
}

}  // namespace intrinsic::icon::force_primitive
