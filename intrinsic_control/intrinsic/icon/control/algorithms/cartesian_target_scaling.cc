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

#include "intrinsic/icon/control/algorithms/cartesian_target_scaling.h"

#include <algorithm>

#include "intrinsic/eigenmath/rotation_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/reference_limit_settings.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

namespace {
// Determines the scale factor of motion scaling within an allowable range
// between a reference position and a sensed position. max_distance is the
// maximum distance tolerated between the reference and sensed position. At
// max_distance*activation_ratio, the scaling factor starts to decrease from 1
// to 0, where 0 is reached at max_distance. This scaling factor can be used to
// scale position, velocity, and acceleration to keep the reference within the
// allowable range. The scaling function is smooth and differentiable.
double GetScaleFactor(double distance, double max_distance,
                      double activation_ratio) {
  double min_distance = max_distance * activation_ratio;
  double scale_factor = 1.0;

  if (distance <= min_distance) {
    scale_factor = 1.0;
  } else if (distance >= max_distance) {
    scale_factor = 0.0;
  } else {
    // Normalize distance to be in [0,1].
    double normalized_distance =
        (distance - min_distance) / (max_distance - min_distance);
    double x = std::clamp(normalized_distance, 0.0, 1.0);
    // The bisquare function is a smooth/differentiable interpolation
    // between 0 and 1.
    scale_factor = (1. - x * x) * (1.0 - x * x);
  }
  return scale_factor;
}
}  // namespace

bool ScaleTargetCommand(const TrajectoryGenerationMode& mode,
                        const CartStatePV& target_state,
                        const CartStatePVA& current_state,
                        const CartStatePVA& current_reference_state,
                        const CartStatePVA& next_reference_state,
                        const ReferenceLimitSettings& settings,
                        CartStatePV& scaled_target_state) {
  // Initialize result to commanded value. If we don't do any scaling, the
  // values will remain untouched.
  bool target_state_was_changed = false;
  scaled_target_state = target_state;

  switch (mode) {
    case TrajectoryGenerationMode::POSITION:
    case TrajectoryGenerationMode::POSITION_AND_VELOCITY: {
      // Clip the target state pose to enforce maximum reference distance and
      // rotation.
      eigenmath::Vector3d target_sensed_diff =
          target_state.pose.translation() - current_state.pose.translation();
      double max_reference_distance_ratio =
          target_sensed_diff.norm() / settings.max_translation;
      if (max_reference_distance_ratio > 1.0) {
        INTRINSIC_RT_LOG_THROTTLED(WARNING)
            << "Clipping reference pose translation!";
        // This computation ensures that correct sign of the correction.
        scaled_target_state.pose.translation() =
            current_state.pose.translation() +
            target_sensed_diff / max_reference_distance_ratio;
        target_state_was_changed = true;
      }

      eigenmath::Vector3d target_sensed_diff_rot =
          eigenmath::QuaternionToAngleTimesAxis(
              target_state.pose.quaternion() *
              current_state.pose.quaternion().conjugate());
      double max_reference_rotation_ratio =
          target_sensed_diff_rot.norm() / settings.max_rotation;
      if (max_reference_rotation_ratio > 1.0) {
        INTRINSIC_RT_LOG_THROTTLED(WARNING)
            << "Clipping reference pose orientation!";

        // Note: We calculated the
        // target_sensed_diff_rot = target_quat * [sensed_quat] ^ (-1). It means
        // that the new_target_quat should be calculated as
        // target_sensed_diff_rot * sensed_quat, as per quaternion
        // multiplication rules.
        scaled_target_state.pose.setQuaternion(
            eigenmath::Quaterniond(eigenmath::Quaterniond::AngleAxisType(
                settings.max_rotation, target_sensed_diff_rot.normalized())) *
            current_state.pose.quaternion());
        target_state_was_changed = true;
      }
      break;
    }
    case TrajectoryGenerationMode::VELOCITY: {
      // Scale down the incoming twist if it drives the reference pose too far
      // away from the sensed pose.
      //
      // This activates when the distance between reference pose and sensed pose
      // is in the band between:
      //
      // task_settings.maximum_reference_distance
      //   * task_settings.reference_limit_activation_ratio
      //
      // and task_settings.maximum_reference_distance

      double new_reference_sensed_distance =
          (next_reference_state.pose.translation() -
           current_state.pose.translation())
              .norm();
      double old_reference_sensed_distance =
          (current_reference_state.pose.translation() -
           current_state.pose.translation())
              .norm();
      // If the twist drives us further away from the reference pose, check
      // whether we should apply scaling.
      if (new_reference_sensed_distance > old_reference_sensed_distance) {
        // Scale the twist's translation component.
        double linear_scale_factor =
            GetScaleFactor(old_reference_sensed_distance,
                           settings.max_translation, settings.activation_ratio);
        if (linear_scale_factor < 1.0) {
          INTRINSIC_RT_LOG_THROTTLED(WARNING)
              << "Scaling reference twist, linear part!";
          scaled_target_state.velocity.head<3>() *= linear_scale_factor;
          target_state_was_changed = true;
        }
      }

      // Now do the same for rotation
      double old_tracking_error_rotation_angle =
          eigenmath::QuaternionToAngleTimesAxis(
              current_reference_state.pose.quaternion() *
              current_state.pose.quaternion().conjugate())
              .norm();
      double new_tracking_error_rotation_angle =
          eigenmath::QuaternionToAngleTimesAxis(
              next_reference_state.pose.quaternion() *
              current_state.pose.quaternion().conjugate())
              .norm();

      // If the twist drives us further away from the reference pose, check
      // whether we should apply scaling.
      if (new_tracking_error_rotation_angle >
          old_tracking_error_rotation_angle) {
        // scale the twist's rotation component.
        double rotational_scale_factor =
            GetScaleFactor(old_tracking_error_rotation_angle,
                           settings.max_rotation, settings.activation_ratio);
        if (rotational_scale_factor < 1.0) {
          INTRINSIC_RT_LOG_THROTTLED(WARNING)
              << "Scaling reference twist, rotational part!";
          scaled_target_state.velocity.tail<3>() *= rotational_scale_factor;
          target_state_was_changed = true;
        }
      }
      break;
    }
    case TrajectoryGenerationMode::TRAJECTORY_GENERATION_MODE_UNSPECIFIED:
      // Do nothing if no trajectory mode is selected.
      break;
    default:
      INTRINSIC_RT_LOG(ERROR) << "Unknown value for TrajectoryGenerationMode!";
      break;
  }

  return target_state_was_changed;
}

bool ScalePoseTwistToLimits(const CartesianLimits& limits,
                            eigenmath::Vector6d* pose_twist) {
  // Find scaling factor for translational velocity.
  double translational_scaling_factor = 1.0;
  bool scaled = false;
  for (int k = 0; k < 3; ++k) {
    double translational_scaling_factor_candidate = 1.0;
    if ((*pose_twist)(k) > limits.max_translational_velocity(k)) {
      translational_scaling_factor_candidate =
          limits.max_translational_velocity(k) / (*pose_twist)(k);
    } else if ((*pose_twist)(k) < limits.min_translational_velocity(k)) {
      translational_scaling_factor_candidate =
          limits.min_translational_velocity(k) / (*pose_twist)(k);
    }
    if (translational_scaling_factor_candidate < translational_scaling_factor) {
      translational_scaling_factor = translational_scaling_factor_candidate;
    }
  }

  if (translational_scaling_factor < 0.0) {
    INTRINSIC_RT_LOG(ERROR)
        << "Negative scaling factor found when scaling the translational "
           "part of the twist. The limit interval must be invalid (does not "
           "include zero?). Scaling twist to zero.";
    scaled = true;
    *pose_twist *= 0.0;

  } else if (translational_scaling_factor < 1.0) {
    INTRINSIC_RT_LOG_THROTTLED(WARNING)
        << "Scaling translational velocity by " << translational_scaling_factor;
    scaled = true;
    pose_twist->head(3) *= translational_scaling_factor;
  }

  // Find scaling factor for rotational velocity.
  if (pose_twist->tail(3).norm() > limits.max_rotational_velocity) {
    double rotational_scaling_factor =
        limits.max_rotational_velocity / pose_twist->tail(3).norm();
    INTRINSIC_RT_LOG_THROTTLED(WARNING)
        << "Scaling rotational velocity by " << rotational_scaling_factor;
    pose_twist->tail(3) *= rotational_scaling_factor;
    scaled = true;
  }

  return scaled;
}

}  // namespace intrinsic::icon
