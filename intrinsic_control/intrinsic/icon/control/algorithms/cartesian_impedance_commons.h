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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_IMPEDANCE_COMMONS_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_IMPEDANCE_COMMONS_H_

#include <limits>
#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_admittance_action.pb.h"
#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {
namespace cartesian_impedance {

// Define default tolerances for numeric checks.
static constexpr double kNumericPrecision = 1e-8;

// Temporary default for nullspace stiffness.
static constexpr double kDefaultNullspaceStiffness = 10;

struct RealTimeCartesianTarget {
  // Offset between robot tip and tool frame.
  Pose3d robot_tip_t_robot_tool;
  // Task frame expressed w.r.t. the robot base frame.
  Pose3d robot_base_t_task;
  // Desired tool frame orientation w.r.t. the task frame.
  eigenmath::Quaterniond task_t_tool_reference_orientation;
  // Desired tool frame position w.r.t. the task frame.
  eigenmath::Vector3d task_t_tool_reference_position;
  // Desired endeffector acceleration expressed in the task frame.
  Acceleration tool_reference_acceleration;
  // Desired endeffector twist expressed in the task frame.
  Twist tool_reference_twist;
  // Desired endeffector wrench expressed in the task frame.
  Wrench tool_reference_wrench;
  // Inverse of the desired virtual cartesian inertia matrix, expressed in the
  // task frame. The inverse inertia matrix must be computed as a preprocessing
  // step before the parameters are moved to the RT thread.
  eigenmath::Matrix6d virtual_cartesian_inertia_inverse =
      eigenmath::Matrix6d::Zero();
  // Cartesian stiffness expressed w.r.t. the task frame.
  eigenmath::Matrix6d cartesian_stiffness = eigenmath::Matrix6d::Zero();
  // Cartesian damping expressed w.r.t. the task frame.
  eigenmath::Matrix6d cartesian_damping = eigenmath::Matrix6d::Zero();
  // Wrench selection matrix expressed w.r.t. the task frame.
  eigenmath::Matrix6d wrench_selection_matrix = eigenmath::Matrix6d::Zero();
};

struct RealTimeNullspaceTarget {
  JointStatePVA joint_state;
  double nullspace_stiffness;
  double nullspace_damping;
};

struct RealTimeConstraints {
  JointLimits joint_limits;
  CartesianLimits cart_limits;
  bool enforce_cartesian_and_joint_limits;
};

// The error state represents the control error and is used to compute control
// signals and for convergence checking.
struct CartesianErrorState {
  eigenmath::Vector6d pose =
      eigenmath::Vector6d::Constant(std::numeric_limits<double>::max());
  eigenmath::Vector6d twist =
      eigenmath::Vector6d::Constant(std::numeric_limits<double>::max());
  eigenmath::Vector6d wrench =
      eigenmath::Vector6d::Constant(std::numeric_limits<double>::max());
  eigenmath::Vector6d pose_integrated = eigenmath::Vector6d::Zero();

  // Sets pose, twist and wrench error to numeric_limits::max() and the
  // integrated pose error to zero.
  void Reset();
};

// Performs computationally expensive preprocessing steps, such as
// definiteness checking or automatic damping derivation on the input
// parameters. Expected to run at non-realtime.
// Optional fields in `params` can be left empty for later derivation at
// real-time.
absl::Status Preprocess(CartesianTarget& params);

// Performs preprocessing steps, such as automatic damping derivation on the
// input parameters. Expected to run at non-realtime. Optional fields in
// `params` can be left empty for later derivation at real-time.
absl::Status Preprocess(NullspaceTarget& params);

// Performs computationally expensive preprocessing steps.
absl::Status Preprocess(AlgorithmConfiguration& params);

// Performs computationally expensive preprocessing steps.
absl::Status Preprocess(StateVariableConfiguration& params);

// Transcribes the Cartesian target defined in non-real-time into its equivalent
// real-time type. Augments missing (optional) fields at real-time. Needs to be
// called at real-time for accuracy, do not move to non-realtime.
//
// Returns kFailedPrecondition in case the cartesian damping matrix was not set,
// since this operation cannot be performed at real-time.
// Returns kInternalError in case the forward kinematics fail.
RealtimeStatusOr<RealTimeCartesianTarget> ToRealTimeCartesianTarget(
    const Pose3d& base_t_tip, const CartesianTarget& nrt_target);

// Transcribes the nullspace target defined in non-real-time into its equivalent
// real-time type. Augments missing (optional) fields at real-time: fills in
// the nullspace joint position with the current joint position. Needs to be
// called at real-time for accuracy, do not move to non-realtime.
//
// Returns kFailedPrecondition in case the nullspace stiffness or damping have
// not been set.
RealtimeStatusOr<RealTimeNullspaceTarget> ToRealTimeNullspaceTarget(
    const NullspaceTarget& nrt_target, const JointStatePVA& joint_state);

// Transcribes the joint limits defined in non-real-time into its equivalent
// real-time type. Augments missing (optional) fields at real-time: fills in
// the joint limits with `system_limits` and cartesian limits with
// `default_cartesian_limits`.
//
// Returns kFailedPrecondition in case the user provided limits have wrong size
// or violate the part's maximum limits.
RealtimeStatusOr<RealTimeConstraints> ToRealTimeConstraints(
    const Constraints& nrt_constraints, const JointLimits& system_limits,
    const CartesianLimits& default_cartesian_limits, int njoints);

// Subtracts deadband from vector-valued measurement.
// Treats the deadband values as principal axes of an n-dimensional
// hyperellipsoid. In comparison to classical per-degree-of-freedom deadband
// subtraction, this formulation creates a "smooth" deadband surface in between
// the different main coordinate directions. When considering wrench
// measurements as living in the Euclidean space Rn, this formulation represents
// the deadband as a generalized n-dimensional hyperellipsoid in the space of
// all possible measurements, centered around zero and with principal axes
// defined by the deadband vector.  Returns error status in case of dimension
// mismatch.
template <typename Vector>
RealtimeStatus ApplyDeadband(const Vector& band, Vector& measured) {
  if (band.size() != measured.size()) {
    return icon::InvalidArgumentError(
        "Deadband and measurement have different dimensions.");
  }
  // Project deadband values in direction of the individual measurement
  // `meas`. This results in a Rn vector pointing from the origin to the point
  // on the deadband hyperellipsoid in which direction the incoming
  // measurement is located.
  Vector proj_band(band.asDiagonal() * measured.normalized());

  // If the measurement lies inside the deadband ellipsoid, set zero.
  if (measured.squaredNorm() < proj_band.squaredNorm()) {
    measured.setZero();
  } else {
    // otherwise substract deadband.
    measured -= proj_band;
  }

  return icon::OkStatus();
}

// Computes distance between two poses along a given vector.
double ComputeTranslationalDistanceTraveled(
    const Pose3d& start, const Pose3d& end,
    const std::optional<eigenmath::Vector3d>& along_normalized);

// Status helper struct for the streaming output conversion.
struct AdmittanceActionStreamingOutputStatus {
  double error_state_position[3];
  double error_state_orientation[3];
  double error_state_translational_velocity[3];
  double error_state_angular_velocity[3];
  double error_state_force[3];
  double error_state_torque[3];
  double previous_cartesian_translational_acceleration_command[3];
  double previous_cartesian_angular_acceleration_command[3];
  double sensed_force_at_tool_in_task_frame[3];
  double sensed_torque_at_tool_in_task_frame[3];
  double post_sensor_dynamics_force_in_task_frame[3];
  double post_sensor_dynamics_torque_in_task_frame[3];
  double settled_for_seconds;
  double sensed_force_magnitude;
  double sensed_torque_magnitude;
  double translational_distance_traveled;
  double maximum_translational_displacement_in_time_window;
  bool is_settled;
  double robot_tip_t_robot_tool[7];
  double robot_base_t_task[7];
  double task_t_tool_reference_pose[7];
  double tool_reference_twist[6];
  double tool_reference_wrench[6];
};

void CopyDoublesToVector3(absl::Span<const double> doubles,
                          intrinsic_proto::Vector3* vector);

// Copies the 6d twist from the input span to the output proto.
absl::Status CopyDoublesToTwist(absl::Span<const double> doubles,
                                intrinsic_proto::icon::Twist* twist);
// Copies the 7d pose from the input span to the output proto.
absl::Status CopyDoublesToPose(absl::Span<const double> doubles,
                               intrinsic_proto::Pose* pose);
// Copies the 6d wrench from the input span to the output proto.
absl::Status CopyDoublesToWrench(absl::Span<const double> doubles,
                                 intrinsic_proto::icon::Wrench* wrench);
// Copies the 7d pose from the input pose to the output span.
void CopyPoseToDoubles(const Pose3d& pose, double* doubles);

absl::StatusOr<intrinsic_proto::icon::actions::proto::CartesianAdmittanceStatus>
ToStreamingOutputStatusProto(
    const AdmittanceActionStreamingOutputStatus& status);

// Rotates a Cartesian (spatial 6d) vector expressed in frame 'b' into frame
// 'a'. Note that this is only pure rotation, and no spatial force/motion
// transform.
template <typename T>
::intrinsic::internal::CartesianVectorSubclass<T> RotateCartesianVector(
    const eigenmath::Quaterniond& a_R_b,
    const ::intrinsic::internal::CartesianVectorSubclass<T>& b_vec) {
  ::intrinsic::internal::CartesianVectorSubclass<T> a_vec;
  a_vec.template head<3>() = a_R_b.matrix() * b_vec.template head<3>();
  a_vec.template tail<3>() = a_R_b.matrix() * b_vec.template tail<3>();
  return a_vec;
}

// Rotates the CartesianErrorState expressed in frame 'b' into frame
// 'a'. Note that this is only pure rotation, and no spatial force/motion
// transform. The `pose_integrated` in the returned CartesianErrorState is
// unset.
CartesianErrorState RotateCartesianErrorState(
    const eigenmath::Quaterniond& a_r_b,
    const CartesianErrorState& error_state_in_b);

// Returns a 6x6 matrix, which is currently represented in frame `b`, in
// frame `a`. Can be used to expresses a stiffness, damping, etc matrix,
// which is currently represented in frame `b`, in frame `a`.
eigenmath::Matrix6d TransformTaskMatrix(const Pose3d& a_t_b,
                                        const eigenmath::Matrix6d& matrix_b);

}  // namespace cartesian_impedance
}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_IMPEDANCE_COMMONS_H_
