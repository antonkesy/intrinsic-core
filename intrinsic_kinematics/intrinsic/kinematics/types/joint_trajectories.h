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

#ifndef INTRINSIC_KINEMATICS_TYPES_JOINT_TRAJECTORIES_H_
#define INTRINSIC_KINEMATICS_TYPES_JOINT_TRAJECTORIES_H_

#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/dynamic_limits_check_mode.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/linspace.h"
#include "intrinsic/math/time_series_utils.h"
#include "intrinsic/math/trajectory/discretized_trajectory.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

// Interpolation types available to connect discrete samples of
// JointTrajectories during realtime execution (fine interpolation).
enum JointTrajectoryInterpolationType {
  kUnspecified = 0,
  kCubicPolynomial = 1,
  kQuinticPolynomial = 2,
};

// A discretized joint trajectory that includes a limits type to annotate if the
// trajectory may ignore ICON's internal joint acceleration limits.
template <typename JointStateType>
class JointTrajectory : public DiscretizedTrajectory<JointStateType> {
 public:
  JointTrajectory() = default;

  // Creates a uniformly time-sampled JointTrajectory from `data_array`
  // points, with sampling time `delta_t`. Defines the
  // `joint_dynamic_limits_check_mode` flag that indicates whether the
  // trajectory satisfies joint acceleration limits, or not. Allows to std::move
  // `data_array`. Returns `kFailedPrecondition` in case `data_array` is empty,
  // or `delta_t` is non-positive.
  static absl::StatusOr<JointTrajectory<JointStateType>> Create(
      std::vector<JointStateType> data_array, absl::Duration delta_t,
      DynamicLimitsCheckMode joint_dynamic_limits_check_mode,
      JointTrajectoryInterpolationType interpolation_type,
      std::optional<std::vector<double>> cartesian_arc_lengths = std::nullopt);

  // Creates a JointTrajectory for `data_array` with corresponding
  // `time_stamps`. Defines the `joint_dynamic_limits_check_mode` flag that
  // indicates whether the trajectory satisfies joint acceleration limits, joint
  // torque limits or none of them. Allows to "std::move" input parameters.
  // Returns `kFailedPrecondition` in case `data_array` or `time_stamps` are
  // empty, in case of dimension mismatch, or if `time_stamps` are not strictly
  // monotonically increasing. The first time stamp must be equal to zero.
  static absl::StatusOr<JointTrajectory<JointStateType>> Create(
      std::vector<JointStateType> data_array,
      std::vector<absl::Duration> time_stamps,
      DynamicLimitsCheckMode joint_dynamic_limits_check_mode,
      JointTrajectoryInterpolationType interpolation_type,
      std::optional<std::vector<double>> cartesian_arc_lengths = std::nullopt);

  const DynamicLimitsCheckMode& joint_dynamic_limits_check_mode() const {
    return joint_dynamic_limits_check_mode_;
  }

  const JointTrajectoryInterpolationType& interpolation_type() const {
    return interpolation_type_;
  }

  // Overwrite the configured interpolation type.
  void set_interpolation_type(
      const JointTrajectoryInterpolationType& interpolation_type) {
    interpolation_type_ = interpolation_type;
  }

  std::optional<absl::Span<const double>> cartesian_arc_lengths() const {
    return cartesian_arc_lengths_;
  };

  bool HasCartesianArcLength() const {
    return cartesian_arc_lengths_.has_value();
  }

  icon::RealtimeStatusOr<double> CartesianArcLengthAt(int index) const {
    if (!HasCartesianArcLength()) {
      return icon::FailedPreconditionError(
          "This trajectory does not have Cartesian arc lengths.");
    }
    if (index < 0) {
      return icon::OutOfRangeError("index must be >= 0.");
    }
    if (index >= cartesian_arc_lengths_->size()) {
      return icon::OutOfRangeError(
          icon::FixedStrCat<icon::RealtimeStatus::kMaxMessageLength>(
              "Index exceeds trajectory length, which is ",
              cartesian_arc_lengths_->size()));
    }

    return cartesian_arc_lengths_->at(index);
  }

 private:
  JointTrajectory(std::vector<absl::Duration> time_stamps,
                  std::vector<JointStateType> data,
                  DynamicLimitsCheckMode joint_dynamic_limits_check_mode,
                  JointTrajectoryInterpolationType interpolation_type,
                  std::optional<std::vector<double>> cartesian_arc_lengths);

  DynamicLimitsCheckMode joint_dynamic_limits_check_mode_ =
      DynamicLimitsCheckMode::kCheckJointAcceleration;
  JointTrajectoryInterpolationType interpolation_type_ =
      JointTrajectoryInterpolationType::kUnspecified;

  std::optional<std::vector<double>> cartesian_arc_lengths_;
};

namespace joint_trajectories_internal {

// Checks the validity of a Cartesian arc length vector.
// Returns an error if the size of the cartesian arc length does not match the
// size of the time stamps, if the first Cartesian arc length is not zero, or
// if the cartesian arc lengths are not monotonically increasing.
absl::Status CheckCartesianArcLength(
    const std::vector<absl::Duration>& time_stamps,
    const std::optional<std::vector<double>>& cartesian_arc_lengths);

}  // namespace joint_trajectories_internal

// Returns a downsampled trajectory which contains every n-th element of the
// original `trajectory`, where n is the `downsampling_factor`. To maintain the
// target point of the trajectory, the final datapoint and timestamp are
// appended in any case, this means that the downsampling rate will be
// violated for the last interval. `downsampling_factor` must be >= 1, where 1
// corresponds to a plain copy of the trajectory.
template <typename JointStateType>
absl::StatusOr<JointTrajectory<JointStateType>> Downsample(
    const JointTrajectory<JointStateType>& trajectory,
    int downsampling_factor) {
  INTR_ASSIGN_OR_RETURN(std::vector<JointStateType> downsampled_data,
                        DownsampleVector(absl::MakeConstSpan(trajectory.data()),
                                         downsampling_factor));
  INTR_ASSIGN_OR_RETURN(
      std::vector<absl::Duration> downsampled_timestamps,
      DownsampleVector(absl::MakeConstSpan(trajectory.time_stamps()),
                       downsampling_factor));

  std::optional<std::vector<double>> downsampled_cartesian_arc_lengths;
  if (trajectory.HasCartesianArcLength()) {
    INTR_ASSIGN_OR_RETURN(
        downsampled_cartesian_arc_lengths,
        DownsampleVector(
            absl::MakeConstSpan(trajectory.cartesian_arc_lengths().value()),
            downsampling_factor));
  }
  return JointTrajectory<JointStateType>::Create(
      std::move(downsampled_data), std::move(downsampled_timestamps),
      trajectory.joint_dynamic_limits_check_mode(),
      trajectory.interpolation_type(),
      std::move(downsampled_cartesian_arc_lengths));
}

// Returns a copied segment ranging from `from_index` to `to_index` of
// JointTrajectory `trajectory`. Returns `kOutOfRangeError` if the indices
// are out of range. The extracted segment will be time-shifted, i.e. it will
// start at t=0.
template <typename JointStateType>
absl::StatusOr<JointTrajectory<JointStateType>> ExtractSegment(
    const JointTrajectory<JointStateType>& trajectory, int from_index,
    int to_index) {
  INTR_ASSIGN_OR_RETURN(
      std::vector<absl::Duration> segment_time_stamps,
      ExtractVectorSegment(absl::MakeConstSpan(trajectory.time_stamps()),
                           from_index, to_index));
  TimeShiftTimestampsVector(-trajectory.time_stamps().at(from_index),
                            absl::MakeSpan(segment_time_stamps));

  INTR_ASSIGN_OR_RETURN(
      std::vector<JointStateType> segment_data,
      ExtractVectorSegment(absl::MakeConstSpan(trajectory.data()), from_index,
                           to_index));

  std::optional<std::vector<double>> segment_cartesian_arc_lengths;
  if (trajectory.HasCartesianArcLength()) {
    INTR_ASSIGN_OR_RETURN(
        segment_cartesian_arc_lengths,
        ExtractVectorSegment(
            absl::MakeConstSpan(trajectory.cartesian_arc_lengths().value()),
            from_index, to_index));
    ShiftDataVector(-trajectory.cartesian_arc_lengths().value().at(from_index),
                    absl::MakeSpan(segment_cartesian_arc_lengths.value()));
  }
  return JointTrajectory<JointStateType>::Create(
      std::move(segment_data), std::move(segment_time_stamps),
      trajectory.joint_dynamic_limits_check_mode(),
      trajectory.interpolation_type(),
      std::move(segment_cartesian_arc_lengths));
}

template <typename JointStateType>
absl::StatusOr<JointTrajectory<JointStateType>>
JointTrajectory<JointStateType>::Create(
    std::vector<JointStateType> data_array, absl::Duration delta_t,
    DynamicLimitsCheckMode joint_dynamic_limits_check_mode,
    JointTrajectoryInterpolationType interpolation_type,
    std::optional<std::vector<double>> cartesian_arc_lengths) {
  if (data_array.empty()) {
    return absl::FailedPreconditionError("data_array is empty.");
  }
  if (delta_t <= absl::ZeroDuration()) {
    return absl::FailedPreconditionError("delta_t must be positive.");
  }

  // Create linearly spaced timestamps, starting at t=0.
  INTR_ASSIGN_OR_RETURN(
      std::vector<absl::Duration> time_stamps,
      Linspace(absl::ZeroDuration(),
               absl::Duration(delta_t * (data_array.size() - 1)),
               data_array.size()));

  INTR_RETURN_IF_ERROR(joint_trajectories_internal::CheckCartesianArcLength(
      time_stamps, cartesian_arc_lengths));

  return JointTrajectory<JointStateType>(
      std::move(time_stamps), std::move(data_array),
      joint_dynamic_limits_check_mode, interpolation_type,
      std::move(cartesian_arc_lengths));
}

template <typename JointStateType>
absl::StatusOr<JointTrajectory<JointStateType>>
JointTrajectory<JointStateType>::Create(
    std::vector<JointStateType> data_array,
    std::vector<absl::Duration> time_stamps,
    DynamicLimitsCheckMode joint_dynamic_limits_check_mode,
    JointTrajectoryInterpolationType interpolation_type,
    std::optional<std::vector<double>> cartesian_arc_lengths) {
  if (data_array.empty()) {
    return absl::FailedPreconditionError("data_array is empty.");
  }

  if (time_stamps.size() != data_array.size()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Size of time_stamps, which is ", time_stamps.size(),
        " does not match size of data_array, which is ", data_array.size()));
  }

  if (time_stamps.front() != absl::ZeroDuration()) {
    return absl::FailedPreconditionError("First time stamp must be zero.");
  }

  INTR_RETURN_IF_ERROR(
      ElementsIncreaseStrictlyMonotonically(absl::MakeConstSpan(time_stamps)))
      << "Time stamps must be strictly monotonically increasing.";

  INTR_RETURN_IF_ERROR(joint_trajectories_internal::CheckCartesianArcLength(
      time_stamps, cartesian_arc_lengths));

  return JointTrajectory<JointStateType>(
      std::move(time_stamps), std::move(data_array),
      joint_dynamic_limits_check_mode, interpolation_type,
      std::move(cartesian_arc_lengths));
}

template <typename JointStateType>
JointTrajectory<JointStateType>::JointTrajectory(
    std::vector<absl::Duration> time_stamps, std::vector<JointStateType> data,
    DynamicLimitsCheckMode joint_dynamic_limits_check_mode,
    JointTrajectoryInterpolationType interpolation_type,
    std::optional<std::vector<double>> cartesian_arc_lengths)
    : joint_dynamic_limits_check_mode_(joint_dynamic_limits_check_mode),
      interpolation_type_(interpolation_type),
      cartesian_arc_lengths_(std::move(cartesian_arc_lengths)) {
  JointTrajectory<JointStateType>::time_stamps_ = std::move(time_stamps);
  JointTrajectory<JointStateType>::data_ = std::move(data);
}

using JointTrajectoryP = DiscretizedTrajectory<JointStateP>;
using JointTrajectoryPVA = JointTrajectory<JointStatePVA>;
using JointTrajectoryPVAJ = JointTrajectory<JointStatePVAJ>;
using JointTrajectoryPVAT = JointTrajectory<JointStatePVAT>;
using JointTrajectoryPVAJT = JointTrajectory<JointStatePVAJT>;

}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_TYPES_JOINT_TRAJECTORIES_H_
