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

#ifndef INTRINSIC_MATH_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
#define INTRINSIC_MATH_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_

#include <stddef.h>

#include <algorithm>
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
#include "intrinsic/math/linspace.h"
#include "intrinsic/math/time_series_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

// Implements a "DiscretizedTrajectory", sampled in time-domain.  A
// DiscretizedTrajectory consists of a DataArray and a TimeArray. A TimeArray is
// a sequence of strictly monotonically increasing time stamps (the time elapsed
// since the beginning of the trajectory), and each time stamp is associated
// with sample from DataArray.
template <typename Data>
class DiscretizedTrajectory {
 public:
  DiscretizedTrajectory() = default;

  using TimeArray = std::vector<absl::Duration>;
  using DataArray = std::vector<Data>;

  // Creates a uniformly time-sampled DiscretizedTrajectory from 'data'
  // points, with sampling time 'delta_t'. Allows to std::move `data_array`.
  // Returns 'kFailedPrecondition' in case 'data' is empty, or 'delta_t' is
  // non-positive.
  static absl::StatusOr<DiscretizedTrajectory> Create(
      std::vector<Data> data_array, absl::Duration delta_t);

  // Creates a DiscretizedTrajectory for 'data_array' with corresponding
  // 'time_stamps'. Allows to "std::move" input parameters. Returns
  // 'kFailedPrecondition' in case 'data_array' or 'time_stamps' are empty, in
  // case of dimension mismatch, or if 'time_stamps' are not strictly
  // monotonically increasing. The first time stamp must be equal to zero.
  static absl::StatusOr<DiscretizedTrajectory> Create(
      std::vector<Data> data_array, std::vector<absl::Duration> time_stamps);

  // Returns time stamp stored at 'index' in the TimeArray. Returns
  // 'kOutOfRange' in case the index is less than zero or exceeds the trajectory
  // length.
  icon::RealtimeStatusOr<absl::Duration> TimeAt(int index) const;

  // Returns data stored at 'index' in the DataArray.  Returns 'kOutOfRange' in
  // case the index is less than zero or exceeds the trajectory length.
  icon::RealtimeStatusOr<Data> DataAt(int index) const;

  // Returns the index of the time-stamp preceding or directly coinciding with
  // 'time_since_trajectory_start'. Returns '0' if the searched time is smaller
  // than zero, returns the final index in case the searched time exceeds the
  // final time stamp.
  int GetLowerIndexForTime(absl::Duration time_since_trajectory_start) const;

  absl::Span<const absl::Duration> time_stamps() const { return time_stamps_; }

  absl::Span<const Data> data() const { return data_; }

  size_t size() const { return time_stamps_.size(); }

  // Returns the overall duration of the trajectory.
  absl::Duration Duration() const { return time_stamps_.back(); }

 private:
  DiscretizedTrajectory(std::vector<absl::Duration> time_stamps,
                        std::vector<Data> data)
      : time_stamps_(std::move(time_stamps)), data_(std::move(data)) {}

 protected:
  TimeArray time_stamps_;
  DataArray data_;
};

// Returns a downsampled discretized trajectory which contains every n-th
// element of the original `trajectory`, where n is the `downsampling_factor`.
// To maintain the target point of the trajectory, the final datapoint and
// timestamp are appended in any case, this means that the downsampling rate
// will be violated for the last interval. `downsampling_factor` must be >= 1,
// where 1 corresponds to a plain copy of the trajectory.
template <typename T>
absl::StatusOr<DiscretizedTrajectory<T>> Downsample(
    const DiscretizedTrajectory<T>& trajectory, int downsampling_factor);

// Returns a copied segment ranging from `from_index` to `to_index` of
// discretized `trajectory`. Returns `kOutOfRangeError` if the indices are out
// of range. The extracted segment will be time-shifted, i.e. it will start at
// t=0.
template <typename T>
absl::StatusOr<DiscretizedTrajectory<T>> ExtractSegment(
    const DiscretizedTrajectory<T>& trajectory, int from_index, int to_index);

template <typename Data>
absl::StatusOr<DiscretizedTrajectory<Data>> DiscretizedTrajectory<Data>::Create(
    std::vector<Data> data_array, absl::Duration delta_t) {
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

  return DiscretizedTrajectory(std::move(time_stamps), std::move(data_array));
}

template <typename Data>
absl::StatusOr<DiscretizedTrajectory<Data>> DiscretizedTrajectory<Data>::Create(
    std::vector<Data> data_array, std::vector<absl::Duration> time_stamps) {
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

  return DiscretizedTrajectory(std::move(time_stamps), std::move(data_array));
}

template <typename Data>
int DiscretizedTrajectory<Data>::GetLowerIndexForTime(
    absl::Duration time_since_trajectory_start) const {
  return GetLowerIndexForValue<absl::Duration>(
      absl::MakeConstSpan(time_stamps_), time_since_trajectory_start);
}

template <typename Data>
icon::RealtimeStatusOr<absl::Duration> DiscretizedTrajectory<Data>::TimeAt(
    int index) const {
  if (index < 0) {
    return icon::OutOfRangeError("index must be >= 0.");
  }
  if (index >= time_stamps_.size()) {
    return icon::OutOfRangeError(
        icon::FixedStrCat<icon::RealtimeStatus::kMaxMessageLength>(
            "Index exceeds trajectory length, which is ", time_stamps_.size()));
  }

  return time_stamps_[index];
}

template <typename Data>
icon::RealtimeStatusOr<Data> DiscretizedTrajectory<Data>::DataAt(
    int index) const {
  if (index < 0) {
    return icon::OutOfRangeError("index must be >= 0.");
  }
  if (index >= data_.size()) {
    return icon::OutOfRangeError(
        icon::FixedStrCat<icon::RealtimeStatus::kMaxMessageLength>(
            "Index exceeds trajectory length, which is ", time_stamps_.size()));
  }
  return Data(data_[index]);
}

template <typename T>
absl::StatusOr<DiscretizedTrajectory<T>> Downsample(
    const DiscretizedTrajectory<T>& trajectory, int downsampling_factor) {
  INTR_ASSIGN_OR_RETURN(std::vector<T> downsampled_data,
                        DownsampleVector(absl::MakeConstSpan(trajectory.data()),
                                         downsampling_factor));
  INTR_ASSIGN_OR_RETURN(
      std::vector<absl::Duration> downsampled_timestamps,
      DownsampleVector(absl::MakeConstSpan(trajectory.time_stamps()),
                       downsampling_factor));
  return DiscretizedTrajectory<T>::Create(std::move(downsampled_data),
                                          std::move(downsampled_timestamps));
}

template <typename T>
absl::StatusOr<DiscretizedTrajectory<T>> ExtractSegment(
    const DiscretizedTrajectory<T>& trajectory, int from_index, int to_index) {
  INTR_ASSIGN_OR_RETURN(
      std::vector<absl::Duration> segment_time_stamps,
      ExtractVectorSegment(absl::MakeConstSpan(trajectory.time_stamps()),
                           from_index, to_index));
  TimeShiftTimestampsVector(-trajectory.time_stamps().at(from_index),
                            absl::MakeSpan(segment_time_stamps));

  INTR_ASSIGN_OR_RETURN(
      std::vector<T> segment_data,
      ExtractVectorSegment(absl::MakeConstSpan(trajectory.data()), from_index,
                           to_index));

  return DiscretizedTrajectory<T>::Create(std::move(segment_data),
                                          std::move(segment_time_stamps));
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
