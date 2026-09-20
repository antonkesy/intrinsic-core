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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_DISTANCE_TRACKER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_DISTANCE_TRACKER_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic::icon {

// Computes the maximum distance between a set of stored points. Points
// are stored in a ring buffer and are intended to capture the position
// of a frame over a fixed time window.
class MaximumTranslationalDistanceTracker {
 public:
  // Creates a `MaximumTranslationalDistanceTracker` that will report
  // the largest translational distance between any points in the tracker's
  // buffer. Points are to be added at the provided `frequency_hz`.
  static absl::StatusOr<std::unique_ptr<MaximumTranslationalDistanceTracker>>
  Create(double frequency_hz, double time_window_sec);

  // Get the maximum distance between points in the buffer. The distance between
  // points is measured with the L2 norm. If the buffer is not full (i.e. `Add`
  // has not been called for a minimum of `GetBufferSize()` times since
  // initialization), the maximum distance returned is infinity.
  double GetMaximumDistance() const;

  // Add a point into the buffer. Expected to be called at the
  // frequency specified on `Create`.
  void Add(const eigenmath::Vector3d& translation);

  // Clear all points in the buffer.
  void Clear();

  // Return the size of the internally used ring buffer.
  int GetBufferSize() const;

 private:
  explicit MaximumTranslationalDistanceTracker(int num_elements);

  eigenmath::MatrixXd translations_;
  int curr_idx_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_DISTANCE_TRACKER_H_
