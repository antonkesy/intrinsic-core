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

#include "intrinsic/icon/control/algorithms/distance_tracker.h"

#include <cmath>
#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_guard.h"

namespace intrinsic::icon {

absl::StatusOr<std::unique_ptr<MaximumTranslationalDistanceTracker>>
MaximumTranslationalDistanceTracker::Create(double frequency_hz,
                                            double time_window_sec) {
  INTRINSIC_ASSERT_NON_REALTIME();
  if (frequency_hz <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The frequency should be larger than zero. Got ", frequency_hz, "."));
  }

  if (time_window_sec <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("The time window should be larger than zero. Got ",
                     time_window_sec, "."));
  }

  int num_elements =
      static_cast<int>(std::ceil(frequency_hz * time_window_sec));

  // Using absl::WrapUnique() to access private constructor.
  return absl::WrapUnique(
      new MaximumTranslationalDistanceTracker(num_elements));
}

MaximumTranslationalDistanceTracker::MaximumTranslationalDistanceTracker(
    int num_elements) {
  curr_idx_ = 0;
  translations_ = eigenmath::MatrixXd(num_elements, 3);
}

void MaximumTranslationalDistanceTracker::Add(
    const eigenmath::Vector3d& translation) {
  translations_.row(curr_idx_ % translations_.rows()) = translation;
  curr_idx_++;
}

void MaximumTranslationalDistanceTracker::Clear() { curr_idx_ = 0; }

int MaximumTranslationalDistanceTracker::GetBufferSize() const {
  return translations_.rows();
}

double MaximumTranslationalDistanceTracker::GetMaximumDistance() const {
  if (curr_idx_ < translations_.rows()) {
    return std::numeric_limits<double>::infinity();
  }
  eigenmath::Vector3d mins = translations_.colwise().minCoeff();
  eigenmath::Vector3d maxs = translations_.colwise().maxCoeff();
  eigenmath::Vector3d largest_diff = mins - maxs;
  return largest_diff.norm();
}

}  // namespace intrinsic::icon
