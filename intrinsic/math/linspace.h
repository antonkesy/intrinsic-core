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

#ifndef INTRINSIC_MATH_LINSPACE_H_
#define INTRINSIC_MATH_LINSPACE_H_

#include <type_traits>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace intrinsic {

// A linspace utility working exactly the same way as MATLAB linspace.
// Generates 'num_samples' linearly equally spaced points between 'start_value'
// and 'end_value'. Returns 'kFailedPrecondition' for an invalid sample number,
// i.e. less than 1 sample. If exactly one sample is requested, returns the
// 'start_value'. Due to floating point arithmetics, the last element in the
// returned vector is only approximately equal to the desired 'end_value'.
template <typename T>
inline absl::StatusOr<std::vector<T>> Linspace(const T& start_value,
                                               const T& end_value,
                                               int num_samples) {
  static_assert(!std::is_integral<T>::value,
                "Integral values not supported in linspace.");

  if (num_samples < 1) {
    return absl::FailedPreconditionError("num_samples must be >= 1");
  }

  if (num_samples == 1) {
    return std::vector<T>{start_value};
  }

  T step = (end_value - start_value) / (num_samples - 1);
  std::vector<T> linspaced_array(num_samples);

  for (int i = 0; i < num_samples; ++i) {
    linspaced_array[i] = start_value + i * step;
  }

  return linspaced_array;
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_LINSPACE_H_
