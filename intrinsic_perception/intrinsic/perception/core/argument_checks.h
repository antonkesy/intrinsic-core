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

#ifndef INTRINSIC_PERCEPTION_CORE_ARGUMENT_CHECKS_H_
#define INTRINSIC_PERCEPTION_CORE_ARGUMENT_CHECKS_H_

#include <limits>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

namespace intrinsic {
namespace perception {

template <typename VectorType>
absl::Status HasUnitLength(const VectorType& v) {
  using T = decltype(v.norm());
  const T length = v.norm();
  if (std::abs(length - 1) >= std::sqrt(std::numeric_limits<T>::epsilon())) {
    return absl::OutOfRangeError(
        absl::StrFormat("Expected vector of length = 1 while the current "
                        "vector has length = %.*g.",
                        std::numeric_limits<T>::max_digits10, length));
  }
  return absl::OkStatus();
}

template <typename T>
absl::Status InClosedInterval(T value, T low, T high) {
  if (value >= low && value <= high) return absl::OkStatus();
  return absl::OutOfRangeError(absl::StrCat("The parameter 'value=", value,
                                            " must be in the closed interval [",
                                            low, ", ", high, "]."));
}

template <typename T>
absl::Status InLeftOpenInterval(T value, T low, T high) {
  if (value > low && value <= high) return absl::OkStatus();
  return absl::OutOfRangeError(absl::StrCat(
      "The parameter 'value=", value, " must be in the left-open interval (",
      low, ", ", high, "]."));
}

template <typename T>
absl::Status InRightOpenInterval(T value, T low, T high) {
  if (value >= low && value < high) return absl::OkStatus();
  return absl::OutOfRangeError(absl::StrCat(
      "The parameter 'value=", value, " must be in the right-open interval [",
      low, ", ", high, ")."));
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_ARGUMENT_CHECKS_H_
