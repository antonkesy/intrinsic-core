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


#include "intrinsic/math/spline/bspline_parameter_transform_function_integrand.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/math/ipow.h"
namespace intrinsic {

namespace internal {

absl::StatusOr<double> RegularizedRatio(
    const double numerator, const double denominator,
    const double min_denominator_for_regularization,
    const double max_regularization_factor) {
  if (numerator < 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("Numerator must be non-negative. Got ", numerator, "."));
  }
  if (denominator < 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Denominator must be non-negative. Got ", denominator, "."));
  }
  if (min_denominator_for_regularization <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Min denominator for regularization must be positive. Got ",
        min_denominator_for_regularization, "."));
  }
  if (max_regularization_factor <= 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("Regularization factor must be positive. Got ",
                     max_regularization_factor, "."));
  }

  const double denominator_ratio =
      denominator / min_denominator_for_regularization;
  double regularization_factor_squared =
      (denominator < min_denominator_for_regularization)
          ? (1 - ::intrinsic::IPow(denominator_ratio, 2)) *
                ::intrinsic::IPow(max_regularization_factor, 2)
          : 0.0;

  return numerator * denominator /
         (::intrinsic::IPow(denominator, 2) + regularization_factor_squared);
}

}  // namespace internal

}  // namespace intrinsic
