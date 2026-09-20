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

#include "intrinsic/perception/core/distortion_params.h"

#include <ostream>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "intrinsic/perception/core/math_utils.h"
#include "intrinsic/util/status/status_builder.h"

namespace intrinsic {
namespace perception {

absl::Status DistortionParamsNear(const DistortionParams& a,
                                  const DistortionParams& b, double eps) {
  if ((!IsApprox(a.k1, b.k1, eps) || !IsApprox(a.k2, b.k2, eps) ||
       !IsApprox(a.k3, b.k3, eps)) ||
      !IsApprox(a.k4, b.k4, eps) || !IsApprox(a.k5, b.k5, eps) ||
      !IsApprox(a.k6, b.k6, eps)) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Radial distortion coefficients are not near.";
  }
  if (!IsApprox(a.p1, b.p1, eps) || !IsApprox(a.p2, b.p2, eps)) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Tangential distortion coefficients are not near.";
  }
  if ((!IsApprox(a.s1, b.s1, eps) || !IsApprox(a.s2, b.s2, eps) ||
       !IsApprox(a.s3, b.s3, eps) || !IsApprox(a.s4, b.s4, eps))) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Thin prism distortion coefficients are not near.";
  }
  if (!IsApprox(a.tx, b.tx, eps) || !IsApprox(a.ty, b.ty, eps)) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Tilt distortion coefficients are not near.";
  }
  return absl::OkStatus();
}

void PrintTo(const DistortionParams& distortion, std::ostream* os) {
  CHECK_NE(os, nullptr);
  *os << absl::StrFormat("{ k1: %f, k2: %f, p1: %f, p2: %f", distortion.k1,
                         distortion.k2, distortion.p1, distortion.p2);
  if (distortion.k3 != 0.0) {
    *os << absl::StrFormat(", k3: %f", distortion.k3);
  }
  if (distortion.k4 != 0.0) {
    *os << absl::StrFormat(", k4: %f", distortion.k4);
  }
  if (distortion.k5 != 0.0) {
    *os << absl::StrFormat(", k5: %f", distortion.k5);
  }
  if (distortion.k6 != 0.0) {
    *os << absl::StrFormat(", k6: %f", distortion.k6);
  }
  if (distortion.s1 != 0.0) {
    *os << absl::StrFormat(", s1: %f", distortion.s1);
  }
  if (distortion.s2 != 0.0) {
    *os << absl::StrFormat(", s2: %f", distortion.s2);
  }
  if (distortion.s3 != 0.0) {
    *os << absl::StrFormat(", s3: %f", distortion.s3);
  }
  if (distortion.s4 != 0.0) {
    *os << absl::StrFormat(", s4: %f", distortion.s4);
  }
  if (distortion.tx != 0.0) {
    *os << absl::StrFormat(", tx: %f", distortion.tx);
  }
  if (distortion.ty != 0.0) {
    *os << absl::StrFormat(", ty: %f", distortion.ty);
  }
  *os << " }";
}

}  // namespace perception
}  // namespace intrinsic
