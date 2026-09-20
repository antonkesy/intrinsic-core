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

#ifndef INTRINSIC_PERCEPTION_CORE_DISTORTION_PARAMS_H_
#define INTRINSIC_PERCEPTION_CORE_DISTORTION_PARAMS_H_

#include <ostream>

#include "absl/status/status.h"

namespace intrinsic {
namespace perception {

// Encapsulates lens distortion coefficients for a polynomial model as it is
// used by OpenCV.
//
// The parameters are ordered as follows, matching OpenCV's convention:
// (k1, k2, p1, p2, [k3, [k4, k5, k6, [s1, s2, s3, s4, [tx, ty]]]]).
//
// The coefficients represent:
// -   Radial distortion: k1, k2, k3, k4, k5, k6
// -   Tangential distortion: p1, p2
// -   Thin prism distortion: s1, s2, s3, s4
// -   Tilted sensor distortion: tx, ty
//
// The actual number of parameters used depends on the calibration model:
// -   **4 parameter model:** Uses k1, k2, p1, p2.
// -   **5 parameter model:** Adds k3 (often referred to as "plumb_bob").
// -   **8 parameter model:** Adds k4, k5, k6 (corresponds to
// CALIB_RATIONAL_MODEL).
// -   **12 parameter model:** Adds s1, s2, s3, s4 (corresponds to
// CALIB_THIN_PRISM_MODEL).
// -   **14 parameter model:** Adds tx, ty (corresponds to CALIB_TILTED_MODEL).
struct DistortionParams {
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  double k3 = 0.0;
  double k4 = 0.0;
  double k5 = 0.0;
  double k6 = 0.0;
  double s1 = 0.0;
  double s2 = 0.0;
  double s3 = 0.0;
  double s4 = 0.0;
  double tx = 0.0;
  double ty = 0.0;
};

// Returns OK status if all values of the given distortion parameters are equal
// up to the user specified epsilon.
absl::Status DistortionParamsNear(const DistortionParams& a,
                                  const DistortionParams& b, double eps);

// Prints distortion params to the specified output stream.
// The PrintTo method generates testing specific output and can be distinct from
// a generic streaming operator.
void PrintTo(const DistortionParams& distortion, std::ostream* os);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_DISTORTION_PARAMS_H_
