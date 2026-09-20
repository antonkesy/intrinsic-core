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

#ifndef INTRINSIC_KINEMATICS_IK_SOLVERS_GEOMETRIC_ALGEBRA_UTILS_H_
#define INTRINSIC_KINEMATICS_IK_SOLVERS_GEOMETRIC_ALGEBRA_UTILS_H_

#include <iostream>
#include <string>

#include "absl/log/log.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "vsr/detail/vsr_algebra.h"
#include "vsr/detail/vsr_multivector.h"
#include "vsr/detail/vsr_xlists.h"
#include "vsr/space/vsr_cga3D_op.h"
#include "vsr/space/vsr_cga3D_round.h"
#include "vsr/space/vsr_cga3D_types.h"

namespace intrinsic {
namespace kinematics {

#define PrintPoint(label, point)                                       \
  LOG(INFO) << label << " = [" << point[0] << ", " << point[1] << ", " \
            << point[2] << "];";

vsr::cga::Pnt PointFromVector3d(const eigenmath::Vector3d& vec);

eigenmath::Vector3d DirectionVectorToVector3d(
    const vsr::cga::DirectionVector& dir_vec);

vsr::cga::Translator LineMagnitudeToTranslator(
    double magnitude, const vsr::cga::Line& direction_line);

vsr::Multivector<vsr::algebra<vsr::metric<4, 1, true>, double>,
                 vsr::Basis<0, 3, 5, 6, 17, 18, 20>>
LineAngleToRotor(double angle, const vsr::cga::Line& axis_line);

double PointDistanceToPlane(const vsr::cga::Pnt& point,
                            const vsr::cga::Plane& plane);

double PointSquaredDistanceToLine(const vsr::cga::Pnt& point,
                                  const vsr::cga::Line& line);

vsr::cga::DualPlane DualPlaneFromLine(const vsr::cga::Line& line);

vsr::cga::Line LinePerpendicularToLines(const vsr::cga::Line& line_a,
                                        const vsr::cga::Line& line_b);

double AngleBetweenLines(const vsr::cga::Line& line_a,
                         const vsr::cga::Line& line_b,
                         const vsr::cga::Line& axis);

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_SOLVERS_GEOMETRIC_ALGEBRA_UTILS_H_
