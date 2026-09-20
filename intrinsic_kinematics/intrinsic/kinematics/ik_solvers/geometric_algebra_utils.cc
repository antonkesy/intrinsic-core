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

#include "intrinsic/kinematics/ik_solvers/geometric_algebra_utils.h"

#include <cmath>

#include "intrinsic/eigenmath/types.h"
#include "vsr/detail/vsr_algebra.h"
#include "vsr/detail/vsr_multivector.h"
#include "vsr/detail/vsr_xlists.h"
#include "vsr/space/vsr_cga3D_op.h"
#include "vsr/space/vsr_cga3D_round.h"
#include "vsr/space/vsr_cga3D_types.h"

using vsr::cga::Circle;
using vsr::cga::DualPlane;
using vsr::cga::DualSphere;
using vsr::cga::Line;
using vsr::cga::Pair;
using vsr::cga::Plane;
using vsr::cga::Pnt;
using vsr::cga::Sca;
using vsr::cga::Vec;

namespace intrinsic {
namespace kinematics {

using eigenmath::Vector3d;

namespace {

// Useful unit vectors.
const auto no = Vec(0, 0, 0).null();
const auto ni = vsr::cga::Inf(1);

}  // namespace

Pnt PointFromVector3d(const Vector3d& vec) {
  return vsr::cga::Construct::point(vsr::cga::Vec(vec[0], vec[1], vec[2]));
}

Vector3d DirectionVectorToVector3d(const vsr::cga::DirectionVector& dir_vec) {
  return Vector3d({dir_vec[0], dir_vec[1], dir_vec[2]});
}

vsr::cga::Translator LineMagnitudeToTranslator(double magnitude,
                                               const Line& direction_line) {
  // We want to create a translator as T = 1 + 0.5*ni*t
  vsr::cga::DirectionVector v =
      vsr::cga::Flat::direction(direction_line.unit());
  vsr::cga::Translator trs = vsr::cga::Gen::translator(Sca(magnitude) * v);
  return trs;
}

vsr::Multivector<vsr::algebra<vsr::metric<4, 1, true>, double>,
                 vsr::Basis<0, 3, 5, 6, 17, 18, 20>>
LineAngleToRotor(double angle, const Line& axis_line) {
  // We want to create a rotor as R = cos(angle/2) - rotation_axis*sin(angle/2)
  // rotation_axis being a dual line.
  //
  // There seems to be a problem adding a scalar to form the rotor and this
  // little detour creates the rotor we are looking for.

  // Assign the scalar component of the rotor
  vsr::cga::Rotor R = vsr::cga::Rot(cos(angle / 2), 0, 0, 0);

  // Get the other components
  auto axis = -(axis_line.unit()).dual();
  axis = Sca(sin(angle / 2)) * axis;

  return R + axis;
}

double PointDistanceToPlane(const Pnt& point, const Plane& plane) {
  return (point <= -plane.dual())[0];
}

// Return the squared distance to point. Can be negative.
double PointSquaredDistanceToLine(const Pnt& point, const Line& line) {
  // Get the closest point on the line to point
  auto p = (point <= line) / line;
  return (p <= point)[0];
}

DualPlane DualPlaneFromLine(const Line& line) { return ni <= (no <= line); }

Line LinePerpendicularToLines(const Line& line_a, const Line& line_b) {
  DualPlane a = ni <= (no <= line_a);
  DualPlane b = ni <= (no <= line_b);

  auto cross_a_b = -(no <= (ni <= -(a ^ b).dual()));

  return cross_a_b ^ ni ^ no;
}

double AngleBetweenLines(const Line& line_a, const Line& line_b,
                         const Line& axis) {
  // Get direction vector of the line as dual plane
  DualPlane a = DualPlaneFromLine(line_a);
  DualPlane b = DualPlaneFromLine(line_b);

  a = a.unit();
  b = b.unit();

  auto N = b ^ a;
  N = N.unit();

  // The sign of N should be selected so that it is aligned with the direction
  // of the axis.
  auto axis_dir = DualPlaneFromLine(axis);
  axis_dir = axis_dir.unit();
  auto Nsign = no <= (ni <= (axis_dir <= -N.dual()));

  auto c = a <= b;
  auto s = (a ^ b) * N;

  return atan2(s[0] * Nsign[0], c[0]);
}

}  // namespace kinematics
}  // namespace intrinsic
