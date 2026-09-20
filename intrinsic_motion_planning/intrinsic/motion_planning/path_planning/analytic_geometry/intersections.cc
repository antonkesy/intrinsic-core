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

#include "intrinsic/motion_planning/path_planning/analytic_geometry/intersections.h"

#include <cmath>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/motion_planning/path_planning/analytic_geometry/types.h"

namespace intrinsic {
namespace analytical_geometry {

absl::StatusOr<LineSegment3d> ShortestLineSegmentBtwTwoLines(
    const LineSegment3d& segment1, const LineSegment3d& segment2,
    bool disable_line_segment_length_check) {
  // The implementation follow the description in
  // http://paulbourke.net/geometry/pointlineplane/

  eigenmath::Vector3d v1 = (segment1.p2() - segment1.p1()).normalized();
  eigenmath::Vector3d v2 = (segment2.p2() - segment2.p1()).normalized();

  if (std::fabs(std::fabs((v1).dot(v2)) - 1) < kZeroThreshold) {
    return absl::OutOfRangeError("Line are parallel.");
  }

  const eigenmath::Vector3d& p1 = segment1.p1();
  const eigenmath::Vector3d& p2 = segment1.p2();
  const eigenmath::Vector3d& p3 = segment2.p1();
  const eigenmath::Vector3d& p4 = segment2.p2();

  eigenmath::Vector3d p13;
  eigenmath::Vector3d p43;
  eigenmath::Vector3d p21;

  p13[0] = p1[0] - p3[0];
  p13[1] = p1[1] - p3[1];
  p13[2] = p1[2] - p3[2];
  p43[0] = p4[0] - p3[0];
  p43[1] = p4[1] - p3[1];
  p43[2] = p4[2] - p3[2];
  p21[0] = p2[0] - p1[0];
  p21[1] = p2[1] - p1[1];
  p21[2] = p2[2] - p1[2];

  double d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
  double d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
  double d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
  double d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
  double d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];

  double denum = d2121 * d4343 - d4321 * d4321;
  if (std::fabs(denum) < kZeroThreshold) {
    return absl::OutOfRangeError("Not solution to the lines intersection.");
  }

  double mua = (d1343 * d4321 - d1321 * d4343) / denum;
  double mub = (d1343 + mua * d4321) / d4343;

  return LineSegment3d::Create(
      /*p1=*/segment1.p1() + mua * (segment1.p2() - segment1.p1()),
      /*p2=*/segment2.p1() + mub * (segment2.p2() - segment2.p1()),
      disable_line_segment_length_check);
}

absl::StatusOr<Point3d> PlaneLineIntersection(const Plane3d& plane,
                                              const Line3d& line) {
  // The implementation follow the description in
  // http://paulbourke.net/geometry/pointlineplane/
  if (std::fabs(1.0 - line.direction().norm()) > kZeroThreshold) {
    return absl::InvalidArgumentError(
        "Line's direction vector is not normalized.");
  }
  if (std::fabs(1.0 - plane.normal().norm()) > kZeroThreshold) {
    return absl::InvalidArgumentError(
        "Plane's normal vector is not normalized.");
  }
  double denominator = plane.normal().dot(line.direction());
  if (std::fabs(denominator) < kZeroThreshold) {
    return absl::OutOfRangeError("Line is perpendicular to plane normal.");
  }
  double numerator = plane.normal().dot(plane.origin() - line.origin());

  double u = numerator / denominator;

  return line.origin() + u * line.direction();
}

bool AreThreePointsCollinear(const Point3d& p1, const Point3d& p2,
                             const Point3d& p3, double tolerance) {
  eigenmath::Vector3d n1 = p2 - p1;
  eigenmath::Vector3d n2 = p3 - p1;

  return n1.cross(n2).norm() < tolerance;
}

bool AreThreeVectorXdCollinear(const eigenmath::VectorXd& p1,
                               const eigenmath::VectorXd& p2,
                               const eigenmath::VectorXd& p3,
                               double tolerance) {
  eigenmath::VectorXd u = p2 - p1;
  u.normalize();
  eigenmath::VectorXd v = p3 - p1;
  v.normalize();
  double cos_angle_uv = u.dot(v);
  double error = 1.0 - cos_angle_uv;
  return error < tolerance;
}

}  // namespace analytical_geometry
}  // namespace intrinsic
