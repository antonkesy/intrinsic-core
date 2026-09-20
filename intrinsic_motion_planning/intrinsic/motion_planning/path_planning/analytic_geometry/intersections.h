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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ANALYTIC_GEOMETRY_INTERSECTIONS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ANALYTIC_GEOMETRY_INTERSECTIONS_H_

#include <cmath>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/motion_planning/path_planning/analytic_geometry/types.h"

namespace intrinsic {
namespace analytical_geometry {

constexpr double kZeroLineSegmentLengthThreshold = 1e-7;

// Compute the shortest line segment between two lines. Normally the generated
// line segment is non-degenerate, i.e. the length of the line is strictly
// positive. However, if otherwise, the `disable_line_segment_length_check` flag
// can be set to true to disable the length check, allowing the generation of a
// degenerate line segment, i.e. a line segment with its length close to zero,
// or in other words a line segment which is a point.
absl::StatusOr<LineSegment3d> ShortestLineSegmentBtwTwoLines(
    const LineSegment3d& segment1, const LineSegment3d& segment2,
    bool disable_line_segment_length_check = false);

// Compute the intersection of a line and plane.
absl::StatusOr<Point3d> PlaneLineIntersection(const Plane3d& plane,
                                              const Line3d& line);

bool AreThreePointsCollinear(const Point3d& p1, const Point3d& p2,
                             const Point3d& p3, double tolerance = 1e-6);

// Default tolerance corresponds to an angular error of ~1e-6 radians.
bool AreThreeVectorXdCollinear(const eigenmath::VectorXd& p1,
                               const eigenmath::VectorXd& p2,
                               const eigenmath::VectorXd& p3,
                               double tolerance = 1e-12);

// Compute the intersection of a line segment and a sphere, following the
// description in https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
// . Some of the supported types are `eigenmath::Vector2d`,
// `eigenmath::Vector3d`, `eigenmath::VectorNd`, and `eigenmath::VectorXd`.
template <typename T>
absl::StatusOr<std::vector<T>> LineSegmentSphereIntersection(
    const HyperLineSegment<T>& line_segment, const HyperSphere<T>& sphere,
    double zero_line_segment_length_threshold =
        kZeroLineSegmentLengthThreshold) {
  if (zero_line_segment_length_threshold <= 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("The zero line segment length threshold must be strictly "
                     "positive, but got ",
                     zero_line_segment_length_threshold, " instead."));
  }
  const double line_segment_length = line_segment.length();
  if (line_segment_length <= zero_line_segment_length_threshold) {
    return absl::InvalidArgumentError(
        absl::StrCat("The line segment's length must be greater than ",
                     zero_line_segment_length_threshold, ", but got ",
                     line_segment_length, " instead."));
  }

  const T o =
      line_segment.p1();  // The `o` vector (line origin) in the description.
  const T u = (line_segment.p2() - line_segment.p1())
                  .normalized();  // The normalized `u` vector (line direction)
                                  // in the description.
  const double minimum_d_value = 0.0;
  const double maximum_d_value = line_segment_length;
  // The alternative parameterization of the `line_segment` is defined as a set
  // of points p = o + (d * u) with `minimum_d_value` <= d <= `maximum_d_value`.

  const T c =
      sphere.center();  // The `c` vector (sphere center) in the description.
  const double r =
      sphere.radius();  // The `r` scalar (sphere radius) in the description.

  const T o_minus_c = o - c;
  const double u_dot_o_minus_c = u.dot(o_minus_c);
  const double nabla = ::intrinsic::IPow(u_dot_o_minus_c, 2) -
                       (o_minus_c.dot(o_minus_c) - ::intrinsic::IPow(r, 2));

  std::vector<T> intersection_points;
  intersection_points.reserve(2);
  intersection_points = {};
  if (nabla == 0) {
    const double d = -u_dot_o_minus_c;
    // Check if `d` is within the range of the line segment, if so, add the
    // corresponding intersection point.
    if ((d >= minimum_d_value) && (d <= maximum_d_value)) {
      intersection_points.push_back(o + (d * u));
    }
  } else if (nabla > 0) {
    const double sqrt_nabla = std::sqrt(nabla);

    const double d0 = -u_dot_o_minus_c - sqrt_nabla;
    // Check if `d0` is within the range of the line segment, if so, add the
    // corresponding intersection point.
    if ((d0 >= minimum_d_value) && (d0 <= maximum_d_value)) {
      intersection_points.push_back(o + (d0 * u));
    }

    const double d1 = -u_dot_o_minus_c + sqrt_nabla;
    // Check if `d1` is within the range of the line segment, if so, add the
    // corresponding intersection point.
    if ((d1 >= minimum_d_value) && (d1 <= maximum_d_value)) {
      intersection_points.push_back(o + (d1 * u));
    }
  }
  return intersection_points;
}

}  // namespace analytical_geometry
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ANALYTIC_GEOMETRY_INTERSECTIONS_H_
