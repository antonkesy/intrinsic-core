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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ANALYTIC_GEOMETRY_TYPES_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ANALYTIC_GEOMETRY_TYPES_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {
namespace analytical_geometry {

constexpr double kZeroThreshold = 1e-15;  // Any value smaller than this is
                                          // considered zero.

// A line defines a point p on the line as `p = origin + t * direction`, where t
// is a real number.
template <typename T>
class HyperLine {
 public:
  // Create a hyper-line with the given origin and direction vector. Return an
  // error if the direction vector's length is considered zero (smaller or equal
  // to `kZeroThreshold`).
  static absl::StatusOr<HyperLine> Create(const T& origin, const T& direction) {
    if (direction.norm() <= kZeroThreshold) {
      return absl::InvalidArgumentError(
          absl::StrCat("Direction must be greater than ", kZeroThreshold,
                       ", but got ", direction.norm(), " instead."));
    }

    return HyperLine(origin, direction);
  }

  HyperLine() = delete;

  T origin() const { return origin_; }
  T direction() const { return direction_; }

 private:
  explicit HyperLine(const T& origin, const T& direction)
      : origin_(origin), direction_(direction) {}
  T origin_;
  T direction_;
};

// Define a line segment by its two end points.
template <typename T>
class HyperLineSegment {
 public:
  // Create a line segment with the given two end points. Normally the created
  // line segment is non-degenerate, i.e. the length of the line is strictly
  // positive. However, if otherwise, the `disable_line_segment_length_check`
  // flag can be set to true to disable the length check, allowing the
  // generation of a degenerate line segment, i.e. a line segment with its
  // length close to zero, or in other words a line segment which is a point.
  static absl::StatusOr<HyperLineSegment> Create(
      const T& p1, const T& p2,
      bool disable_line_segment_length_check = false) {
    const double line_segment_length = (p2 - p1).norm();
    if (!disable_line_segment_length_check &&
        (line_segment_length <= kZeroThreshold)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Line segment length must be greater than ", kZeroThreshold,
          ", but got ", line_segment_length, " instead."));
    }

    return HyperLineSegment(p1, p2);
  }

  HyperLineSegment() = delete;

  T p1() const { return p1_; }
  T p2() const { return p2_; }

  double length() const { return (p2_ - p1_).norm(); }

 private:
  explicit HyperLineSegment(const T& p1, const T& p2) : p1_(p1), p2_(p2) {}

  T p1_;
  T p2_;
};

// Define a hyper-plane by its origin and normal vector.
template <typename T>
class HyperPlane {
 public:
  // Create a hyper-plane with the given origin and normal vector. Return an
  // error if the normal vector's length is considered zero (smaller or equal to
  // `kZeroThreshold`).
  static absl::StatusOr<HyperPlane> Create(const T& origin, const T& normal) {
    if (normal.norm() <= kZeroThreshold) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Normal vector's magnitude must be greater than ", kZeroThreshold,
          ", but got ", normal.norm(), " instead."));
    }

    return HyperPlane(origin, normal);
  }

  HyperPlane() = delete;

  T origin() const { return origin_; }
  T normal() const { return normal_; }

 private:
  explicit HyperPlane(const T& origin, const T& normal)
      : origin_(origin), normal_(normal) {}

  T origin_;
  T normal_;
};

// Define a hyper-sphere by its center and radius.
template <typename T>
class HyperSphere {
 public:
  // Create a hyper-sphere with the given center and radius. Return an error
  // if the radius is considered zero (smaller or equal to `kZeroThreshold`)
  // or negative.
  static absl::StatusOr<HyperSphere> Create(const T& center, double radius) {
    if (radius <= kZeroThreshold) {
      return absl::InvalidArgumentError(
          absl::StrCat("Radius must be greater than ", kZeroThreshold,
                       ", but got ", radius, " instead."));
    }

    return HyperSphere(center, radius);
  }

  HyperSphere() = delete;

  T center() const { return center_; }
  double radius() const { return radius_; }

 private:
  explicit HyperSphere(const T& center, double radius)
      : center_(center), radius_(radius) {}

  T center_;  // The center point of the sphere.
  double radius_;
};

using Point3d = eigenmath::Vector3d;
using Line3d = HyperLine<eigenmath::Vector3d>;
using LineSegment3d = HyperLineSegment<eigenmath::Vector3d>;
using Plane3d = HyperPlane<eigenmath::Vector3d>;

}  // namespace analytical_geometry
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ANALYTIC_GEOMETRY_TYPES_H_
