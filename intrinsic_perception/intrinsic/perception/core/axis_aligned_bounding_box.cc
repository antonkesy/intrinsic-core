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

#include "intrinsic/perception/core/axis_aligned_bounding_box.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "intrinsic/perception/core/coordinate.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/intrinsic_params.h"
#include "intrinsic/perception/core/rectangle.h"

namespace intrinsic {
namespace perception {
namespace {

std::pair<Vector3f, Vector3f> ComputeTopLeftAndBottomRight(
    const std::vector<Vector3f>& points) {
  Vector3f top_left = Vector3f::Constant(std::numeric_limits<float>::max());
  Vector3f bottom_right =
      Vector3f::Constant(std::numeric_limits<float>::lowest());
  for (const Vector3f& pt : points) {
    for (size_t i = 0; i < 3; ++i) {
      top_left[i] = std::min(top_left[i], pt[i]);
      bottom_right[i] = std::max(bottom_right[i], pt[i]);
    }
  }
  return {top_left, bottom_right};
}

Vector3f ComputeCentroid(const Vector3f& min_dims, const Vector3f& max_dims) {
  return (max_dims - min_dims) / 2.0f + min_dims;
}

float ComputeSphereRadiusEnclosingPoints(const std::vector<Vector3f>& points,
                                         const Vector3f& centroid) {
  float sphere_radius_enclosing_points = 0.0f;
  for (const Vector3f& pt : points) {
    sphere_radius_enclosing_points =
        std::max(sphere_radius_enclosing_points, (pt - centroid).norm());
  }
  return sphere_radius_enclosing_points;
}

}  // namespace

AxisAlignedBoundingBox::AxisAlignedBoundingBox(
    const std::vector<Vector3f>& points)
    : box_data_{std::make_optional<AxisAlignedBoundingBox::BoxData>()} {
  if (points.empty()) return;
  auto [top_left, bottom_right] = ComputeTopLeftAndBottomRight(points);
  box_data_->min_dims = top_left;
  box_data_->max_dims = bottom_right;
  box_data_->bounding_corners = {
      Vector3f(top_left.x(), top_left.y(), top_left.z()),
      Vector3f(bottom_right.x(), top_left.y(), top_left.z()),
      Vector3f(bottom_right.x(), bottom_right.y(), top_left.z()),
      Vector3f(top_left.x(), bottom_right.y(), top_left.z()),
      Vector3f(top_left.x(), top_left.y(), bottom_right.z()),
      Vector3f(bottom_right.x(), top_left.y(), bottom_right.z()),
      Vector3f(bottom_right.x(), bottom_right.y(), bottom_right.z()),
      Vector3f(top_left.x(), bottom_right.y(), bottom_right.z())};
  box_data_->centroid =
      ComputeCentroid(box_data_->min_dims, box_data_->max_dims);
  box_data_->sphere_radius_enclosing_points =
      ComputeSphereRadiusEnclosingPoints(points, box_data_->centroid);
}

bool AxisAlignedBoundingBox::Empty() const { return box_data_ == std::nullopt; }

const Vector3f& AxisAlignedBoundingBox::MinCorner() const {
  return box_data_->min_dims;
}

const Vector3f& AxisAlignedBoundingBox::MaxCorner() const {
  return box_data_->max_dims;
}

Vector3f AxisAlignedBoundingBox::Size() const {
  return box_data_->max_dims - box_data_->min_dims;
}

const std::array<Vector3f, 8>& AxisAlignedBoundingBox::Corners() const {
  return box_data_->bounding_corners;
}

float AxisAlignedBoundingBox::SphereRadiusEnclosingPoints() const {
  return box_data_->sphere_radius_enclosing_points;
}

Vector3f AxisAlignedBoundingBox::Centroid() const {
  return box_data_->centroid;
}

Rectangle ProjectBoundingBox(const AxisAlignedBoundingBox& bbox,
                             const IntrinsicParams& intrinsics,
                             const Isometry3f& transformation) {
  constexpr float max_f = std::numeric_limits<float>::max();
  constexpr float min_f = std::numeric_limits<float>::lowest();

  Vector2f projected_min(max_f, max_f);
  Vector2f projected_max(min_f, min_f);

  for (const Vector3f& pt : bbox.Corners()) {
    const Vector2f projected = ProjectToImage(intrinsics, transformation * pt);
    projected_min[0] = std::min(projected_min[0], projected[0]);
    projected_max[0] = std::max(projected_max[0], projected[0]);
    projected_min[1] = std::min(projected_min[1], projected[1]);
    projected_max[1] = std::max(projected_max[1], projected[1]);
  }
  const int32_t minx = std::floor(projected_min[0]);
  const int32_t miny = std::floor(projected_min[1]);
  const int32_t maxx = std::ceil(projected_max[0]);
  const int32_t maxy = std::ceil(projected_max[1]);

  return Rectangle(Coordinate(minx, miny), Coordinate(maxx, maxy));
}

Rectangle ProjectAndClipBoundingBox(const AxisAlignedBoundingBox& bbox,
                                    const IntrinsicParams& intrinsics,
                                    const Isometry3f& transformation) {
  return Intersected(ProjectBoundingBox(bbox, intrinsics, transformation),
                     Rectangle(intrinsics.dimensions()));
}

bool Contains(const AxisAlignedBoundingBox& bbox, const Vector3f& v) {
  if (bbox.Empty()) return false;
  return v.x() >= bbox.MinCorner().x() && v.x() <= bbox.MaxCorner().x() &&
         v.y() >= bbox.MinCorner().y() && v.y() <= bbox.MaxCorner().y() &&
         v.z() >= bbox.MinCorner().z() && v.z() <= bbox.MaxCorner().z();
}

}  // namespace perception
}  // namespace intrinsic
