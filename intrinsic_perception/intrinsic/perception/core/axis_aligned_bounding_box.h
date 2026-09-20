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

#ifndef INTRINSIC_PERCEPTION_CORE_AXIS_ALIGNED_BOUNDING_BOX_H_
#define INTRINSIC_PERCEPTION_CORE_AXIS_ALIGNED_BOUNDING_BOX_H_

#include <array>
#include <optional>
#include <vector>

#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/intrinsic_params.h"
#include "intrinsic/perception/core/rectangle.h"

namespace intrinsic {
namespace perception {

// Bounding box for Mesh object.
class AxisAlignedBoundingBox {
 public:
  AxisAlignedBoundingBox() = default;

  // Returns the minimal bounding box which contains all passed points. If an
  // empty vector is passed, the returned bounding box is empty.
  explicit AxisAlignedBoundingBox(const std::vector<Vector3f>& points);

  // Returns true, if the bounding box is empty / uninitialized.
  bool Empty() const;

  // Returns the top-left coordinate of the bounding box.
  const Vector3f& MinCorner() const;

  // Returns the bottom right coordinate of the bounding box.
  const Vector3f& MaxCorner() const;

  // Returns the size of the bounding box.
  Vector3f Size() const;

  // Returns the centroid of the bounding box.
  Vector3f Centroid() const;

  // Returns the  radius of the sphere (centered at the axis aligned bounding
  // box centroid) enclosing all points that lead to its creation.
  float SphereRadiusEnclosingPoints() const;

  // Returns the 8 corner points of the bounding box.
  const std::array<Vector3f, 8>& Corners() const;

 private:
  struct BoxData {
    // Radius of the sphere (centered at the axis aligned bounding box centroid)
    // enclosing all points that lead to its creation.
    float sphere_radius_enclosing_points = 0.0f;
    // Maximum values in each dimension.
    Vector3f max_dims = Vector3f::Zero();
    // Minimum values in each dimension.
    Vector3f min_dims = Vector3f::Zero();
    // Centroid of the bounding corners.
    Vector3f centroid = Vector3f::Zero();
    // Eight bounding corners of the axis aligned bounding box.
    std::array<Vector3f, 8> bounding_corners;
  };

  std::optional<BoxData> box_data_ = std::nullopt;
};

// Returns true, of the given point is contained in the bounding box.
bool Contains(const AxisAlignedBoundingBox& bbox, const Vector3f& v);

// Returns the projected bounding box. The projection is performed with
// the given camera intrinsics and a isometry transformation.
Rectangle ProjectBoundingBox(const AxisAlignedBoundingBox& bbox,
                             const IntrinsicParams& intrinsics,
                             const Isometry3f& transformation);

// Returns the projected bounding box. The projection is performed with
// the given camera intrinsics and a isometry transformation.
Rectangle ProjectAndClipBoundingBox(const AxisAlignedBoundingBox& bbox,
                                    const IntrinsicParams& intrinsics,
                                    const Isometry3f& transformation);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_AXIS_ALIGNED_BOUNDING_BOX_H_
