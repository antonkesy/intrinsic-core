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

#ifndef INTRINSIC_PERCEPTION_CORE_MESH_H_
#define INTRINSIC_PERCEPTION_CORE_MESH_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/core/axis_aligned_bounding_box.h"
#include "intrinsic/perception/core/coordinate.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/intrinsic_params.h"
#include "intrinsic/perception/core/rectangle.h"

namespace intrinsic {
namespace perception {

// The MeshType defines the type of the mesh. Currently, triangles and lines are
// supported.
enum MeshType { TRIANGLES = 0, LINES = 1 };

// Mesh struct that keeps mesh vertices, its colors and normals, texture
// coordinates if given vertex indices and vectors of faces.
class Mesh {
 public:
  Mesh() = default;

  // Constructor for meshes which are given in ply format.
  // The containers may not be empty.
  Mesh(const std::vector<Vector3f>& vertices,
       const std::vector<Vector3f>& normals,
       const std::vector<Vector3f>& colors,
       const std::vector<uint32_t>& indices, const MeshType& mesh_type);

  // Constructor for meshes which are given in obj format.
  // The containers may not be empty.
  Mesh(const std::vector<Vector3f>& vertices,
       const std::vector<Vector3f>& normals,
       const std::vector<Vector2f>& texels,
       const std::vector<uint32_t>& indices,
       const Image<Rgb8u>& diffuse_texture, const MeshType& mesh_type);

  // Some getter functions.
  const std::vector<Vector3f>& vertices() const { return vertices_; }
  const std::vector<Vector3f>& normals() const { return normals_; }
  const std::vector<Vector3f>& colors() const { return colors_; }
  const std::vector<Vector2f>& texels() const { return texels_; }
  const std::vector<uint32_t>& indices() const { return indices_; }
  const Image<Rgb8u>& diffuse_texture() const { return diffuse_texture_; }
  const MeshType& mesh_type() const { return mesh_type_; }

  // Convenience function to get back the axis aligned bounding box of the mesh.
  const AxisAlignedBoundingBox& AxisAlignedBoundingBox() const {
    return axis_aligned_bounding_box_;
  }

  // Returns true if either the vertices or the indices are empty.
  bool Empty() const;

  // Shift mesh to the axis aligned centroid such that the new axis aligned
  // centroid becomes the mesh origin and return the transformation that caused
  // this shift.
  absl::StatusOr<Isometry3f> ToAxisAlignedBoundingBoxCentroid();

  // Align mesh to major axis and return the transformation that does so.
  absl::StatusOr<Isometry3f> AlignToMajorAxes();

  // Scale all vertices of the mesh with scale_factor and return the
  // transformation that does so.
  absl::StatusOr<Affine3f> ScaleVertices(float scale_factor);

  // Transform a mesh with provided transform and return a reference to itsself.
  absl::Status Transform(const Affine3f& transform);

  // Transform a mesh with transformation and give it back.
  absl::StatusOr<Mesh> Transformed(const Affine3f& transform) const;

  // Computes the 2D bounding box of the Mesh projected to the image. Note
  // that the bounding box might not be absolute accurate in case the
  // vertices are not densely sampled across the object.
  Rectangle MinimumProjectedBoundingBox(const IntrinsicParams& intrinsics,
                                        const Isometry3f& T) const;

 private:
  // It computes that new mesh properties and returns a reference to itself.
  absl::Status UpdateMeshProperties();

  // The following variables define a mesh. Vertices and normals are
  // mandatory. Apart from that either colors or texcoords must be available.
  // If texcoords are available the corresponding texture map image "texture"
  // must be available, too.
  std::vector<Vector3f> vertices_;
  std::vector<Vector3f> normals_;
  std::vector<Vector3f> colors_;
  std::vector<Vector2f> texels_;

  // Needs to be a vector of unsigned - size_t can not be handled in opengl.
  std::vector<uint32_t> indices_;

  // Texture map of the mesh.
  Image<Rgb8u> diffuse_texture_;

  // Axis aligned bounding box of the mesh vertices.
  class AxisAlignedBoundingBox axis_aligned_bounding_box_;

  // Type of the mesh e.g. lines and triangles.
  MeshType mesh_type_;
};

// Makes coordinate axis for the input mesh. The mesh consists of lines and not
// triangles.
Mesh CenteredAxesMesh(const Mesh& mesh, float factor);

// Returns a mesh representing a orthogonal coordinate system. The axes have the
// user specified length and the coordinate system is centered at the specified
// origin.
Mesh CenteredAxesMesh(float axes_length,
                      const Vector3f& origin = Vector3f::Zero());

// Convenience function: makes a bounding box for the input mesh. The mesh
// consists of lines and not triangles.
Mesh BoundingBoxMesh(const Mesh& mesh,
                     const Vector3f& color = {0.0f, 1.0f, 0.0f});

// Create a box mesh of the given size centered at 0.
// 'num_segments_in_longest_side' defines the number of segments in the longest
// side of the box, the smaller sides are adapted accordingly so that a triangle
// has roughly the same size.
Mesh CreateBoxMesh(const Vector3f& size, int num_segments_in_longest_side = 1,
                   const Vector3f& color = {0.5f, 0.5f, 0.5f});

// Get the adapted projected bounding box given the camera parameter and
// a transformation. This function is much more computationally expensive
// than getting the 3D bounding box only and projecting it to the image by
// hand. This is because the projected bounding box is computed tightly.
// Note that the bounding box might not be absolute accurate in case the
// vertices are not densely sampled across the object.
template <typename Vector, typename Alloc>
Rectangle ProjectedBoundingBox(const std::vector<Vector, Alloc>& vertices,
                               const IntrinsicParams& intrinsics,
                               const Isometry3f& T) {
  using FloatTraits = std::numeric_limits<float>;
  Vector2f projected_min(FloatTraits::max(), FloatTraits::max());
  Vector2f projected_max(FloatTraits::lowest(), FloatTraits::lowest());

  const Matrix3f R = T.matrix().topLeftCorner<3, 3>();
  const Vector3f t = T.translation();

  for (const auto& v : vertices) {
    const Vector3f ov = R * v.template head<3>() + t;
    // Projected parts of the object must be in front of the camera.
    if (ov.z() > 0.f) {
      const Vector2f pv = ProjectToImage(intrinsics, ov);
      projected_min[0] = std::min(projected_min[0], pv[0]);
      projected_max[0] = std::max(projected_max[0], pv[0]);
      projected_min[1] = std::min(projected_min[1], pv[1]);
      projected_max[1] = std::max(projected_max[1], pv[1]);
    }
  }
  // Check for numeric limits to avoid overflow exception when converting floats
  // to ints in case the object is located behind the camera.
  if (projected_min[0] == FloatTraits::max() ||
      projected_min[1] == FloatTraits::max() ||
      projected_max[0] == FloatTraits::min() ||
      projected_max[1] == FloatTraits::min())
    return {};

  const int32_t tl_col = std::floor(projected_min[0]);
  const int32_t tl_row = std::floor(projected_min[1]);
  const int32_t br_col = std::ceil(projected_max[0]);
  const int32_t br_row = std::ceil(projected_max[1]);

  if (tl_col > br_col || tl_row > br_row) return {};

  const Rectangle projected_bbox(Coordinate{tl_col, tl_row},
                                 Coordinate{br_col, br_row});
  return Intersected(projected_bbox, Rectangle{intrinsics.dimensions()});
}

// Merges meshes into a single mesh by simple concatenation.
// Meshes must contain exactly the same properties (e.g., mixing triangle and
// line meshes is not possible). Note: Merging textured meshes is not yet
// supported.
absl::StatusOr<Mesh> MergeMeshes(const std::vector<Mesh>& meshes);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_MESH_H_
