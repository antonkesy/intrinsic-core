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

#include "intrinsic/perception/core/mesh.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/core/axis_aligned_bounding_box.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/intrinsic_params.h"
#include "intrinsic/perception/core/rectangle.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {

Mesh::Mesh(const std::vector<Vector3f>& vertices,
           const std::vector<Vector3f>& normals,
           const std::vector<Vector3f>& colors,
           const std::vector<uint32_t>& indices, const MeshType& mesh_type)
    : vertices_(vertices),
      normals_(normals),
      colors_(colors),
      indices_(indices),
      axis_aligned_bounding_box_(vertices),
      mesh_type_(mesh_type) {
  CHECK(vertices_.size() == normals_.size() &&
        vertices_.size() == colors_.size())
      << "Size of vertices, normals and/or colors are not equal";
  CHECK(!Empty()) << "Mesh is not supposed to be empty!";
  CHECK((indices_.size() % 2 == 0 && mesh_type_ == MeshType::LINES) ||
        (indices_.size() % 3 == 0 && mesh_type_ == MeshType::TRIANGLES))
      << "Indices not in proper triangle or line format.";
}

Mesh::Mesh(const std::vector<Vector3f>& vertices,
           const std::vector<Vector3f>& normals,
           const std::vector<Vector2f>& texels,
           const std::vector<uint32_t>& indices,
           const Image<Rgb8u>& diffuse_texture, const MeshType& mesh_type)
    : vertices_(vertices),
      normals_(normals),
      texels_(texels),
      indices_(indices),
      diffuse_texture_(diffuse_texture),
      axis_aligned_bounding_box_(vertices),
      mesh_type_(mesh_type) {
  CHECK(vertices_.size() == normals_.size() &&
        vertices_.size() == texels_.size())
      << "Size of vertices, normals and/or colors are not equal";
  CHECK(!Empty()) << "Mesh is not supposed to be empty!";
  CHECK((indices_.size() % 2 == 0 && mesh_type_ == MeshType::LINES) ||
        (indices_.size() % 3 == 0 && mesh_type_ == MeshType::TRIANGLES))
      << "Indices not in proper triangle or line format.";
  CHECK(diffuse_texture_.area() > 0) << "Diffuse texture map is empty";
}

bool Mesh::Empty() const { return vertices_.empty() || indices_.empty(); }

absl::StatusOr<Isometry3f> Mesh::ToAxisAlignedBoundingBoxCentroid() {
  Isometry3f transform = Isometry3f::Identity();
  transform.translation() -= AxisAlignedBoundingBox().Centroid();
  for (Vector3f& v : vertices_) {
    v -= AxisAlignedBoundingBox().Centroid();
  }
  axis_aligned_bounding_box_ =
      ::intrinsic::perception::AxisAlignedBoundingBox(vertices_);
  return transform;
}

absl::StatusOr<Affine3f> Mesh::ScaleVertices(float s) {
  Affine3f transform = Affine3f::Identity();
  transform.linear() *= s;
  for (Vector3f& v : vertices_) {
    v *= s;
  }
  axis_aligned_bounding_box_ =
      ::intrinsic::perception::AxisAlignedBoundingBox(vertices_);
  return transform;
}

absl::StatusOr<Isometry3f> Mesh::AlignToMajorAxes() {
  for (Vector3f& p : vertices_) {
    p -= AxisAlignedBoundingBox().Centroid();
  }
  Isometry3f T_c = Isometry3f::Identity();
  T_c.translation() = -AxisAlignedBoundingBox().Centroid();

  Matrix3f covariance = Matrix3f::Zero();
  for (const Vector3f& p : vertices_) {
    covariance += p * p.transpose();
  }
  JacobiSVD<Matrix3f> svd(covariance, EigenComputeFullU);

  Isometry3f T = Isometry3f::Identity();
  T.linear() = svd.matrixU();

  if (T.linear().determinant() < 0) {
    // For square matrices det(AB) = det(A) * det(B).
    T.linear() *= -1;
  }
  T.linear() = T.inverse().linear();
  INTR_RETURN_IF_ERROR(Transform(T));
  return T * T_c;
}

absl::Status Mesh::Transform(const Affine3f& T) {
  for (Vector3f& v : vertices_) {
    v = T * v;
  }
  Matrix3f R = T.rotation();
  for (Vector3f& n : normals_) {
    n = R * n;
  }
  axis_aligned_bounding_box_ =
      ::intrinsic::perception::AxisAlignedBoundingBox(vertices_);
  return absl::OkStatus();
}

absl::StatusOr<Mesh> Mesh::Transformed(const Affine3f& T) const {
  Mesh out(*this);
  INTR_RETURN_IF_ERROR(out.Transform(T));
  return out;
}

Rectangle Mesh::MinimumProjectedBoundingBox(const IntrinsicParams& intrinsics,
                                            const Isometry3f& T) const {
  return ProjectedBoundingBox(vertices_, intrinsics, T);
}

// Make coordinate axis for the input mesh.
Mesh CenteredAxesMesh(const Mesh& mesh, const float factor) {
  const float length =
      mesh.AxisAlignedBoundingBox().SphereRadiusEnclosingPoints() * factor;
  return CenteredAxesMesh(length, mesh.AxisAlignedBoundingBox().Centroid());
}

Mesh CenteredAxesMesh(float axes_length, const Vector3f& origin) {
  std::vector<Vector3f> vertices;
  std::vector<Vector3f> normals;
  std::vector<Vector3f> colors;
  std::vector<uint32_t> indices;

  vertices.reserve(6);
  normals.reserve(6);
  colors.reserve(6);

  vertices.push_back(origin);
  vertices.emplace_back(origin + Vector3f::UnitX() * axes_length);
  vertices.push_back(origin);
  vertices.emplace_back(origin + Vector3f::UnitY() * axes_length);
  vertices.push_back(origin);
  vertices.emplace_back(origin + Vector3f::UnitZ() * axes_length);
  colors.emplace_back(Vector3f(1, 0, 0));
  colors.emplace_back(Vector3f(1, 0, 0));
  colors.emplace_back(Vector3f(0, 1, 0));
  colors.emplace_back(Vector3f(0, 1, 0));
  colors.emplace_back(Vector3f(0, 0, 1));
  colors.emplace_back(Vector3f(0, 0, 1));
  normals.emplace_back(Vector3f(1, 0, 0));
  normals.emplace_back(Vector3f(1, 0, 0));
  normals.emplace_back(Vector3f(0, 1, 0));
  normals.emplace_back(Vector3f(0, 1, 0));
  normals.emplace_back(Vector3f(0, 0, 1));
  normals.emplace_back(Vector3f(0, 0, 1));

  indices = {0, 1, 2, 3, 4, 5};

  return Mesh(vertices, normals, colors, indices, MeshType::LINES);
}

Mesh CreateBoxMesh(const Vector3f& size, int num_segments_in_longest_side,
                   const Vector3f& color) {
  const float quad_size = size.maxCoeff() / num_segments_in_longest_side;
  // Compute number of vertices in each dimension. Make sure that we have at
  // least two vertices.
  const Vector3i num_vertices =
      (size / quad_size + Vector3f::Ones()).cwiseMax(2).cast<int>();

  std::vector<Vector3f> vertices;
  std::vector<Vector3f> normals;
  std::vector<Vector3f> colors;
  std::vector<uint32_t> indices;

  const auto add_plane = [&](int dim0, int dim1, float sgn) {
    Vector3f n = Vector3f::Ones() * sgn;
    n(dim0) = 0;
    n(dim1) = 0;
    const int vertex_index0 = vertices.size();
    for (int y = 0; y < num_vertices(dim1); ++y) {
      for (int x = 0; x < num_vertices(dim0); ++x) {
        Vector3f v = Vector3f::Zero();
        v(dim0) = x / (-1.0f + num_vertices(dim0)) - 0.5f;
        v(dim1) = y / (-1.0f + num_vertices(dim1)) - 0.5f;
        v += n * 0.5f;
        v = v.cwiseProduct(size.cast<float>());
        vertices.push_back(v);
        normals.push_back(n);
        colors.push_back(color);

        if (x == 0 || y == 0) {
          continue;
        }

        const int index0 = vertex_index0 + x - 1 + (y - 1) * num_vertices(dim0);
        indices.push_back(index0);
        indices.push_back(index0 + 1);
        indices.push_back(index0 + num_vertices(dim0));
        // Flip vertex order to match normal direction.
        if (sgn < 0) {
          std::swap(indices.back(), indices[indices.size() - 2]);
        }

        indices.push_back(index0 + num_vertices(dim0));
        indices.push_back(index0 + 1);
        indices.push_back(index0 + num_vertices(dim0) + 1);
        if (sgn < 0) {
          std::swap(indices.back(), indices[indices.size() - 2]);
        }
      }
    }
  };
  add_plane(0, 1, 1.0f);   // x-y plane, z up.
  add_plane(0, 1, -1.0f);  // x-y plane, z down.
  add_plane(2, 0, 1.0f);   // z-x plane, y up.
  add_plane(2, 0, -1.0f);  // z-x plane, y down.
  add_plane(1, 2, 1.0f);   // y-z plane, x up.
  add_plane(1, 2, -1.0f);  // y-z plane, x down.

  return Mesh{vertices, normals, colors, indices, MeshType::TRIANGLES};
}

// Make a bounding box for the input mesh.
Mesh BoundingBoxMesh(const Mesh& mesh, const Vector3f& color) {
  std::vector<Vector3f> vertices;
  std::vector<Vector3f> normals;
  std::vector<Vector3f> colors;
  std::vector<uint32_t> indices = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6,
                                   6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7};
  vertices.reserve(indices.size());
  colors.reserve(indices.size());
  normals.reserve(indices.size());
  indices.reserve(indices.size());

  for (const auto& i : indices) {
    vertices.push_back(mesh.AxisAlignedBoundingBox().Corners()[i]);
  }
  for (size_t i = 0; i < vertices.size(); ++i) {
    colors.push_back(color);
    normals.push_back(vertices[i]);
    indices.push_back(i);
  }
  return Mesh(vertices, normals, colors, indices, MeshType::LINES);
}

absl::StatusOr<Mesh> MergeMeshes(const std::vector<Mesh>& meshes) {
  INTR_RET_CHECK(!meshes.empty()).SetCode(absl::StatusCode::kInvalidArgument)
      << "No meshes provided";
  INTR_RET_CHECK(meshes.front().texels().empty())
          .SetCode(absl::StatusCode::kInvalidArgument)
      << "Can't merge meshes with textures";
  // Count vertices and indices and verify consistency between meshes.
  int64_t num_vertices = meshes.front().vertices().size();
  int64_t num_indices = meshes.front().indices().size();
  for (int i = 1; i < meshes.size(); ++i) {
    const Mesh& mesh = meshes[i];
    if (mesh.vertices().empty()) {
      return absl::InvalidArgumentError("Mesh without vertices provided");
    }
    if (mesh.colors().empty() != meshes.front().colors().empty()) {
      return absl::InvalidArgumentError(
          "Inconsistent mesh colors during MergeMeshes. All meshes must have "
          "the same properties set.");
    }
    if (mesh.normals().empty() != meshes.front().normals().empty()) {
      return absl::InvalidArgumentError(
          "Inconsistent mesh normals during MergeMeshes. All meshes must have "
          "the same properties set.");
    }
    if (mesh.texels().empty() != meshes.front().texels().empty()) {
      return absl::InvalidArgumentError(
          "Inconsistent mesh texels during MergeMeshes. All meshes must have "
          "the same properties set.");
    }
    if (mesh.mesh_type() != meshes.front().mesh_type()) {
      return absl::InvalidArgumentError(
          "Inconsistent mesh type during MergeMeshes. All meshes must have the "
          "same properties set.");
    }
    num_vertices += mesh.vertices().size();
    num_indices += mesh.indices().size();
  }

  // Merge meshes.
  std::vector<Vector3f> vertices;
  vertices.reserve(num_vertices);
  std::vector<Vector3f> normals;
  if (!meshes.front().normals().empty()) {
    normals.reserve(num_vertices);
  }
  std::vector<Vector3f> colors;
  if (!meshes.front().colors().empty()) {
    colors.reserve(num_vertices);
  }
  std::vector<Vector2f> texels;
  if (!meshes.front().texels().empty()) {
    texels.reserve(num_vertices);
  }
  std::vector<uint32_t> indices;
  indices.reserve(num_indices);

  for (const Mesh& mesh : meshes) {
    for (const uint32_t index : mesh.indices()) {
      indices.push_back(index + vertices.size());
    }

    vertices.insert(vertices.end(), mesh.vertices().begin(),
                    mesh.vertices().end());
    normals.insert(normals.end(), mesh.normals().begin(), mesh.normals().end());
    colors.insert(colors.end(), mesh.colors().begin(), mesh.colors().end());
    texels.insert(texels.end(), mesh.texels().begin(), mesh.texels().end());
  }
  return Mesh(vertices, normals, colors, indices, meshes.front().mesh_type());
}

}  // namespace perception
}  // namespace intrinsic
