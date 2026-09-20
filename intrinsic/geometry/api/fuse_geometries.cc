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

#include "intrinsic/geometry/api/fuse_geometries.h"

#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/affine_transform_of.h"
#include "intrinsic/geometry/api/affine_transform_of_geometry.h"
#include "intrinsic/geometry/api/exact_geometry.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/api/geometry_options.h"
#include "intrinsic/geometry/internal/mesh/concatenate_meshes.h"
#include "intrinsic/geometry/internal/mesh/mesh.h"
#include "intrinsic/util/object_store/object_ref.h"
#include "intrinsic/util/object_store/object_store.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::geo {
namespace {
static absl::Status* kPointCloudFuseError =
    new absl::Status(absl::StatusCode::kInvalidArgument,
                     "Cannot fuse a point cloud with other geometry types.");
}  // namespace

absl::StatusOr<TransformedGeometry> FuseGeometries(
    const std::vector<TransformedGeometry>& transformed_geos) {
  if (transformed_geos.empty()) {
    return TransformedGeometry{Geometry{}};
  } else if (transformed_geos.size() == 1) {
    // This is here as a way to ensure we have consistent errors even though
    // fusing a single point cloud with nothing is a no-op in general.
    if (transformed_geos.front().shape().GetExactGeometry().HasPointCloud()) {
      return *kPointCloudFuseError;
    }

    return transformed_geos.front();
  }

  std::vector<TransformedPrimitiveShapePtr> primitive_shapes;
  std::vector<Mesh> meshes;
  meshes.reserve(transformed_geos.size());
  bool are_primitive_shapes_valid = true;

  GeometryOptions fused_options = GeometryOptions::Default();
  for (const auto& geo : transformed_geos) {
    if (geo.shape().GetExactGeometry().HasPointCloud()) {
      return *kPointCloudFuseError;
    }

    const auto& exact_geo = geo.shape().GetExactGeometry();
    INTR_ASSIGN_OR_RETURN(auto mesh_ref, exact_geo.GetMesh());

    // TODO(stoyang): Evaluate if we should allow merging 'incompatible'
    // geometries based on their options.
    fused_options.MergeWith(exact_geo.options());

    auto mesh = mesh_ref.Value().Clone();
    mesh.Transform(geo.ref_t_shape());
    meshes.push_back(std::move(mesh));

    // Combine the primitive shapes if possible so the result can maintain them
    const auto shapes = exact_geo.GetPrimitiveShapes();
    if (shapes.empty()) {
      are_primitive_shapes_valid = false;
    } else if (are_primitive_shapes_valid) {
      for (const auto& shape : shapes) {
        primitive_shapes.emplace_back(shape.shape(),
                                      geo.ref_t_shape() * shape.ref_t_shape());
      }
    }
  }

  auto mesh = ConcatenateMeshes(meshes);
  ExactGeometry fused_geo = ExactGeometry::CreateEmpty();
  if (are_primitive_shapes_valid) {
    INTR_ASSIGN_OR_RETURN(fused_geo,
                          ExactGeometry::Create(std::move(primitive_shapes),
                                                DeDuplicate(std::move(mesh)),
                                                std::move(fused_options)));
  } else {
    fused_geo = ExactGeometry(std::move(mesh), std::move(fused_options));
  }

  // We lose the idea of provenance and material properties here, because we do
  // not have a single source of geometry but a collection of them.
  return TransformedGeometry(Geometry(std::move(fused_geo),
                                      /*renderable=*/nullptr,
                                      /*keep_renderable=*/false,
                                      /*material_properties=*/std::nullopt,
                                      /*provenance=*/std::nullopt));
}

}  // namespace intrinsic::geo
