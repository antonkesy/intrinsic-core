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

#include "intrinsic/geometry/api/compute_convex_hull.h"

#include <memory>
#include <utility>
#include <vector>

#include "CGAL/convex_hull_3.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/exact_geometry.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/api/geometry_options.h"
#include "intrinsic/geometry/internal/conversion_utils/extract_all_point_3d.h"
#include "intrinsic/geometry/internal/conversion_utils/mesh_conversion.h"
#include "intrinsic/geometry/internal/kernel_3/exact_predicates_inexact_constructions_kernel.h"
#include "intrinsic/geometry/internal/mesh/mesh.h"
#include "intrinsic/geometry/internal/surface_mesh_3/surface_mesh_3d.h"
#include "intrinsic/util/object_store/memoize.h"
#include "intrinsic/util/object_store/object_ref.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::geo {
namespace {

class ComputeConvexHullFunctor {
 public:
  template <typename Geo>
  absl::StatusOr<ObjectRef<Mesh>> operator()(const Geo& geo) const {
    LOG(WARNING) << "WARNING: intrinsic::ConvexHullFunctor does not support\n"
                 << "operator()(const Geo&)) const, with \n"
                 << "Geo = " << Demangle<Geo>();
    return absl::InvalidArgumentError(
        "ConvexHullFunctor does not support argument type.");
  }

  template <>
  absl::StatusOr<ObjectRef<Mesh>> operator()(const ObjectRef<Mesh>& geo) const {
    absl::StatusOr<ObjectRef<Mesh>> result_mesh = Memoize(
        MemoizeOptions("ComputeConvexHullFunctor-v1"),
        [](ObjectRef<Mesh> geo) -> absl::StatusOr<Mesh> {
          std::vector<EpicPoint3> points = EpicExtractAllPoint3(geo.Value());
          EpicSurfaceMesh3 hull;
          CGAL::convex_hull_3(points.begin(), points.end(), hull);
          return ToMesh(hull);
        },
        geo);
    return result_mesh;
  }
};

}  // namespace

absl::StatusOr<Geometry> ComputeConvexHull(const Geometry& geo) {
  INTR_ASSIGN_OR_RETURN(
      auto mesh, geo.GetExactGeometry().visit(ComputeConvexHullFunctor()));
  Geometry::Provenance provenance = {
      .human_readable_update_reason = "Compute convex hull",
      .previous_geometry = std::make_shared<const Geometry>(geo),
  };

  return Geometry(
      ExactGeometry(std::move(mesh), geo.GetExactGeometry().options()),
      /*renderable=*/nullptr, /*keep_renderable=*/false,
      geo.material_properties(), std::move(provenance));
}

}  // namespace intrinsic::geo
