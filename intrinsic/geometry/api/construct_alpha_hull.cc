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

#include "intrinsic/geometry/api/construct_alpha_hull.h"

#include <memory>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/exact_geometry.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/api/geometry_options.h"
#include "intrinsic/geometry/internal/alpha_hull_3/alpha_hull_3.h"
#include "intrinsic/geometry/internal/conversion_utils/mesh_conversion.h"
#include "intrinsic/geometry/internal/mesh/mesh.h"
#include "intrinsic/util/object_store/memoize.h"
#include "intrinsic/util/object_store/object_ref.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::geo {
namespace {

class ConstructAlphaHullFunctor {
 public:
  explicit ConstructAlphaHullFunctor(double max_error)
      : max_error_(max_error) {}

  template <typename Geo>
  absl::StatusOr<ObjectRef<Mesh>> operator()(const Geo& geo) const {
    LOG(WARNING)
        << "WARNING: intrinsic::ConstructAlphaHullFunctor does not support\n"
        << "operator()(const Geo&)) const, with \n"
        << "Geo = " << Demangle<Geo>();
    return absl::InvalidArgumentError(
        "ConstructAlphaHullFunctor does not support argument type.");
  }

  template <>
  absl::StatusOr<ObjectRef<Mesh>> operator()(const ObjectRef<Mesh>& geo) const {
    absl::StatusOr<ObjectRef<Mesh>> result_mesh = Memoize(
        MemoizeOptions("ConstructAlphaHullFunctor-v1"),
        [](ObjectRef<Mesh> geo, double max_error) -> absl::StatusOr<Mesh> {
          INTR_ASSIGN_OR_RETURN(
              auto hull,
              gsan::AlphaHull3(ToEpicSurfaceMesh3(geo.Value()), max_error));
          return ToMesh(hull);
        },
        geo, max_error_);
    return result_mesh;
  }

  double max_error_;
};

}  // namespace

absl::StatusOr<Geometry> ConstructAlphaHull(const Geometry& geo,
                                            double max_error) {
  INTR_ASSIGN_OR_RETURN(auto mesh, geo.GetExactGeometry().visit(
                                       ConstructAlphaHullFunctor(max_error)));
  Geometry::Provenance provenance = {
      .human_readable_update_reason = "Construct alpha hull",
      .previous_geometry = std::make_shared<const Geometry>(geo),
  };

  return Geometry(
      ExactGeometry(std::move(mesh), geo.GetExactGeometry().options()),
      std::move(provenance));
}

}  // namespace intrinsic::geo
