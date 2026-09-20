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

#include "intrinsic/geometry/api/construct_simplified_mesh.h"

#include <memory>
#include <optional>
#include <utility>

#include "CGAL/Surface_mesh_simplification/edge_collapse.h"
#include "CGAL/boost/graph/helpers.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/geometry/api/exact_geometry.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/internal/conversion_utils/mesh_conversion.h"
#include "intrinsic/geometry/internal/distance_3/hausdorff_optimization_state.h"
#include "intrinsic/geometry/internal/distance_3/hausdorff_stop_predicate.h"
#include "intrinsic/geometry/internal/mesh/mesh.h"
#include "intrinsic/geometry/internal/surface_mesh_3/surface_mesh_3d.h"
#include "intrinsic/geometry/internal/util/timeout_predicate.h"
#include "intrinsic/util/object_store/memoize.h"
#include "intrinsic/util/object_store/object_ref.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/stop_token.h"

namespace intrinsic::geo {
namespace {

class ConstructSimplifiedMeshFunctor {
 public:
  explicit ConstructSimplifiedMeshFunctor(
      double max_hausdorff_distance, const StopToken& stop_token,
      const TimeoutPredicate& custom_early_stop_predicate)
      : max_hausdorff_distance_(max_hausdorff_distance),
        stop_token_(stop_token),
        custom_early_stop_predicate_(custom_early_stop_predicate) {}

  template <typename Geo>
  absl::StatusOr<ObjectRef<Mesh>> operator()(const Geo& geo) const {
    LOG(WARNING) << "WARNING: intrinsic::ConstructSimplifiedMeshFunctor does "
                    "not support\n"
                 << "operator()(const Geo&)) const, with \n"
                 << "Geo = " << Demangle<Geo>();
    return absl::InvalidArgumentError(
        "ConstructSimplifiedMeshFunctor does not support argument type.");
  }

  /**
   * Constructs a simplified mesh using edge collapse with a custom stop
   * predicate that uses the Hausdorff distance as a stop criterion. The max
   * Hausdorff distance, with units of meters, must be greater than 0.0.
   * `HausdorffStopPredicate` employs a golden section search to efficiently
   * find the number of edge collapses that yields a Hausdorff distance closest
   * to the max.
   */
  template <>
  absl::StatusOr<ObjectRef<Mesh>> operator()(const ObjectRef<Mesh>& geo) const {
    return Memoize(
        MemoizeOptions("ConstructSimplifiedMeshFunctor-v1"),
        [](ObjectRef<Mesh> geo, double max_hausdorff_distance,
           const StopToken& stop_token,
           const TimeoutPredicate& custom_early_stop_predicate)
            -> absl::StatusOr<Mesh> {
          INTR_ASSIGN_OR_RETURN(EpicSurfaceMesh3 surface_mesh,
                                ToEpicSurfaceMesh3WithCleaning(geo.Value()));
          if (CGAL::is_closed(surface_mesh)) {
            LOG(WARNING) << "Non-watertight mesh provided to "
                            "ConstructSimplifiedMesh. Resulting mesh may be "
                            "non-manifold.";
          }
          // Should stop early if stop token issues stop request or custom
          // predicate criteria met
          auto combined_early_stop_predicate = [custom_early_stop_predicate,
                                                &stop_token]() {
            return stop_token.stop_requested() ||
                   custom_early_stop_predicate.HasTimedOut();
          };
          INTR_ASSIGN_OR_RETURN(
              HausdorffStopPredicate stop_predicate,
              HausdorffStopPredicate::ConstructHausdorffStopPredicate(
                  HausdorffOptimizationState::HausdorffOptimizationParams{
                      .max_hausdorff_distance = max_hausdorff_distance,
                      .right_check_freq = surface_mesh.edges().size(),
                      .initial_mesh = surface_mesh,
                  },
                  combined_early_stop_predicate));
          while (!stop_predicate.ShouldStopEarly() &&
                 !stop_predicate.HasConverged() &&
                 !surface_mesh.edges().empty()) {
            // After each Golden Section iteration, the input mesh for
            // simplification is updated to the mesh associated with the left
            // endpoint of the current interval. This is done to avoid
            // unnecessary resimplification. If the inner left point had the
            // minimum the error for the previous iteration, then the input
            // mesh will be the same as the input mesh of that previous
            // iteration. But if the inner right point had the minimum error,
            // then the input mesh will be the mesh associated with the
            // inner left pointof the previous iteration. Note the left
            // endpoint of the current iteration is the same as the inner left
            // point of the previous iteration in this case.
            surface_mesh = stop_predicate.GetInputMesh();
            CGAL::Surface_mesh_simplification::edge_collapse(surface_mesh,
                                                             stop_predicate);
            if (!stop_predicate.GetStatus().ok()) {
              return stop_predicate.GetStatus();
            }
          }
          if (stop_token.stop_requested()) {
            return absl::CancelledError(
                "ConstructSimplifiedMeshFunctor cancelled.");
          }
          // Processing to clean up the mesh after remeshing to avoid unhalting
          // mesh conversions.
          return ToMeshWithCleaning(stop_predicate.GetBestSimplifiedMesh());
        },
        geo, max_hausdorff_distance_, stop_token_,
        custom_early_stop_predicate_);
  }

  double max_hausdorff_distance_;
  const StopToken& stop_token_;
  const TimeoutPredicate& custom_early_stop_predicate_;
};

}  // namespace

absl::StatusOr<Geometry> ConstructSimplifiedMesh(
    const Geometry& geo, double max_hausdorff_distance,
    const StopToken& stop_token,
    const TimeoutPredicate& custom_early_stop_predicate) {
  if (max_hausdorff_distance < 0.0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Max Hausdorff distance must be greater than 0.0"
                        "but %f was provided.",
                        max_hausdorff_distance));
  }
  if (max_hausdorff_distance == 0.0) {
    return geo;
  }
  if (!geo.GetExactGeometry().GetPrimitiveShapes().empty()) {
    return absl::InvalidArgumentError(
        "Simplified mesh does not support primitive shapes.");
  }
  INTR_ASSIGN_OR_RETURN(const ObjectRef<Mesh>& mesh,
                        geo.GetExactGeometry().GetMesh());
  if (mesh.Value().empty()) {
    LOG(WARNING) << "Empty mesh provided to ConstructSimplifiedMesh.";
    return geo;
  }
  if (custom_early_stop_predicate.HasTimedOut()) {
    return geo;
  }
  INTR_ASSIGN_OR_RETURN(
      ObjectRef<Mesh> simplified_mesh,
      geo.GetExactGeometry().visit(ConstructSimplifiedMeshFunctor(
          max_hausdorff_distance, stop_token, custom_early_stop_predicate)));
  Geometry::Provenance provenance = {
      .human_readable_update_reason = "Construct simplified mesh",
      .previous_geometry = std::make_shared<const Geometry>(geo),
  };

  return Geometry(ExactGeometry(std::move(simplified_mesh),
                                geo.GetExactGeometry().options()),
                  std::move(provenance));
}

}  // namespace intrinsic::geo
