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

#include "intrinsic/geometry/api/construct_isotropic_remeshing.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "CGAL/Polygon_mesh_processing/measure.h"
#include "CGAL/Polygon_mesh_processing/remesh.h"
#include "CGAL/boost/graph/graph_traits_Surface_mesh.h"
#include "CGAL/boost/graph/helpers.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/geometry/api/exact_geometry.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/internal/conversion_utils/mesh_conversion.h"
#include "intrinsic/geometry/internal/mesh/mesh.h"
#include "intrinsic/geometry/internal/surface_mesh_3/surface_mesh_3d.h"
#include "intrinsic/util/object_store/memoize.h"
#include "intrinsic/util/object_store/object_ref.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::geo {

using CGAL::parameters::number_of_iterations;

namespace {

// Get the max edge length of the mesh.
double GetMaxEdgeLength(const EpicSurfaceMesh3& surface_mesh) {
  std::vector<double> edge_lengths;
  edge_lengths.reserve(surface_mesh.edges().size());
  for (const auto& edge : surface_mesh.edges()) {
    edge_lengths.push_back(CGAL::Polygon_mesh_processing::edge_length(
        halfedge(edge, surface_mesh), surface_mesh));
  }
  if (edge_lengths.empty()) {
    return 0.0;
  }
  return *std::max_element(edge_lengths.begin(), edge_lengths.end());
}

// Returns the length of the longest boundary edge of the mesh. This is used
// to precondition the isotropic remeshing.
double GetLongestBoundaryEdge(const EpicSurfaceMesh3& surface_mesh) {
  if (CGAL::is_closed(surface_mesh)) {
    return 0.0;
  }
  double max_constraint_length_sq = 0.0;
  for (const auto& halfedge : halfedges(surface_mesh)) {
    if (!surface_mesh.is_border(halfedge)) {
      continue;
    }
    auto source_vertex = CGAL::source(halfedge, surface_mesh);
    auto target_vertex = CGAL::target(halfedge, surface_mesh);
    const auto& source_point = surface_mesh.point(source_vertex);
    const auto& target_point = surface_mesh.point(target_vertex);
    double current_length_sq = (source_point - target_point).squared_length();
    max_constraint_length_sq =
        std::max(max_constraint_length_sq, current_length_sq);
  }
  return std::sqrt(max_constraint_length_sq);
}

class ConstructIsotropicRemeshingFunctor {
 public:
  explicit ConstructIsotropicRemeshingFunctor(double edge_reduction_factor,
                                              int num_iterations)
      : edge_reduction_factor_(edge_reduction_factor),
        num_iterations_(num_iterations) {}

  template <typename Geo>
  absl::StatusOr<ObjectRef<Mesh>> operator()(const Geo& geo) const {
    LOG(WARNING)
        << "WARNING: intrinsic::ConstructIsotropicRemeshingFunctor does "
           "not support\n"
        << "operator()(const Geo&)) const, with \n"
        << "Geo = " << Demangle<Geo>();
    return absl::InvalidArgumentError(
        "ConstructIsotropicRemeshingFunctor does not support argument type.");
  }

  template <>
  absl::StatusOr<ObjectRef<Mesh>> operator()(const ObjectRef<Mesh>& geo) const {
    return Memoize(
        MemoizeOptions("ConstructIsotropicRemeshingFunctor-v1"),
        [](ObjectRef<Mesh> geo, double edge_reduction_factor,
           int num_iterations) -> absl::StatusOr<Mesh> {
          INTR_ASSIGN_OR_RETURN(EpicSurfaceMesh3 surface_mesh,
                                ToEpicSurfaceMesh3WithCleaning(geo.Value()));
          if (surface_mesh.is_empty()) {
            return geo.Value().Clone();
          }
          double max_mesh_edge_length = GetMaxEdgeLength(surface_mesh);
          double target_edge_length_desired =
              max_mesh_edge_length * edge_reduction_factor;
          double longest_constraint_edge = GetLongestBoundaryEdge(surface_mesh);
          // Internally CGAL guards against processing meshes if the target edge
          // length is smaller than 3/4 of the mesh edge length or the longest
          // constraint edge.
          const double kPreconditionRatio = 0.75;
          // This precondition is necessary to avoid failure in CGAL.
          double target_edge_length_min =
              longest_constraint_edge * kPreconditionRatio;
          double target_edge_length =
              std::max(target_edge_length_desired, target_edge_length_min);
          CGAL::Polygon_mesh_processing::isotropic_remeshing(
              surface_mesh.faces(), target_edge_length, surface_mesh,
              number_of_iterations(num_iterations).protect_constraints(true));
          return ToMeshWithCleaning(surface_mesh);
        },
        geo, edge_reduction_factor_, num_iterations_);
  }

  double edge_reduction_factor_;
  int num_iterations_;
};

}  // namespace

absl::StatusOr<Geometry> ConstructIsotropicRemeshing(
    const Geometry& geo, double edge_reduction_factor, int num_iterations) {
  if (edge_reduction_factor <= 0.0 || edge_reduction_factor > 1.0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Edge reduction factor must be between 0.0 and 1.0 but "
                        "%f was provided.",
                        edge_reduction_factor));
  }
  if (num_iterations <= 0 || num_iterations > 100) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Number of iterations must be positive and less than "
                        "100 but %d was provided.",
                        num_iterations));
  }
  if (!geo.GetExactGeometry().GetPrimitiveShapes().empty()) {
    return absl::InvalidArgumentError(
        "Isotropic remeshing does not support primitive shapes.");
  }
  INTR_ASSIGN_OR_RETURN(
      ObjectRef<Mesh> mesh,
      geo.GetExactGeometry().visit(ConstructIsotropicRemeshingFunctor(
          edge_reduction_factor, num_iterations)));

  Geometry::Provenance provenance = {
      .human_readable_update_reason = "Construct isotropic remeshing",
      .previous_geometry = std::make_shared<const Geometry>(geo),
  };

  return Geometry(
      ExactGeometry(std::move(mesh), geo.GetExactGeometry().options()),
      std::move(provenance));
}

}  // namespace intrinsic::geo
