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

#ifndef INTRINSIC_MATH_NUMOPT_POLYTOPE_UTILS_H_
#define INTRINSIC_MATH_NUMOPT_POLYTOPE_UTILS_H_

#include <type_traits>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Type of solvers supported for geometry computations.
enum class GeometrySolverType {
  kQHull,                // Quickhull algorithm.
  kCGAL,                 // The Computational Geometry Algorithms Library.
  kAndrewMonotoneChain,  // Andrew's Monotone Chain for 2D convex hull.
  kIncrementalDualHull,  // Supports only halfspace intersections in 3D using
                         // polar duality and incremental 3D convex hull.
};

// Represents a bounded convex polyhedron (polytope) in n-dimensional space.
// Stores both the constraint-based definition (H-representation)
// and the point-based definition (V-representation) of a convex hull.
template <int Dim = Eigen::Dynamic>
struct ConvexHullResult {
  // Use the standard aliases for 2D and 3D to ensure alignment flags match.
  using VectorType =
      std::conditional_t<Dim == 2, eigenmath::Vector2d,
                         std::conditional_t<Dim == 3, eigenmath::Vector3d,
                                            eigenmath::Vector<double, Dim>>>;
  using MatrixType = eigenmath::Matrix<double, Eigen::Dynamic, Dim>;

  // The normal vectors (coefficient matrix) of the linear inequality system
  // defining the facets: A * x <= b. It has size [num_facets x
  // point_dimension]. Each row is a facet normal.
  MatrixType A;

  // The offset vector of the linear inequality system: A * x <= b.
  // It has size [num_facets x 1]. Each element corresponds to a row in A.
  eigenmath::VectorXd b;

  // The extreme points (corners) of the convex hull. These are the minimal
  // points required to form the polytope (the V-representation). It contains
  // `num_vertices` vertices. Each vertex is of size [point_dimension x 1].
  std::vector<VectorType> vertices;
};

// Aliases for ease of use.
using ConvexHullResult2d = ConvexHullResult<2>;
using ConvexHullResult3d = ConvexHullResult<3>;
using ConvexHullResultXd = ConvexHullResult<Eigen::Dynamic>;
using HalfspaceIntersection2d = std::vector<eigenmath::Vector2d>;
using HalfspaceIntersection3d = std::vector<eigenmath::Vector3d>;
using HalfspaceIntersectionXd = std::vector<eigenmath::VectorXd>;

// Computes the vertices of a convex polyhedron formed by the intersection
// of multiple half-spaces defined by the system `A * x <= b`.
// `matrix_a` is an (M x N) matrix where each row is a normal vector 'a_i'.
// `vector_b` is an (M x 1) vector where each element 'b_i' is the offset.
// `interior_point` is an (N x 1) point strictly inside the intersection
//        (must satisfy a_i * interior_point < b_i for all i).
// Returns an std::vector of size V containing all the intersection vertices.
// Each element is a vertex of size (N x 1). The output vertices are not
// necessarily ordered (e.g., in clockwise order). If the input data has
// inconsistent size or if the `interior_point` is not strictly interior, an
// error status is returned. NOTE: This function does not perform an explicit
// check to ensure the polyhedron defined by `matrix_a` and `vector_b` is
// bounded. If the constraints form an unbounded region, the algorithm will
// silently discard intersections at infinity. The returned halfspace
// intersection will contain only the finite extreme points of the region, which
// is insufficient to fully reconstruct the unbounded shape.
// For 2D point dimension.
absl::StatusOr<HalfspaceIntersection2d> ComputeHalfspaceIntersection2d(
    const Eigen::Ref<const eigenmath::MatrixX2d>& matrix_a,
    const Eigen::Ref<const eigenmath::VectorXd>& vector_b,
    const Eigen::Ref<const eigenmath::Vector2d>& interior_point);

// Same as above, but for 3D point dimension. This function also takes an
// additional parameter to select the `geometry_solver_type` used for all
// geometric calculations. It defaults to using the QuickHull algorithm.
absl::StatusOr<HalfspaceIntersection3d> ComputeHalfspaceIntersection3d(
    const Eigen::Ref<const eigenmath::MatrixX3d>& matrix_a,
    const Eigen::Ref<const eigenmath::VectorXd>& vector_b,
    const Eigen::Ref<const eigenmath::Vector3d>& interior_point,
    const GeometrySolverType geometry_solver_type = GeometrySolverType::kQHull);

// Same as above, but for dynamic point dimension.
absl::StatusOr<HalfspaceIntersectionXd> ComputeHalfspaceIntersectionXd(
    const Eigen::Ref<const eigenmath::MatrixXd>& matrix_a,
    const Eigen::Ref<const eigenmath::VectorXd>& vector_b,
    const Eigen::Ref<const eigenmath::VectorXd>& interior_point);

// Computes the convex hull of a given set of `candidate_vertices`. This
// function computes both the half-space representation (H-rep: Ax <= b) and the
// vertex representation (V-rep) of the convex hull for the provided
// `candidate_vertices`.
// The input `candidate_vertices` contains individual points or potential
// vertices of the hull. Returns a ConvexHullResult containing the minimal
// bounding facets (A, b) and extreme vertices, or an error if the input is
// empty.
// Note: The returned vertices (V-rep) represent the set of extreme points and
// are not guaranteed to follow any specific ordering (e.g., Counter-Clockwise),
// especially when using the default QuickHull solver.
// For 2D point dimension. This version takes an additional parameter to select
// the `geometry_solver_type` used for all geometric calculations. It defaults
// to using the QuickHull algorithm.
absl::StatusOr<ConvexHullResult2d> ComputeConvexHull2d(
    const HalfspaceIntersection2d& candidate_vertices,
    const GeometrySolverType geometry_solver_type = GeometrySolverType::kQHull);

// Same as above, but for 3D point dimension.
absl::StatusOr<ConvexHullResult3d> ComputeConvexHull3d(
    const HalfspaceIntersection3d& candidate_vertices);

// Same as above, but for dynamic point dimension.
absl::StatusOr<ConvexHullResultXd> ComputeConvexHullXd(
    const HalfspaceIntersectionXd& candidate_vertices);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_NUMOPT_POLYTOPE_UTILS_H_
