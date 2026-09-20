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

#ifndef INTRINSIC_GEOMETRY_API_CONSTRUCT_ISOTROPIC_REMESHING_H_
#define INTRINSIC_GEOMETRY_API_CONSTRUCT_ISOTROPIC_REMESHING_H_

#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/internal/surface_mesh_3/surface_mesh_3d.h"

namespace intrinsic::geo {
// Constructs a new geometry object by isotropic remeshing the input mesh.
// The edge_reduction_factor is the target reduction of the edge length of the
// mesh i.e. the ratio of the edge length of the remeshed mesh to the maximum
// edge length and the num_iterations is the maximum number of iterations to
// run the remeshing algorithm.
absl::StatusOr<Geometry> ConstructIsotropicRemeshing(
    const Geometry& geo, double edge_reduction_factor, int num_iterations);

}  // namespace intrinsic::geo
#endif  // INTRINSIC_GEOMETRY_API_CONSTRUCT_ISOTROPIC_REMESHING_H_
