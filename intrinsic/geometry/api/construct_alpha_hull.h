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

#ifndef INTRINSIC_GEOMETRY_API_CONSTRUCT_ALPHA_HULL_H_
#define INTRINSIC_GEOMETRY_API_CONSTRUCT_ALPHA_HULL_H_

#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/geometry.h"

namespace intrinsic::geo {
// Returns a new geometry object that has been made solid using the alpha hull
// algorithm
absl::StatusOr<Geometry> ConstructAlphaHull(const Geometry& geo,
                                            double max_error);

}  // namespace intrinsic::geo
#endif  // INTRINSIC_GEOMETRY_API_CONSTRUCT_ALPHA_HULL_H_
