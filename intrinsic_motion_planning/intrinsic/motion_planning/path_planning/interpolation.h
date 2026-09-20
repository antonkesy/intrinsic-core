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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_INTERPOLATION_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_INTERPOLATION_H_

#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Return an interpolated point in ratio t of the overall distance between
// points in baked_path. The interpolated point is located between points in
// index prev, and prev + 1, where prev is the second returned value.
std::tuple<eigenmath::VectorXd, int> InterpolateQ(
    double t, const std::vector<eigenmath::VectorXd>& baked_path);

// Linearly interpolates the `path` with the given spacing, and applies fn at
// each point sequentially. If fn returns true for an interpolated point, this
// function immediately returns this point and the index of the element in
// `path` immediately before this point. If fn returns false for all points
// checked, the function returns nullopt.
//
// Important things to note:
//   * This function does not explicit check the start and end of the path. We
//     usually handle these separately anyway.
//   * This function does not do anything special to enforce checks AT the
//     points that define `path`. The logic here is that we prefer to follow the
//     the provided `spacing` rather than whatever discretization `path` uses.
//   * This implementation iterates through the points from beginning to end.
absl::StatusOr<std::optional<std::tuple<eigenmath::VectorXd, int>>>
InterpolateAndCheck(
    const std::vector<eigenmath::VectorXd>& path, double spacing,
    const std::function<absl::StatusOr<bool>(const eigenmath::VectorXd&)>& fn);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_INTERPOLATION_H_
