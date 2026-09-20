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

#include "intrinsic/motion_planning/path_planning/interpolation.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/substitute.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/util/eigen.h"

namespace intrinsic {
namespace {

std::string ToString(const std::vector<eigenmath::VectorXd>& path) {
  std::string result = "{\n";
  for (const auto& point : path) {
    absl::StrAppend(&result, "  ", toString(point), "\n");
  }
  absl::StrAppend(&result, "}\n");

  return result;
}

}  // namespace

std::tuple<eigenmath::VectorXd, int> InterpolateQ(
    double t, const std::vector<eigenmath::VectorXd>& baked_path) {
  // If a motion plan contains more than 2 waypoints, the motion is interpolated
  // along the whole path (in joint space). So, consider the following
  // waypoints:
  //   a: (0, 0)
  //   b: (0, 1)
  //   c: (3, 1)
  // In this situation, since points b and c are 3x farther apart than points a
  // and b, t=0 -> t=0.25 will be spent between points a and b, while t=0.25 ->
  // t=1 will be spent between points b and c.
  CHECK(!baked_path.empty());
  CHECK_GE(t, 0);
  CHECK_LE(t, 1);
  if (t <= 0) {
    return std::make_tuple(baked_path.front(), 0);
  }
  if (t >= 1) {
    return std::make_tuple(baked_path.back(), baked_path.size() - 1);
  }
  if (baked_path.size() == 1) {
    return std::make_tuple(baked_path.front(), 0);
  }

  std::vector<double> distance(baked_path.size());
  double total_distance = 0;
  distance[0] = 0;
  for (int i = 0; i < baked_path.size() - 1; ++i) {
    total_distance += (baked_path[i + 1] - baked_path[i]).stableNorm();
    distance[i + 1] = total_distance;
  }

  // If total distance is 0, it doesn't matter what we return, they're the same
  if (total_distance == 0) {
    return std::make_tuple(baked_path.front(), 0);
  }

  double path_distance = t * total_distance;

  int next_point =
      std::lower_bound(distance.begin(), distance.end(), path_distance) -
      distance.begin();
  int prev_point = next_point - 1;
  CHECK_GE(prev_point, 0) << "t: " << t << " total_distance: " << total_distance
                          << " path_distance: " << path_distance
                          << " next_point: " << next_point
                          << "\npath=" << ToString(baked_path);
  CHECK_LT(next_point, baked_path.size())
      << "t: " << t << " total_distance: " << total_distance
      << " path_distance: " << path_distance << " next_point: " << next_point
      << "\npath=" << ToString(baked_path);

  double sub_t = (path_distance - distance[prev_point]) /
                 (distance[next_point] - distance[prev_point]);
  if (isnan(sub_t)) {  // There's a no-op move.
    sub_t = 0;
  }
  return std::make_tuple(
      (baked_path[prev_point] +
       (baked_path[next_point] - baked_path[prev_point]) * sub_t),
      prev_point);
}

absl::StatusOr<std::optional<std::tuple<eigenmath::VectorXd, int>>>
InterpolateAndCheck(
    const std::vector<eigenmath::VectorXd>& path, double spacing,
    const std::function<absl::StatusOr<bool>(const eigenmath::VectorXd&)>& fn) {
  if (path.empty()) {
    return absl::InvalidArgumentError(
        "Cannot check path with no points defined.");
  }
  if (spacing <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Spacing between collision checks must be positive. Got ", spacing));
  }

  double total_distance = 0;
  for (int i = 0; i < path.size() - 1; ++i) {
    total_distance += (path[i + 1] - path[i]).stableNorm();
  }

  double current_distance = spacing;
  double distance_on_current_segment = 0;
  int current_index = 0;
  while (current_distance < total_distance) {
    // Walk forward until we find the segment the next point should lie on.
    double distance_remaining = spacing;
    while (true) {
      double current_segment_length =
          (path.at(current_index) - path.at(current_index + 1)).stableNorm();
      if (distance_on_current_segment + distance_remaining >
          current_segment_length) {
        current_index++;
        if (current_index == path.size() - 1) {
          return absl::InternalError(
              "Collision checking interpolation: Unable to access next point "
              "for collision checking. This is an internal motion planning "
              "programming error.");
        }
        // The usage of distance_on_current_segment is important. We could be
        // already be partway through a segment.
        distance_remaining -=
            current_segment_length - distance_on_current_segment;
        distance_on_current_segment = 0;

        // Try again on the next segment.
        continue;
      }
      // The next point to check lies on this segment. Exit the loop and do the
      // actual check.
      break;
    }

    // We found the segment our next point lies on. The remaining distance is
    // how far along the segment we need to go.
    //
    // += is important here because we could be stepping along the same segment.
    distance_on_current_segment += distance_remaining;

    // Get the actual point on the segment.
    double distance_ratio =
        distance_on_current_segment /
        (path.at(current_index) - path.at(current_index + 1)).stableNorm();
    eigenmath::VectorXd point =
        path.at(current_index) +
        (path.at(current_index + 1) - path.at(current_index)) * distance_ratio;

    // Run this point through the user-provided callback.
    absl::StatusOr<bool> result = fn(point);
    if (!result.ok()) {
      return absl::InternalError(absl::Substitute(
          "User provided callback failed at point ($0). Error: $1",
          toString(point), result.status().ToString()));
    }
    if (result.value()) {
      return std::make_tuple(point, current_index);
    }

    current_distance += spacing;
  }

  return std::nullopt;
}

}  // namespace intrinsic
