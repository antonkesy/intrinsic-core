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

#include "intrinsic/motion_planning/motion_planner/trajectory_generation_utils.h"

#include <algorithm>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

absl::Status SetWaypointBlendingToMinimum(
    const int waypoint_segment_index, const int waypoint_joint_index,
    absl::Span<PathSegment> path_segments) {
  INTR_RET_CHECK_GE(waypoint_segment_index, 0)
      << "Waypoint segment index must be non-negative, got "
      << waypoint_segment_index;
  INTR_RET_CHECK_LT(waypoint_segment_index, path_segments.size())
      << "Waypoint segment index " << waypoint_segment_index
      << " is out of range (size: " << path_segments.size() << ")";

  PathSegment& segment = path_segments[waypoint_segment_index];

  INTR_RET_CHECK_EQ(segment.joint_blending_parameter_rad.size(),
                    segment.joint_configurations.size())
      << "Blending parameters and joint configurations size mismatch for "
         "segment "
      << waypoint_segment_index;

  INTR_RET_CHECK_GE(segment.joint_configurations.size(), 2)
      << "Segment " << waypoint_segment_index
      << " must have at least 2 waypoints, but has "
      << segment.joint_configurations.size();

  INTR_RET_CHECK_GE(waypoint_joint_index, 0)
      << "Waypoint joint index must be non-negative, got "
      << waypoint_joint_index;
  INTR_RET_CHECK_LT(waypoint_joint_index,
                    segment.joint_blending_parameter_rad.size())
      << "Waypoint joint index " << waypoint_joint_index
      << " is out of range (size: "
      << segment.joint_blending_parameter_rad.size() << ")";

  const double new_blending_radius = kMinimumLowLevelJointBlendingDeviationRad;
  segment.joint_blending_parameter_rad[waypoint_joint_index] =
      new_blending_radius;

  // Update the adjacent segment's shared waypoint. Otherwise, the TOPP
  // optimizer may prioritize the larger radius, re-generating the colliding
  // spline.

  // If this is the first waypoint of any segment other than the first, modify
  // also the last waypoint of the previous segment.
  if (waypoint_joint_index == 0 && waypoint_segment_index > 0) {
    PathSegment& prev_segment = path_segments[waypoint_segment_index - 1];
    if (!prev_segment.joint_blending_parameter_rad.empty()) {
      prev_segment.joint_blending_parameter_rad.back() = new_blending_radius;
    }
  }
  // If this is the last waypoint of any segment other than the last, modify
  // also the first waypoint of the next segment.
  else if (waypoint_joint_index + 1 == segment.joint_configurations.size() &&
           waypoint_segment_index + 1 < path_segments.size()) {
    PathSegment& next_segment = path_segments[waypoint_segment_index + 1];
    if (!next_segment.joint_blending_parameter_rad.empty()) {
      next_segment.joint_blending_parameter_rad.front() = new_blending_radius;
    }
  }

  return absl::OkStatus();
}

absl::StatusOr<std::vector<AdjustedLineSegment>> AdjustPathBlendingLocally(
    absl::Span<PathSegment> path_segments,
    absl::Span<const topp::PathSample> spline_path_samples,
    absl::Span<const int> invalid_sample_indices) {
  INTR_RET_CHECK(!spline_path_samples.empty())
      << "Spline path samples cannot be empty.";
  INTR_RET_CHECK(!path_segments.empty()) << "Path segments cannot be empty.";

  if (invalid_sample_indices.empty()) {
    return std::vector<AdjustedLineSegment>{};
  }

  for (int idx : invalid_sample_indices) {
    INTR_RET_CHECK_GE(idx, 0)
        << "Invalid sample index cannot be negative: " << idx;
    INTR_RET_CHECK_LT(idx, spline_path_samples.size())
        << "Invalid sample index " << idx
        << " is out of bounds (size: " << spline_path_samples.size() << ")";
  }
  // Make sure the indices are sorted for the filtering below to work properly.
  INTR_RET_CHECK(std::is_sorted(invalid_sample_indices.begin(),
                                invalid_sample_indices.end()))
      << "Invalid sample indices must be sorted.";

  // Group adjacent collision indices and select the center index of each group.
  std::vector<int> grouped_collision_indices;
  int group_start = 0;
  for (int i = 1; i <= invalid_sample_indices.size(); ++i) {
    if (i == invalid_sample_indices.size() ||
        invalid_sample_indices[i] > invalid_sample_indices[i - 1] + 1) {
      const int center_index = group_start + (i - 1 - group_start) / 2;
      grouped_collision_indices.push_back(invalid_sample_indices[center_index]);
      group_start = i;
    }
  }

  if (grouped_collision_indices.empty()) {
    return std::vector<AdjustedLineSegment>{};
  }

  // Precalculate for the polyline points:
  //  1) path lengths: once we have the normalized spline collision sample
  //     path length we can search for the polyline point with the closest
  //     normalized path length.
  //  2) metadata (segment and point index): after we find the closest match
  //     we know whose segment-point blend radius to modify.
  struct PolylinePointData {
    // The segment this waypoint belongs to.
    int segment_index;
    // The index of this waypoint in the segment it belongs.
    int point_index;
    // The arc-length of the polyline from the start to this waypoint.
    double path_length;
  };
  std::vector<PolylinePointData> polyline_points;
  double polyline_path_length = 0.0;
  const eigenmath::VectorNd* prev_joint_configuration = nullptr;
  for (int segment_index = 0; segment_index < path_segments.size();
       ++segment_index) {
    INTR_RET_CHECK_EQ(
        path_segments[segment_index].joint_blending_parameter_rad.size(),
        path_segments[segment_index].joint_configurations.size())
        << "Blending parameters and joint configurations size mismatch for "
           "segment "
        << segment_index;
    for (int config_idx = 0;
         config_idx < path_segments[segment_index].joint_configurations.size();
         ++config_idx) {
      const eigenmath::VectorNd& joint_configuration =
          path_segments[segment_index].joint_configurations[config_idx];
      // Only accumulate distance within the same segment. The transition from
      // the end of segment S-1 to the start of segment S represents the same
      // waypoint, so its physical step distance along the path is zero. This is
      // to ensure that the lower_bound search below does not fall erroneously
      // (due to tiny numerical errors) at the gap between two segments.
      if (config_idx > 0 && prev_joint_configuration != nullptr) {
        polyline_path_length +=
            (joint_configuration - *prev_joint_configuration).norm();
      }
      polyline_points.push_back(
          PolylinePointData{.segment_index = segment_index,
                            .point_index = config_idx,
                            .path_length = polyline_path_length});
      prev_joint_configuration = &joint_configuration;
    }
  }

  // Approximate the spline's true arc length by accumulating Euclidean
  // distances between dense samples, as the B-spline parameter 's' does not map
  // linearly.
  std::vector<double> spline_sample_path_lengths(spline_path_samples.size(),
                                                 0.0);
  for (int i = 1; i < spline_path_samples.size(); ++i) {
    spline_sample_path_lengths[i] =
        spline_sample_path_lengths[i - 1] +
        (spline_path_samples[i].q - spline_path_samples[i - 1].q).norm();
  }
  const double spline_total_length = spline_sample_path_lengths.back();

  // For each collision point, find its normalized spline path length to get
  // the corresponding polyline target path length and then search for the
  // polyline point that is closer to that target path length.
  std::vector<AdjustedLineSegment> adjusted_lines_out;
  // Since collision indices are sorted, target path lengths are monotonic.
  // We can search from the last found polyline point to optimize the search.
  auto it = polyline_points.begin();
  for (const int sample_collision_index : grouped_collision_indices) {
    const double normalized_collision_sample_path_length =
        (spline_total_length > 0.0)
            ? (spline_sample_path_lengths[sample_collision_index] /
               spline_total_length)
            : 0.0;
    const double polyline_target_path_length =
        normalized_collision_sample_path_length * polyline_path_length;

    // Binary search: `polyline_points` are already sorted w.r.t. path lengths.
    it = std::lower_bound(
        it, polyline_points.end(), polyline_target_path_length,
        [](const PolylinePointData& p, double d) { return p.path_length < d; });

    std::vector<PolylinePointData> waypoints_to_update;
    AdjustedLineSegment line_segment;

    if (it == polyline_points.end()) {
      line_segment = {
          .segment_index = polyline_points.back().segment_index,
          .first_point_index = polyline_points.back().point_index,
          .second_point_index = polyline_points.back().point_index,
      };
      waypoints_to_update.push_back(polyline_points.back());
    } else if (it == polyline_points.begin()) {
      line_segment = {
          .segment_index = polyline_points.front().segment_index,
          .first_point_index = polyline_points.front().point_index,
          .second_point_index = polyline_points.front().point_index,
      };
      waypoints_to_update.push_back(polyline_points.front());
    } else {
      const PolylinePointData& prev = *std::prev(it);
      const PolylinePointData& next = *it;

      // Because we do not accumulate path length distance between segment
      // transitions (coincident waypoints at segment boundaries), the target
      // path length will never land in a segment gap. This guarantees that both
      // binary search bounds ('prev' and 'next') belong to the exact same
      // segment.
      INTR_RET_CHECK_EQ(prev.segment_index, next.segment_index)
          << "Segment index mismatch: prev has " << prev.segment_index
          << ", next has " << next.segment_index;
      INTR_RET_CHECK_EQ(prev.point_index + 1, next.point_index)
          << "Point index is not consecutive: prev has " << prev.point_index
          << ", next has " << next.point_index;

      line_segment = {
          .segment_index = prev.segment_index,
          .first_point_index = prev.point_index,
          .second_point_index = next.point_index,
      };

      const double dist_to_prev =
          polyline_target_path_length - prev.path_length;
      const double segment_length = next.path_length - prev.path_length;
      const double relative_pos =
          (segment_length > 1e-9) ? (dist_to_prev / segment_length) : 0.0;

      // Heuristic: Tighten only the closest waypoint if the collision is near
      // the ends (<25% or >75% of the segment), otherwise tighten both.
      if (relative_pos < 0.25) {
        waypoints_to_update.push_back(prev);
      } else if (relative_pos > 0.75) {
        waypoints_to_update.push_back(next);
      } else {
        waypoints_to_update.push_back(prev);
        waypoints_to_update.push_back(next);
      }
      // NOTE: If a collision is skewed toward waypoint q1 which is already at
      // minimum blending, we could theoretically tighten the other endpoint q2.
      // However, if q1 is on a segment boundary, the actual applied blending
      // depends on the adjacent segment's boundary waypoint (q1'), making
      // tie-breaking logic complex. To remain agnostic to the blending policy,
      // we simply set q1 to minimum. If that was already minimum, validation
      // will fail and strict local fallback will tighten both endpoints.
      // This trade-off might cause an extra optimization pass in rare cases,
      // but avoids complex boundary tracking.
    }

    adjusted_lines_out.push_back(line_segment);

    for (const PolylinePointData& waypoint : waypoints_to_update) {
      INTR_RETURN_IF_ERROR(SetWaypointBlendingToMinimum(
          waypoint.segment_index, waypoint.point_index, path_segments));
    }
  }

  return adjusted_lines_out;
}

}  // namespace intrinsic
