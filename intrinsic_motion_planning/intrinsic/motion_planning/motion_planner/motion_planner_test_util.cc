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

#include "intrinsic/motion_planning/motion_planner/motion_planner_test_util.h"

#include <optional>

#include "intrinsic/motion_planning/motion_planner/trajectory_generation_utils.h"
#include "intrinsic/motion_planning/proto/v1/motion_blending_parameter.pb.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::testing {
namespace {

constexpr double kMaxDeviation = 1e-10;

// Assumes that each sample in `path_samples` belongs to `path_segment`. Returns
// OkStatus if all path samples are within `max_deviation` c-space distance of
// `path_segment`.
absl::Status CheckThatPathSamplesStrictlyFollowPathSegment(
    absl::Span<const topp::PathSample> path_samples,
    const PathSegment& path_segment) {
  INTR_RET_CHECK(!path_samples.empty());
  INTR_RET_CHECK_EQ(path_samples.at(0).segment_id, path_segment.GetUniqueId());
  INTR_RET_CHECK_GE(path_segment.joint_configurations.size(), 2);
  INTR_RET_CHECK_LE(
      (path_samples.front().q - path_segment.joint_configurations.front())
          .norm(),
      kMaxDeviation);
  if (path_samples.size() == 1) {
    return absl::OkStatus();
  }

  // Precompute necessary info for all edges in `path_segment`.
  struct EdgeInfo {
    // start point of edge
    eigenmath::VectorNd start;

    // End point of edge
    eigenmath::VectorNd end;

    // If edge is too short, this is nullopt. Otherwise, this is the unit vector
    // pointing in the direction of the edge.
    std::optional<eigenmath::VectorNd> dir;

    // Length of the edge from start to endpoint.
    double length;

    // Cumulative arc length from the beginning of the path segment to the start
    // point of this edge.
    double cumulative_arc_length;
  };
  std::vector<EdgeInfo> edge_infos;
  const int num_edges = path_segment.joint_configurations.size() - 1;
  edge_infos.reserve(num_edges);
  double cumulative_arc_length = 0.0;
  for (int edge_idx = 0; edge_idx < num_edges; ++edge_idx) {
    const eigenmath::VectorNd start =
        path_segment.joint_configurations.at(edge_idx);
    const eigenmath::VectorNd end =
        path_segment.joint_configurations.at(edge_idx + 1);
    const eigenmath::VectorNd edge_vec = end - start;
    const double edge_length = edge_vec.norm();
    std::optional<eigenmath::VectorNd> dir;
    // If an edge is much shorter than `kMaxDeviation`, treat the edge as a
    // single point.
    constexpr double kMinimumValidEdgeLength = kMaxDeviation / 10.0;
    if (edge_length >= kMinimumValidEdgeLength) {
      dir = edge_vec / edge_length;
    }
    edge_infos.push_back(
        EdgeInfo{.start = start,
                 .end = end,
                 .dir = dir,
                 .length = edge_length,
                 .cumulative_arc_length = cumulative_arc_length});
    cumulative_arc_length += edge_length;
  }

  // For each path sample, we find the path segment edge it belongs to and check
  // that the distance between the sample and the edge is less than
  // `kMaxDeviation`. We match path samples to segment edges by accumulating
  // path lengths for both the samples and the edges and comparing them.
  int segment_edge_idx = 0;
  double path_length_to_sample = 0.0;
  for (int sample_idx = 1; sample_idx < path_samples.size(); ++sample_idx) {
    const topp::PathSample& path_sample = path_samples.at(sample_idx);
    INTR_RET_CHECK_EQ(path_sample.segment_id, path_segment.GetUniqueId());
    const topp::PathSample& prev_path_sample = path_samples.at(sample_idx - 1);
    path_length_to_sample += (path_sample.q - prev_path_sample.q).norm();
    INTR_RET_CHECK_LT(segment_edge_idx + 1,
                      path_segment.joint_configurations.size());

    // This padding accounts for small errors when accumulating path
    // lengths. Without this padding, we incorrectly associate samples with
    // the following edge and report false errors.
    constexpr double kEndpointPadding = 1e-10;

    // Find the edge in `path_segment` that contains this path sample.
    for (; segment_edge_idx < edge_infos.size(); ++segment_edge_idx) {
      const EdgeInfo& edge = edge_infos.at(segment_edge_idx);
      if (path_length_to_sample <=
          edge.cumulative_arc_length + edge.length + kEndpointPadding) {
        // The sample is on this edge!
        break;
      } else {
        // If this path sample maps to the next edge, then we should confirm
        // that the previous path sample landed near the last edge's endpoint.
        INTR_RET_CHECK_LT((prev_path_sample.q - edge.end).norm(),
                          kMaxDeviation);
      }
    }
    INTR_RET_CHECK_LT(segment_edge_idx, edge_infos.size())
        << "path sample " << sample_idx
        << " does not lie in any edge of its path segment.";

    // Compute distance of path_sample from the segment edge.
    const EdgeInfo& edge = edge_infos.at(segment_edge_idx);
    if (!edge.dir.has_value()) {
      // This edge is very short. Compute the deviation as the distance to the
      // edge's start point.
      const double error = (path_sample.q - edge.start).norm();
      INTR_RET_CHECK_LE(error, kMaxDeviation);
      continue;
    }

    const eigenmath::VectorNd sample_pos_rel_to_edge =
        path_sample.q - edge.start;
    const double sample_dist_along_edge = sample_pos_rel_to_edge.dot(*edge.dir);
    INTR_RET_CHECK_GT(sample_dist_along_edge, -kEndpointPadding)
        << "sample went backward from path segment!";
    INTR_RET_CHECK_LT(sample_dist_along_edge, edge.length + kEndpointPadding)
        << "UNEXPECTED! sample goes beyond edge";
    const eigenmath::VectorNd sample_proj_along_edge =
        sample_dist_along_edge * edge.dir.value();
    const eigenmath::VectorNd sample_offset_from_edge =
        sample_pos_rel_to_edge - sample_proj_along_edge;
    const double sample_deviation_from_segment = sample_offset_from_edge.norm();
    INTR_RET_CHECK_LE(sample_deviation_from_segment, kMaxDeviation);
  }

  return absl::OkStatus();
}
}  // namespace

absl::Status CheckThatPathSamplesStrictlyFollowPathSegments(
    absl::Span<const topp::PathSample> path_samples,
    absl::Span<const PathSegment> path_segments,
    const bool every_sample_must_match_a_segment) {
  INTR_RET_CHECK(!path_samples.empty());
  if (path_segments.empty()) {
    return absl::OkStatus();
  }

  size_t sample_idx = 0;
  for (const PathSegment& path_segment : path_segments) {
    if (!every_sample_must_match_a_segment) {
      // Find first path sample that matches this path segment's ID.
      while (sample_idx < path_samples.size() &&
             path_samples.at(sample_idx).segment_id !=
                 path_segment.GetUniqueId()) {
        ++sample_idx;
      }
    }
    const size_t segment_start_sample_idx = sample_idx;
    // Find contiguous path samples that match this path segment's ID.
    while (sample_idx < path_samples.size() &&
           path_samples.at(sample_idx).segment_id ==
               path_segment.GetUniqueId()) {
      ++sample_idx;
    }
    const size_t sample_count = sample_idx - segment_start_sample_idx;
    INTR_RET_CHECK_GT(sample_count, 0)
        << "Segment " << path_segment.GetUniqueId() << " has no path samples.";

    INTR_RETURN_IF_ERROR(CheckThatPathSamplesStrictlyFollowPathSegment(
        absl::MakeConstSpan(&path_samples[segment_start_sample_idx],
                            sample_count),
        path_segment));
  }

  if (every_sample_must_match_a_segment) {
    INTR_RET_CHECK_EQ(sample_idx, path_samples.size());

    // Check that the last path sample reaches the last configuration in the
    // path segments.
    INTR_RET_CHECK_LT((path_samples.back().q -
                       path_segments.back().joint_configurations.back())
                          .norm(),
                      kMaxDeviation);
  }

  return absl::OkStatus();
}

}  // namespace intrinsic::testing
