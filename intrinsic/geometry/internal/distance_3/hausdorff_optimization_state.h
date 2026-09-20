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

#ifndef INTRINSIC_GEOMETRY_INTERNAL_DISTANCE_3_HAUSDORFF_OPTIMIZATION_STATE_H_
#define INTRINSIC_GEOMETRY_INTERNAL_DISTANCE_3_HAUSDORFF_OPTIMIZATION_STATE_H_

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <optional>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/geometry/internal/distance_3/compute_bounded_hausdorff_distance.h"
#include "intrinsic/geometry/internal/surface_mesh_3/surface_mesh_3d.h"
#include "intrinsic/util/status/status_macros.h"

// Error bounds for computing the hausdorff distance
constexpr double kErrorBounds = 1E-5;

namespace intrinsic::geo {
// `HausdorffOptimizationState` tracks the state of a mesh simplification
// process that uses edge collapse. It implements the Golden Section search
// algorithm to efficiently find the number of edge collapses that produces a
// simplified mesh with a Hausdorff distance closest to a specified max without
// exceeding it. For more details on the algorithm, see:
// https://en.wikipedia.org/wiki/Golden_section_method
//
// The process is configured using `HausdorffOptimizationParams`. The
// `max_hausdorff_distance` specifies the maximum geometric deviation.
// `left_check_freq` and `right_check_freq` define the initial search interval
// for the Golden Section method, representing the number of edges to remove
// before a Hausdorff distance calculation. A nominal configuration (e.g., a
// max distance of zero) will result in the original mesh being returned.
//
// The state is initialized with two inner points calculated using the golden
// ratio. The algorithm iteratively narrows the search interval by comparing the
// Hausdorff distances at these inner points. To ensure the simplified mesh's
// Hausdorff distance does not exceed the max, any calculated distance above
// the max is treated as `MAXFLOAT`, effectively penalizing that solution.
//
// The optimization converges when the relative error between the best-found
// Hausdorff distance and the max is within a tolerance, or when the search
// interval becomes too small to refine further (i.e., the difference between
// inner check frequencies is at most 1). The returned mesh is guaranteed to
// have a Hausdorff distance less than or equal to the max, within floating
// point precision.
class HausdorffOptimizationState {
 public:
  struct HausdorffOptimizationParams {
    // Max Hausdorff distance (geometric deviation) to the initial mesh in m.
    double max_hausdorff_distance = 0.0;
    // Left endpoint of golden section method.
    size_t left_check_freq = 0;
    // Right endpoint of golden section method.
    size_t right_check_freq = 0;
    // Initial left Hausdorff distance.
    double left_hausdorff_distance = 0.0;
    // Initial right Hausdorff distance.
    double right_hausdorff_distance = MAXFLOAT;
    // Initial mesh.
    EpicSurfaceMesh3 initial_mesh = EpicSurfaceMesh3();
  };
  // A point in the optimization space.
  struct OptimizationPoint {
    std::optional<size_t> check_freq;          // number of edges removed.
    std::optional<double> hausdorff_distance;  // Hausdorff distance to the
                                               // initial mesh.
    std::optional<EpicSurfaceMesh3> mesh;  // mesh at the optimization point.

    OptimizationPoint() = default;

    OptimizationPoint(std::optional<size_t> check_freq,
                      std::optional<double> hausdorff_distance,
                      std::optional<EpicSurfaceMesh3> mesh)
        : check_freq(check_freq),
          hausdorff_distance(hausdorff_distance),
          mesh(mesh) {}
    bool hasCheckFreqSet() const { return check_freq.has_value(); }
    bool hasValidCheckFreq() const {
      return hasCheckFreqSet() && check_freq.value() >= 0;
    }
    bool hasHausdorffDistanceSet() const {
      return hausdorff_distance.has_value();
    }
    bool hasMeshSet() const { return mesh.has_value(); }
    bool hasValidMembers() const {
      return hasValidCheckFreq() && hasHausdorffDistanceSet() && hasMeshSet();
    }
  };
  // A Point is an optimization point with non-optional fields.
  struct Point {
    size_t check_freq;
    double hausdorff_distance;
    EpicSurfaceMesh3 mesh;
    Point(size_t check_freq, double hausdorff_distance,
          const EpicSurfaceMesh3& mesh)
        : check_freq(check_freq),
          hausdorff_distance(hausdorff_distance),
          mesh(mesh) {}
    static absl::StatusOr<Point> Create(const OptimizationPoint& op) {
      if (!op.hasValidMembers()) {
        return absl::FailedPreconditionError(
            "OptimizationPoint must be valid to create Endpoint.");
      }
      return Point(op.check_freq.value(), op.hausdorff_distance.value(),
                   op.mesh.value());
    }
    absl::Status SetFromOptimizationPoint(const OptimizationPoint& op) {
      if (!op.hasValidMembers()) {
        return absl::FailedPreconditionError(
            "OptimizationPoint must be valid to set Endpoint.");
      }
      check_freq = op.check_freq.value();
      hausdorff_distance = op.hausdorff_distance.value();
      mesh = op.mesh.value();
      return absl::OkStatus();
    }
  };

  static absl::StatusOr<HausdorffOptimizationState>
  ConstructHausdorffOptimizationState(
      const HausdorffOptimizationParams& params) {
    HausdorffOptimizationState state(params);
    INTR_RETURN_IF_ERROR(
        state.CalculateInnerOptimizationPointCheckFrequencies());
    return state;
  }

  // Returns true if the optimization has converged. If the max Hausdorff
  // distance is zero, then the optimization is considered converged. Otherwise,
  // the optimization is considered converged if the relative error between the
  // best-found Hausdorff distance and the max is within a tolerance, or if the
  // search interval becomes too small to refine further (i.e., the difference
  // between inner check frequencies is at most 1).
  bool HasConverged() const {
    if (max_hausdorff_distance_ == 0.0) {
      return true;
    }
    double error =
        abs(max_hausdorff_distance_ - current_best_.hausdorff_distance) /
        max_hausdorff_distance_;
    int inner_range =
        inner_right_.check_freq.value() - inner_left_.check_freq.value();
    return error < kMaximumError || inner_range <= 1;
  }
  // Updates the state for the next iteration based on which inner point has the
  // minimum Hausdorff distance. Returns true if the state was updated.
  absl::StatusOr<bool> UpdateStateForNextIteration(
      const EpicSurfaceMesh3& mesh) {
    if (inner_right_.hasValidMembers() && inner_left_.hasValidMembers()) {
      INTR_RETURN_IF_ERROR(IsMinHausdorffInRightInterval() ? UpdateStateRight()
                                                           : UpdateStateLeft());
      return true;
    }
    edges_removed_ = NumberOfEdgesRemoved(mesh);
    return false;
  }
  // Evaluates the inner optimization points and updates the state if the
  // number of edges removed is greater than or equal to the inner check
  // frequency and the value is not already set.
  absl::Status EvaluateInnerOptimizationPoints(
      const EpicSurfaceMesh3& initial_mesh, const EpicSurfaceMesh3& mesh) {
    INTR_RETURN_IF_ERROR(
        EvaluateOptimizationPoint(initial_mesh, mesh, &inner_left_));
    INTR_RETURN_IF_ERROR(
        EvaluateOptimizationPoint(initial_mesh, mesh, &inner_right_));
    return absl::OkStatus();
  }

  size_t GetRange() const { return right_.check_freq - left_.check_freq; }
  const EpicSurfaceMesh3& GetLeftEndpointMesh() const { return left_.mesh; }
  const EpicSurfaceMesh3& GetBestSimplifiedMesh() const {
    return current_best_.mesh;
  }

 private:
  static constexpr double kInvPhi = 0.382;  // Inverse of the Golden Ratio
  static constexpr double kMaximumError = 0.01;

  // The constructor initializes the state of the optimization with the user
  // provided parameters.
  explicit HausdorffOptimizationState(const HausdorffOptimizationParams& params)
      // endpoints initialized to the user provided parameters.
      : max_hausdorff_distance_(params.max_hausdorff_distance),
        left_(Point(params.left_check_freq, params.left_hausdorff_distance,
                    params.initial_mesh)),
        right_(Point(params.right_check_freq, params.right_hausdorff_distance,
                     EpicSurfaceMesh3())),
        inner_left_(OptimizationPoint()),
        inner_right_(OptimizationPoint()),
        // initial mesh is the best result.
        current_best_(left_),
        edge_count_(params.initial_mesh.edges().size()),
        edges_removed_(0) {}

  // Updates the state for the case where the minimum Hausdorff distance is in
  // the right interval. Returns OkStatus if the state is updated successfully,
  // an error otherwise.
  absl::Status UpdateStateRight() {
    if (!inner_right_.hasValidMembers() || !inner_left_.hasValidMembers()) {
      return absl::FailedPreconditionError(
          "Inner right or left optimization point is not valid.");
    }
    if (inner_right_.hausdorff_distance.value() <= max_hausdorff_distance_) {
      INTR_RETURN_IF_ERROR(
          current_best_.SetFromOptimizationPoint(inner_right_));
    }
    INTR_RETURN_IF_ERROR(left_.SetFromOptimizationPoint(inner_left_));
    inner_left_ = inner_right_;
    inner_right_ = OptimizationPoint();
    INTR_RETURN_IF_ERROR(CalculateInnerOptimizationPointCheckFrequencies());
    return absl::OkStatus();
  }
  // Updates the state for the case where the minimum Hausdorff distance is in
  // the left interval. Returns OkStatus if the state is updated successfully,
  // an error otherwise.
  absl::Status UpdateStateLeft() {
    if (!inner_right_.hasValidMembers() || !inner_left_.hasValidMembers()) {
      return absl::FailedPreconditionError(
          "Inner right or left optimization point is not valid.");
    }
    if (inner_left_.hausdorff_distance.value() <= max_hausdorff_distance_) {
      INTR_RETURN_IF_ERROR(current_best_.SetFromOptimizationPoint(inner_left_));
    }
    INTR_RETURN_IF_ERROR(right_.SetFromOptimizationPoint(inner_right_));
    inner_right_ = inner_left_;
    inner_left_ = OptimizationPoint();
    INTR_RETURN_IF_ERROR(CalculateInnerOptimizationPointCheckFrequencies());
    return absl::OkStatus();
  }
  // Determines if the minimum Hausdorff distance is in the right interval.
  // This is evaluated by comparing the minimum absolute error to
  // `max_hausdorff_distance_` within the `(inner_right_, right_)` range versus
  // the `(left_, inner_left_)` range.
  bool IsMinHausdorffInRightInterval() const {
    double inner_right_error =
        abs(inner_right_.hausdorff_distance.value() - max_hausdorff_distance_);
    double inner_left_error =
        abs(inner_left_.hausdorff_distance.value() - max_hausdorff_distance_);
    double right_error =
        abs(right_.hausdorff_distance - max_hausdorff_distance_);
    double left_error = abs(left_.hausdorff_distance - max_hausdorff_distance_);
    return std::fmin(inner_right_error, right_error) <
           std::fmin(inner_left_error, left_error);
  }
  // Sets the optimization point if the number of edges removed is greater than
  // or equal to the check frequency and the value is not already set.
  absl::Status EvaluateOptimizationPoint(const EpicSurfaceMesh3& initial_mesh,
                                         const EpicSurfaceMesh3& mesh,
                                         OptimizationPoint* point) {
    // Edge Collapse may fail to remove edges so an inner check frequency
    // may never be reached, in that case, we detect this state and update the
    // inner points if they have not yet been updated using the stagnant mesh
    bool edge_removal_predicate =
        NumberOfEdgesRemoved(mesh) >= point->check_freq.value() ||
        ((NumberOfEdgesRemoved(mesh) == edges_removed_) && edges_removed_ > 0);
    if (point->hasCheckFreqSet() && !point->hasHausdorffDistanceSet() &&
        edge_removal_predicate) {
      INTR_ASSIGN_OR_RETURN(
          double hausdorff_distance,
          ComputeBoundedHausdorffDistance(initial_mesh, mesh, kErrorBounds));
      point->hausdorff_distance = hausdorff_distance <= max_hausdorff_distance_
                                      ? hausdorff_distance
                                      : MAXFLOAT;
      point->mesh = mesh;
    }
    return absl::OkStatus();
  }
  // Calculates the inner check frequencies based on the left and right check
  // frequencies.
  absl::Status CalculateInnerOptimizationPointCheckFrequencies() {
    if (left_.check_freq > right_.check_freq) {
      return absl::FailedPreconditionError(
          "Left check frequency must be less than or equal to right check "
          "frequency.");
    }
    inner_left_.check_freq = static_cast<size_t>(std::round(
        kInvPhi * (right_.check_freq - left_.check_freq) + left_.check_freq));
    inner_right_.check_freq = static_cast<size_t>(
        std::round((1 - kInvPhi) * (right_.check_freq - left_.check_freq) +
                   left_.check_freq));
    if (inner_left_.check_freq > right_.check_freq) {
      return absl::FailedPreconditionError(
          "Inner left check frequency must be less than or equal to right "
          "check frequency.");
    }
    return absl::OkStatus();
  }
  size_t NumberOfEdgesRemoved(const EpicSurfaceMesh3& current_mesh) const {
    return edge_count_ - current_mesh.edges().size();
  }
  double max_hausdorff_distance_;
  Point left_;
  Point right_;
  OptimizationPoint inner_left_;
  OptimizationPoint inner_right_;
  Point current_best_;
  size_t edge_count_;
  size_t edges_removed_;
};
}  // namespace intrinsic::geo
#endif  // INTRINSIC_GEOMETRY_INTERNAL_DISTANCE_3_HAUSDORFF_OPTIMIZATION_STATE_H_
