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

#ifndef INTRINSIC_GEOMETRY_INTERNAL_DISTANCE_3_HAUSDORFF_STOP_PREDICATE_H_
#define INTRINSIC_GEOMETRY_INTERNAL_DISTANCE_3_HAUSDORFF_STOP_PREDICATE_H_

#include <cstddef>
#include <cstdlib>
#include <functional>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/geometry/internal/distance_3/hausdorff_optimization_state.h"
#include "intrinsic/geometry/internal/surface_mesh_3/surface_mesh_3d.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::geo {
/**
 * The HausdorffStopPredicate is a stop predicate for the CGAL edge collapse
 * simplification algorithm. It is used to efficiently find the number of edge
 * collapses that yields a Hausdorff distance closest to the max. This is
 * following the implementation of the CGAL edge collapse simplification
 * algorithm.
 *
 * The HausdorffStopPredicate is initialized with a HausdorffOptimizationState
 * which is initialized with the user provided parameters.
 *
 * The HausdorffStopPredicate overrides the stop_predicate parameter of the CGAL
 * edge collapse simplification algorithm. It is called for every edge collapse
 * and will update the internal optimization points if the correct criteria is
 * met.
 *
 * The internal logic for checking and updating the state is implemented in
 * the HausdorffOptimizationState class.
 *
 * The operator returns `true` when a full step of the Golden Section Search
 * has been completed (i.e., the interval in `HausdorffOptimizationState`
 * has been successfully narrowed). This signals the CGAL `edge_collapse`
 * function to stop the current simplification pass, allowing the
 * `ConstructSimplifiedMeshFunctor` to potentially start a new simplification
 * pass with an updated mesh based on the Golden Section's progress.
 *
 * An additional `custom_early_stop_predicate` function must be passed that
 * allows for immediate stopping of the `HausdorffStopPredicate`. It is required
 * that once the custom predicate returns true, all subsequent calls must also
 * return true else the behavior is undefined
 */

class HausdorffStopPredicate {
 public:
  static absl::StatusOr<HausdorffStopPredicate> ConstructHausdorffStopPredicate(
      const HausdorffOptimizationState::HausdorffOptimizationParams& params,
      std::function<bool()> custom_early_stop_predicate) {
    INTR_ASSIGN_OR_RETURN(
        HausdorffOptimizationState state,
        HausdorffOptimizationState::ConstructHausdorffOptimizationState(
            params));
    return HausdorffStopPredicate(params, std::move(state),
                                  std::move(custom_early_stop_predicate));
  }

  template <typename F, typename Profile>
  bool operator()(const F& /*current_cost*/, const Profile& profile /*profile*/,
                  std::size_t /*initial_edge_count*/,
                  std::size_t /*current_edge_count*/) const {
    if (!status_.ok()) {
      LOG(ERROR) << "Failed to simplify mesh: " << status_;
      return true;
    }
    if (params_.max_hausdorff_distance == 0.0) {
      return true;
    }
    if (ShouldStopEarly()) {
      return true;
    }
    // Update inner points if unset and enough edges have been removed.
    const absl::Status is_optimized = state_.EvaluateInnerOptimizationPoints(
        params_.initial_mesh, profile.surface_mesh());
    if (!is_optimized.ok()) {
      LOG(ERROR) << "Failed to simplify mesh: " << is_optimized.message();
      status_ = is_optimized;
      return true;
    }
    // Check if a Golden Section Search iteration has completed. If so, the
    // state will be updated, and this will return true to stop the current
    // CGAL edge collapse pass.
    absl::StatusOr<bool> is_updated =
        state_.UpdateStateForNextIteration(profile.surface_mesh());
    if (!is_updated.ok()) {
      LOG(ERROR) << "Failed to check if state is updated: "
                 << is_updated.status();
      status_ = is_updated.status();
      return true;
    }
    return *is_updated;
  }
  bool HasConverged() const { return state_.HasConverged(); }

  bool ShouldStopEarly() const { return custom_early_stop_predicate_(); }

  const EpicSurfaceMesh3& GetBestSimplifiedMesh() const {
    return state_.GetBestSimplifiedMesh();
  }
  const EpicSurfaceMesh3& GetInputMesh() const {
    return state_.GetLeftEndpointMesh();
  }
  int GetRange() const { return state_.GetRange(); }
  absl::Status GetStatus() const { return status_; }

 private:
  // Private constructor used by the factory method.
  HausdorffStopPredicate(
      const HausdorffOptimizationState::HausdorffOptimizationParams& params,
      HausdorffOptimizationState state,
      std::function<bool()> custom_early_stop_predicate)
      : params_(params),
        state_(std::move(state)),
        status_(absl::OkStatus()),
        custom_early_stop_predicate_(std::move(custom_early_stop_predicate)) {}

  const HausdorffOptimizationState::HausdorffOptimizationParams params_;
  mutable HausdorffOptimizationState state_;
  mutable absl::Status status_;
  std::function<bool()> custom_early_stop_predicate_;
};
}  // namespace intrinsic::geo
#endif  // INTRINSIC_GEOMETRY_INTERNAL_DISTANCE_3_HAUSDORFF_STOP_PREDICATE_H_
