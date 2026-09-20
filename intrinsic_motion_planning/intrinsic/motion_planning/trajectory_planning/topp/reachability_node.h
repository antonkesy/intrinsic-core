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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_REACHABILITY_NODE_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_REACHABILITY_NODE_H_

#include "intrinsic/math/numopt/polytope_utils.h"

namespace intrinsic {
namespace topp {

// Reachability node encapsulating the region and reference path index.
struct ReachabilityNode {
  // The path_sample index corresponding to this reachability region.
  int sample_id;

  // The description of the reachability region.
  ConvexHullResult2d cvx_hull;
};

}  // namespace topp
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_REACHABILITY_NODE_H_
