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

#include "intrinsic/motion_planning/motion_planner/motion_planning_path_segment_type_utils.h"

#include "intrinsic/motion_planning/path_planning/path_segment.h"

namespace intrinsic {

bool IsJointOrAnyMove(const PathSegment::Type& path_segment_type) {
  return (path_segment_type == PathSegment::Type::kJoint) ||
         (path_segment_type == PathSegment::Type::kAny);
}

bool IsLinearMove(const PathSegment::Type& path_segment_type) {
  return path_segment_type == PathSegment::Type::kLinear;
}
}  // namespace intrinsic
