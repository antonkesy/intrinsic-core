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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ROBOT_CHAIN_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ROBOT_CHAIN_H_

#include <cstdint>
#include <utility>

#include "intrinsic/world/entity_id.h"

namespace intrinsic {

// A container to track the base and tip of a robot chain.
struct RobotChain {
  uint32_t base_id;
  uint32_t tip_id;

  explicit RobotChain(
      const std::pair<AttachmentEntityId, AttachmentEntityId>& base_tip_ids);

  explicit RobotChain(
      const std::pair<PhysicalEntityId, PhysicalEntityId>& base_tip_ids);

  explicit RobotChain(const std::pair<uint32_t, uint32_t>& base_tip_ids);

  RobotChain(uint32_t base, uint32_t tip);

  // This is added to allow RobotChain to be inserted into sorted containers.
  inline bool operator<(const RobotChain& rhs) const {
    return this->base_id < rhs.base_id;
  }
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_ROBOT_CHAIN_H_
