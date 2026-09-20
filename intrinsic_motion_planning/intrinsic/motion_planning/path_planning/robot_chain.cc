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

#include "intrinsic/motion_planning/path_planning/robot_chain.h"

#include <cstdint>
#include <utility>

#include "intrinsic/world/entity_id.h"

namespace intrinsic {

RobotChain::RobotChain(
    const std::pair<AttachmentEntityId, AttachmentEntityId>& base_tip_ids) {
  base_id = base_tip_ids.first.value();
  tip_id = base_tip_ids.second.value();
}

RobotChain::RobotChain(
    const std::pair<PhysicalEntityId, PhysicalEntityId>& base_tip_ids) {
  base_id = base_tip_ids.first.value();
  tip_id = base_tip_ids.second.value();
}

RobotChain::RobotChain(const std::pair<uint32_t, uint32_t>& base_tip_ids) {
  base_id = base_tip_ids.first;
  tip_id = base_tip_ids.second;
}

RobotChain::RobotChain(uint32_t base, uint32_t tip) {
  base_id = base;
  tip_id = tip;
}

}  // namespace intrinsic
