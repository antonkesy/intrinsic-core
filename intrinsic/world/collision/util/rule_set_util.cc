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

#include "intrinsic/world/collision/util/rule_set_util.h"

#include "intrinsic/world/proto/collision_action.pb.h"

namespace intrinsic {

const intrinsic_proto::world::CollisionAction& GetDefaultAction() {
  static const intrinsic_proto::world::CollisionAction* default_action = [] {
    auto* action = new intrinsic_proto::world::CollisionAction();
    constexpr double kDefaultMargin = 0.0;
    action->mutable_margin()->set_hard_margin(kDefaultMargin);
    return action;
  }();
  return *default_action;
}

}  // namespace intrinsic
