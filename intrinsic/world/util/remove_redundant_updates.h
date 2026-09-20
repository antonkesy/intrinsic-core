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

#ifndef INTRINSIC_WORLD_UTIL_REMOVE_REDUNDANT_UPDATES_H_
#define INTRINSIC_WORLD_UTIL_REMOVE_REDUNDANT_UPDATES_H_

#include "absl/status/statusor.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"

namespace intrinsic {

// Removes redundant updates. For example two updates that both set the joint
// values of a robot one after the other can be collapsed to just the second
// update as it overrides the first one completely.
absl::StatusOr<intrinsic_proto::world::ObjectWorldUpdates>
RemoveRedundantUpdates(
    intrinsic_proto::world::ObjectWorldUpdates input_updates);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_REMOVE_REDUNDANT_UPDATES_H_
