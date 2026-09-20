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

#ifndef INTRINSIC_SIMULATION_WORLD_GENERATE_COLLISION_BITMASKS_H_
#define INTRINSIC_SIMULATION_WORLD_GENERATE_COLLISION_BITMASKS_H_

#include <cstdint>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"

namespace intrinsic {
namespace simulation {

// The default bitmask used in Gazebo is one with the low sixteen bits set. See
// http://gazebosim.org/tutorials?tut=collide_bitmask&cat=physics#Defaultvalue
constexpr uint32_t kDefaultCollisionBitMask = 0xFFFF;

// Returns a mapping of entity ids to bitmasks for a given mapping of entity IDs
// to the entities with which they *do not* collide, In other words, takes the
// graph induced by the exclusion set of each entity, and uses it to find
// mutually exclusive sets of entities that all collide with each other, until
// we've covered all collision relationships.
//
// In the resulting hash map, for entities with IDs P and Q, we will maintain
// the property that iff the entities collide, then result[P] & result[Q] != 0.
//
// It is not always the case that we can fit all relationships within 32 bits,
// so the algorithm may fail. Also, it might fail due to naievety. It may also
// fail on malformed input, such as having entities listed as neighbors in the
// exclusion graph that are not in the exclusion graph themselves.
absl::StatusOr<absl::flat_hash_map<uint32_t, uint32_t>>
GenerateCollisionBitmasks(
    absl::flat_hash_map<uint32_t, absl::flat_hash_set<uint32_t>>
        exclusion_graph);

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_WORLD_GENERATE_COLLISION_BITMASKS_H_
