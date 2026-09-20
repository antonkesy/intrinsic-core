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

#ifndef INTRINSIC_SKILLS_INTERNAL_CONFLICTS_H_
#define INTRINSIC_SKILLS_INTERNAL_CONFLICTS_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"

namespace intrinsic {

// Checks `left_footprints` against `right_footprints`. Returns pairs of
// indices for conflicts.
//
// Example:
//   left_footprints = {a, b}
//   right_footprints = {x, y}
//   return value = {{0, 1}, {1, 1}}
// means that 'a' conflicts with 'y', and 'b' conflicts with 'y'.
struct ConflictIndices {
  int left_footprint_index;
  int right_footprint_index;
  ConflictIndices(int left, int right);
  bool operator==(const ConflictIndices& other) const;
};
absl::StatusOr<std::vector<ConflictIndices>> CheckConflicts(
    absl::Span<const intrinsic_proto::skills::Footprint> left_footprints,
    absl::Span<const intrinsic_proto::skills::Footprint> right_footprints,
    absl::string_view world_id,
    intrinsic_proto::world::ObjectWorldService::StubInterface&
        object_world_service);

namespace internal {
// This is exposed only for testing. It is already checked within
// CheckConflicts.
std::vector<ConflictIndices> CanExecuteConcurrentlyByEquipment(
    absl::Span<const intrinsic_proto::skills::Footprint> left_footprints,
    absl::Span<const intrinsic_proto::skills::Footprint> right_footprints);
}  // namespace internal
}  // namespace intrinsic

#endif  // INTRINSIC_SKILLS_INTERNAL_CONFLICTS_H_
