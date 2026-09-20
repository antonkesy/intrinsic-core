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

#include "intrinsic/skills/internal/conflicts.h"

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"
#include "grpcpp/client_context.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"
#include "intrinsic/world/proto/object_world_service.pb.h"

namespace intrinsic {
namespace {

using intrinsic_proto::skills::ResourceReservation;
using EquipmentSharingType =
    intrinsic_proto::skills::ResourceReservation::SharingType;
using EquipmentUsage = absl::flat_hash_map<
    std::string, intrinsic_proto::skills::ResourceReservation::SharingType>;

absl::StatusOr<bool> Writes(EquipmentSharingType sharing_type) {
  switch (sharing_type) {
    case ResourceReservation::UNSPECIFIED:
    case ResourceReservation::WRITE:
    case ResourceReservation::WRITE_EXCLUSIVE_READ_NONEXCLUSIVE:
    case ResourceReservation::WRITE_NONEXCLUSIVE:
      return true;
    case ResourceReservation::READ:
      return false;
    default:
      return absl::InternalError(
          absl::StrCat("Unsupported type: ",
                       ResourceReservation::SharingType_Name(sharing_type)));
  }
}

absl::StatusOr<bool> WritesExclusive(EquipmentSharingType sharing_type) {
  switch (sharing_type) {
    case ResourceReservation::WRITE:
    case ResourceReservation::WRITE_EXCLUSIVE_READ_NONEXCLUSIVE:
      return true;
    case ResourceReservation::UNSPECIFIED:
    case ResourceReservation::READ:
    case ResourceReservation::WRITE_NONEXCLUSIVE:
      return false;
    default:
      return absl::InternalError(
          absl::StrCat("Unsupported type: ",
                       ResourceReservation::SharingType_Name(sharing_type)));
  }
}

// Returns OK if two different skills can execute currently on the same
// hardware given each skill's proposed sharing type. Otherwise returns a status
// with an explanation.
absl::Status CanExecuteConcurrently(EquipmentSharingType left,
                                    EquipmentSharingType right) {
  auto error_message_fn = [left, right]() {
    return absl::InvalidArgumentError(
        absl::StrCat(ResourceReservation::SharingType_Name(left),
                     " cannot execute concurrently with ",
                     ResourceReservation::SharingType_Name(right), "."));
  };

  // Test whether either skill wants exclusive access to the resource.
  if (left == ResourceReservation::WRITE ||
      right == ResourceReservation::WRITE) {
    return error_message_fn();
  }

  // Test whether either skill wants to write exclusively while the other wants
  // to write.
  INTR_ASSIGN_OR_RETURN(bool left_writes, Writes(left));
  INTR_ASSIGN_OR_RETURN(bool right_writes, Writes(right));
  INTR_ASSIGN_OR_RETURN(bool left_writes_exclusive, WritesExclusive(left));
  INTR_ASSIGN_OR_RETURN(bool right_writes_exclusive, WritesExclusive(right));
  if ((left_writes_exclusive && right_writes) ||
      (right_writes_exclusive && left_writes)) {
    return error_message_fn();
  }

  return absl::OkStatus();
}

absl::Status CanExecuteConcurrently(
    const google::protobuf::RepeatedPtrField<ResourceReservation>& left,
    const google::protobuf::RepeatedPtrField<ResourceReservation>& right) {
  for (const auto& left_resource : left) {
    for (const auto& right_resource : right) {
      if (left_resource.name() == right_resource.name()) {
        INTR_RETURN_IF_ERROR(CanExecuteConcurrently(left_resource.type(),
                                                    right_resource.type()));
      }
    }
  }
  return absl::OkStatus();
}

}  // namespace

namespace internal {
std::vector<ConflictIndices> CanExecuteConcurrentlyByEquipment(
    absl::Span<const intrinsic_proto::skills::Footprint> left_footprints,
    absl::Span<const intrinsic_proto::skills::Footprint> right_footprints) {
  std::vector<ConflictIndices> conflicts;
  for (int left_index = 0; left_index < left_footprints.size(); left_index++) {
    for (int right_index = 0; right_index < right_footprints.size();
         right_index++) {
      auto can_execute_concurrently = CanExecuteConcurrently(
          left_footprints.at(left_index).resource_reservation(),
          right_footprints.at(right_index).resource_reservation());
      if (!can_execute_concurrently.ok()) {
        conflicts.push_back(ConflictIndices(left_index, right_index));
      }
    }
  }
  return conflicts;
}
}  // namespace internal

ConflictIndices::ConflictIndices(int left, int right)
    : left_footprint_index(left), right_footprint_index(right) {}

bool ConflictIndices::operator==(const ConflictIndices& other) const {
  return this->left_footprint_index == other.left_footprint_index &&
         this->right_footprint_index == other.right_footprint_index;
}

absl::StatusOr<std::vector<ConflictIndices>> CheckConflicts(
    absl::Span<const intrinsic_proto::skills::Footprint> left_footprints,
    absl::Span<const intrinsic_proto::skills::Footprint> right_footprints,
    absl::string_view world_id,
    intrinsic_proto::world::ObjectWorldService::StubInterface&
        object_world_service) {
  std::vector<ConflictIndices> conflicts;
  if (left_footprints.empty() || right_footprints.empty()) {
    return conflicts;
  }

  intrinsic_proto::world::AreFootprintsCompatibleRequest request;
  request.set_world_id(world_id);
  request.set_return_all_incompatible_pairs(true);

  for (const auto& footprint : left_footprints) {
    *request.add_left_set() = footprint;
  }
  for (const auto& footprint : right_footprints) {
    *request.add_right_set() = footprint;
  }

  grpc::ClientContext context;
  intrinsic::ConfigureClientContext(&context);
  intrinsic_proto::world::AreFootprintsCompatibleResponse response;
  INTR_RETURN_IF_ERROR(
      ToAbslStatus(object_world_service.AreFootprintsCompatible(
          &context, request, &response)));

  conflicts = internal::CanExecuteConcurrentlyByEquipment(left_footprints,
                                                          right_footprints);
  conflicts.reserve(conflicts.size() + response.pairs_size());
  for (const auto& conflict_pair : response.pairs()) {
    conflicts.push_back(ConflictIndices(conflict_pair.left_index(),
                                        conflict_pair.right_index()));
  }
  return conflicts;
}

}  // namespace intrinsic
