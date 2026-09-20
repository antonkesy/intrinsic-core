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

#include "intrinsic/world/world_entity_acl_spec.h"

#include <ostream>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/world/proto/acl_spec.pb.h"

namespace intrinsic {

WorldEntityACLSpec::WorldEntityACLSpec(bool can_read, bool can_write)
    : can_read_(can_read), can_write_(can_write) {}

const WorldEntityACLSpec& WorldEntityACLSpec::FullAccess() {
  static auto* kFullAccess = new WorldEntityACLSpec(true, true);
  return *kFullAccess;
}

const WorldEntityACLSpec& WorldEntityACLSpec::NoAccess() {
  static auto* kNoAccess = new WorldEntityACLSpec(false, false);
  return *kNoAccess;
}

const WorldEntityACLSpec& WorldEntityACLSpec::ReadOnlyAccess() {
  static auto* kReadOnlyAccess = new WorldEntityACLSpec(true, false);
  return *kReadOnlyAccess;
}

bool WorldEntityACLSpec::CanReadEntityData() const { return can_read_; }
bool WorldEntityACLSpec::CanWriteEntityData() const { return can_write_; }

WorldEntityACLSpec WorldEntityACLSpec::AndWith(
    const WorldEntityACLSpec& other) const {
  return WorldEntityACLSpec(can_read_ && other.can_read_,
                            can_write_ && other.can_write_);
}

WorldEntityACLSpec WorldEntityACLSpec::OrWith(
    const WorldEntityACLSpec& other) const {
  return WorldEntityACLSpec(can_read_ || other.can_read_,
                            can_write_ || other.can_write_);
}

bool WorldEntityACLSpec::operator==(const WorldEntityACLSpec& other) const {
  return can_read_ == other.can_read_ && can_write_ == other.can_write_;
}

bool WorldEntityACLSpec::operator!=(const WorldEntityACLSpec& other) const {
  return !(*this == other);
}

absl::Status WorldEntityACLSpec::CanNarrowTo(
    const WorldEntityACLSpec& narrow_spec) const {
  if (!can_write_ && narrow_spec.CanWriteEntityData()) {
    return absl::PermissionDeniedError(
        "Cannot provide write access to the entity when it does not currently "
        "have write access.");
  } else if (!can_read_ && narrow_spec.CanReadEntityData()) {
    return absl::PermissionDeniedError(
        "Cannot provide read access to the entity when it does not currently "
        "have read access.");
  }

  return absl::OkStatus();
}

bool WorldEntityACLSpec::IsCompatible(const WorldEntityACLSpec& other) const {
  if (can_write_ && (other.CanWriteEntityData() || other.CanReadEntityData())) {
    // If we can write the entity then no one else is allowed to read or
    // write at the same time.
    return false;
  }

  if (can_read_ && other.CanWriteEntityData()) {
    // If we want to read the entity then the other spec is not allowed to
    // write to it, but can read from it as well.
    return false;
  }

  // If there were no conflicts detected we are compatible.
  return true;
}

std::string WorldEntityACLSpec::ToString() const {
  return absl::StrCat("(r:", can_read_ ? "true" : "false",
                      ", w:", can_write_ ? "true" : "false", ")");
}

intrinsic_proto::world::WorldEntityACLSpec WorldEntityACLSpec::ToProto() const {
  intrinsic_proto::world::WorldEntityACLSpec result;
  result.set_can_read_entity_data(can_read_);
  result.set_can_write_entity_data(can_write_);
  return result;
}

absl::StatusOr<WorldEntityACLSpec> WorldEntityACLSpec::FromProto(
    const intrinsic_proto::world::WorldEntityACLSpec& proto_data) {
  return WorldEntityACLSpec(proto_data.can_read_entity_data(),
                            proto_data.can_write_entity_data());
}

std::ostream& operator<<(std::ostream& os, const WorldEntityACLSpec& acls) {
  return os << acls.ToString();
}

}  // namespace intrinsic
