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

#ifndef INTRINSIC_WORLD_WORLD_ENTITY_ACL_SPEC_H_
#define INTRINSIC_WORLD_WORLD_ENTITY_ACL_SPEC_H_

#include <ostream>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/proto/acl_spec.pb.h"

namespace intrinsic {

// Represents access permissions/restrictions for enforcement on a WorldEntity
// type.
class WorldEntityACLSpec {
 public:
  explicit WorldEntityACLSpec(bool can_read, bool can_write);

  // Returns a specification that provides full access and no restrictions.
  static const WorldEntityACLSpec& FullAccess();

  // Returns a specification that provides read only access.
  static const WorldEntityACLSpec& ReadOnlyAccess();

  // Returns a specification that provides no access to the data.
  static const WorldEntityACLSpec& NoAccess();

  bool CanReadEntityData() const;
  bool CanWriteEntityData() const;

  // Returns a spec that is equivalent to using a logical AND on the underlying
  // access permissions.
  WorldEntityACLSpec AndWith(const WorldEntityACLSpec& other) const;

  // Returns a spec that is equivalent to using a logical OR on the underlying
  // access permissions.
  WorldEntityACLSpec OrWith(const WorldEntityACLSpec& other) const;

  // Returns true if the current and given spec are equivalent.
  bool operator==(const WorldEntityACLSpec& other) const;

  // Returns true if the current and given spec are not equivalent.
  bool operator!=(const WorldEntityACLSpec& other) const;

  // Returns Ok if this spec is actually compatible with the given narrower
  // spec. Compatible in this case means that the narrow spec gives less
  // permissions than the current spec and does not provide any specs that are
  // not present in the existing one.
  absl::Status CanNarrowTo(const WorldEntityACLSpec& narrow_spec) const;

  // Returns true if the given spec does not conflict with this spec. For
  // example only one spec can have a write permission while they can both share
  // a read permission.
  bool IsCompatible(const WorldEntityACLSpec& other) const;

  // Returns a human readable string representation of this spec.
  std::string ToString() const;

  // Returns a proto version of this world entity acl spec instance.
  intrinsic_proto::world::WorldEntityACLSpec ToProto() const;

  // Returns an instance of a WorldEntityACLSpec parsed rom the given proto.
  static absl::StatusOr<WorldEntityACLSpec> FromProto(
      const intrinsic_proto::world::WorldEntityACLSpec& proto_data);

 private:
  bool can_read_ = true;
  bool can_write_ = true;
};

// Prints WorldEntityACLSpec to an ostream in a human readable version.
std::ostream& operator<<(std::ostream& os, const WorldEntityACLSpec& acls);

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_WORLD_ENTITY_ACL_SPEC_H_
