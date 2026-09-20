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

#ifndef INTRINSIC_WORLD_COMPONENT_USER_DATA_COMPONENT_H_
#define INTRINSIC_WORLD_COMPONENT_USER_DATA_COMPONENT_H_

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/user_data_component.pb.h"

namespace intrinsic {

// A component to transport blind user data.
class UserDataComponent {
 public:
  virtual ~UserDataComponent() = default;

  // Returns a new UserDataComponent instance.
  static std::unique_ptr<UserDataComponent> Create();

  // Returns a new UserDataComponent instance derived from the given proto.
  static absl::StatusOr<std::unique_ptr<UserDataComponent>> FromProto(
      const intrinsic_proto::world::UserDataComponent& proto);

  // Returns a new UserDataComponent instance that is a copy of this one.
  virtual std::unique_ptr<UserDataComponent> Clone() const = 0;

  // Returns a proto representation of this component.
  virtual absl::StatusOr<intrinsic_proto::world::UserDataComponent> ToProto()
      const = 0;

  virtual WorldHashMap<std::string, std::string>& MutableUserDataMap() = 0;

  virtual const WorldHashMap<std::string, std::string>& UserDataMap() const = 0;

  virtual WorldHashMap<std::string, google::protobuf::Any>&
  MutableUserDataProtos() = 0;

  virtual const WorldHashMap<std::string, google::protobuf::Any>&
  UserDataProtos() const = 0;

  // Updates the component based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  virtual absl::Status UpdateFromProto(
      const intrinsic_proto::world::UserDataComponent& proto) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COMPONENT_USER_DATA_COMPONENT_H_
