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

#include "intrinsic/world/objects/user_data_conversion.h"

#include "absl/strings/str_cat.h"

namespace intrinsic {
namespace object_world {

using WorldUpdateUserData = intrinsic_proto::world::UpdateUserData;
using SOUpdateUserData = intrinsic_proto::scene_object::v1::UpdateUserData;

absl::StatusOr<
    intrinsic_proto::scene_object::v1::UpdateUserData::UpdateUserDataPolicy>
ToSceneObjectUserDataPolicy(
    const intrinsic_proto::world::UpdateUserData::UpdateUserDataPolicy policy) {
  switch (policy) {
    case WorldUpdateUserData::POLICY_UNSPECIFIED:
      return SOUpdateUserData::POLICY_UNSPECIFIED;

    case WorldUpdateUserData::POLICY_INSERT:
      return SOUpdateUserData::POLICY_INSERT;

    case WorldUpdateUserData::POLICY_INSERT_OR_UPDATE:
      return SOUpdateUserData::POLICY_INSERT_OR_UPDATE;

    case WorldUpdateUserData::POLICY_REMOVE:
      return SOUpdateUserData::POLICY_REMOVE;

    case WorldUpdateUserData::POLICY_CLEAR_AND_REPLACE:
      return SOUpdateUserData::POLICY_CLEAR_AND_REPLACE;

    case WorldUpdateUserData::POLICY_CLEAR_ALL:
      return SOUpdateUserData::POLICY_CLEAR_ALL;
    default:
      break;
  }

  return absl::InvalidArgumentError(absl::StrCat("Invalid policy: ", policy));
}

absl::StatusOr<intrinsic_proto::world::UpdateUserData::UpdateUserDataPolicy>
FromSceneObjectUserDataPolicy(const intrinsic_proto::scene_object::v1::
                                  UpdateUserData::UpdateUserDataPolicy policy) {
  switch (policy) {
    case SOUpdateUserData::POLICY_UNSPECIFIED:
      return WorldUpdateUserData::POLICY_UNSPECIFIED;

    case SOUpdateUserData::POLICY_INSERT:
      return WorldUpdateUserData::POLICY_INSERT;

    case SOUpdateUserData::POLICY_INSERT_OR_UPDATE:
      return WorldUpdateUserData::POLICY_INSERT_OR_UPDATE;

    case SOUpdateUserData::POLICY_REMOVE:
      return WorldUpdateUserData::POLICY_REMOVE;

    case SOUpdateUserData::POLICY_CLEAR_AND_REPLACE:
      return WorldUpdateUserData::POLICY_CLEAR_AND_REPLACE;

    case SOUpdateUserData::POLICY_CLEAR_ALL:
      return WorldUpdateUserData::POLICY_CLEAR_ALL;
    default:
      break;
  }

  return absl::InvalidArgumentError(absl::StrCat("Invalid policy: ", policy));
}

}  // namespace object_world
}  // namespace intrinsic
