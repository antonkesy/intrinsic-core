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

#ifndef INTRINSIC_ICON_PROTO_TYPES_PROTO_CONVERSIONS_H_
#define INTRINSIC_ICON_PROTO_TYPES_PROTO_CONVERSIONS_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/session_interface.h"

namespace intrinsic {
namespace icon {

absl::StatusOr<ActionsAndReactions> ActionsAndReactionsFromProto(
    const intrinsic_proto::icon::v1::ActionsAndReactions&
        actions_and_reactions);

// Converts a intrinsic_proto::ActionCollectionIds to ActionAndReactionIds.
ActionAndReactionIds ActionAndReactionIdsFromProto(
    const intrinsic_proto::icon::v1::ActionAndReactionIds&
        action_and_reaction_ids);

// Converts a ReactionEvent to a proto::ReactionEvent.
intrinsic_proto::icon::v1::ReactionEvent ReactionEventToProto(
    const ReactionEvent& reaction_event);

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_PROTO_TYPES_PROTO_CONVERSIONS_H_
