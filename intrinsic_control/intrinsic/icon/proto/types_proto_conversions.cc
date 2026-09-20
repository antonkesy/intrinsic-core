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

#include "intrinsic/icon/proto/types_proto_conversions.h"

#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/slot_part_map.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/session_interface.h"

namespace intrinsic {
namespace icon {

absl::StatusOr<ActionsAndReactions> ActionsAndReactionsFromProto(
    const intrinsic_proto::icon::v1::ActionsAndReactions&
        actions_and_reactions) {
  std::vector<ActionDescription> action_descriptions;
  for (const intrinsic_proto::icon::v1::ActionInstance& action_instance :
       actions_and_reactions.action_instances()) {
    auto slot_part_map = SlotPartMapFromProto(action_instance.slot_part_map());
    action_descriptions.push_back(
        {.id = ActionInstanceId(action_instance.action_instance_id()),
         .slot_part_map = std::move(slot_part_map),
         .action_type_name = action_instance.action_type_name(),
         .params = action_instance.fixed_parameters()});
  }

  return ActionsAndReactions{
      .action_descriptions = std::move(action_descriptions),
      .reaction_descriptions = std::vector<intrinsic_proto::icon::v1::Reaction>(
          actions_and_reactions.reactions().begin(),
          actions_and_reactions.reactions().end())};
}

ActionAndReactionIds ActionAndReactionIdsFromProto(
    const intrinsic_proto::icon::v1::ActionAndReactionIds&
        action_and_reaction_ids) {
  return {.action_ids = std::vector<ActionInstanceId>(
              action_and_reaction_ids.action_instance_ids().begin(),
              action_and_reaction_ids.action_instance_ids().end()),
          .reaction_ids = std::vector<ReactionId>(
              action_and_reaction_ids.reaction_ids().begin(),
              action_and_reaction_ids.reaction_ids().end())};
}

intrinsic_proto::icon::v1::ReactionEvent ReactionEventToProto(
    const ReactionEvent& reaction_event) {
  intrinsic_proto::icon::v1::ReactionEvent reaction_event_proto;
  reaction_event_proto.set_reaction_id(reaction_event.id.value());
  if (reaction_event.previous_action_id.has_value()) {
    reaction_event_proto.set_previous_action_instance_id(
        reaction_event.previous_action_id->value());
  }
  if (reaction_event.current_action_id.has_value()) {
    reaction_event_proto.set_current_action_instance_id(
        reaction_event.current_action_id->value());
  }
  return reaction_event_proto;
}

}  // namespace icon
}  // namespace intrinsic
