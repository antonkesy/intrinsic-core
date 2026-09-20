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

#ifndef INTRINSIC_ICON_SERVER_SESSION_INTERFACE_H_
#define INTRINSIC_ICON_SERVER_SESSION_INTERFACE_H_

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_group.h"
#include "intrinsic/icon/common/slot_part_map.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/icon/proto/streaming_output.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic {
namespace icon {

// A description of the reaction that has occurred.
struct ReactionEvent {
  ReactionId id;
  // Action that this reaction is bound to.
  // Not set if the reaction is not bound to an action.
  // TODO (b/278083799): Rename this to better reflect the new meaning of the
  // variable.
  std::optional<ActionInstanceId> previous_action_id;
  // Action that was started by reaction.
  // Not set if the reaction did not trigger an action change.
  // TODO (b/278083799): Rename this to better reflect the new meaning of the
  // variable.
  std::optional<ActionInstanceId> current_action_id;
};

// A description of an action, including all of the information needed to create
// one.
struct ActionDescription {
  ActionInstanceId id;
  // A mapping from slot name to part name. This indicates which Part(s) an
  // Action is using.
  SlotPartMap slot_part_map;
  std::string action_type_name;
  google::protobuf::Any params;
};

// A description of actions and reactions.
struct ActionsAndReactions {
  std::vector<ActionDescription> action_descriptions;
  std::vector<intrinsic_proto::icon::v1::Reaction> reaction_descriptions;
};

// The Ids of actions and reactions.
struct ActionAndReactionIds {
  std::vector<ActionInstanceId> action_ids;
  std::vector<ReactionId> reaction_ids;
};

// SessionInterface implementations provide a mechanism for the user to
// to modify the action state machine, which is being evaluated cyclically,
// on-the-fly. The allowable modifications include altering the reactions
// that are being evaluated during operation of the state machine, the set of
// actions available in the state machine, etc. Each instance of an action
// operator operates on a given set of parts, and must ensure that added actions
// only affect the parts that are allocated to the action operator.
//
// Each individual mutating method invocation must be implemented to take effect
// atomically. For example, a single invocation of
// AddActionsAndReactions() must guarantee that all reactions
// and actions are added before any are evaluated by the state machine.
class SessionInterface {
 public:
  virtual ~SessionInterface() = default;

  // Adds a collection of reactions and actions to the state machine that
  // is being cyclically evaluated. Returns an aborted error if the session
  // should end. May return other errors if an error occurs, but the session
  // should not end.
  virtual absl::Status AddActionsAndReactions(
      const ActionsAndReactions& actions_and_reactions) = 0;

  // Removes the collection of reactions and action instances from the
  // state machine that is being cyclically evaluated.  Returns an aborted error
  // if the session should end. May return other errors if an error occurs, but
  // the session should not end.
  virtual absl::Status RemoveActionsAndReactions(
      const ActionAndReactionIds& action_and_reaction_ids) = 0;

  // Removes all actions and reactions that have been added.  Returns an aborted
  // error if the session should end. May return other errors if an error
  // occurs, but the session should not end.
  virtual absl::Status RemoveAllActionsAndReactions() = 0;

  // Requests that the actions start as soon as possible, regardless of existing
  // reactions. `stop_active_actions` indicates whether active actions should
  // continue to run or should be stopped.
  //
  // Returns an aborted error if the session should end. May return other errors
  // if an error occurs, but the session should not end.
  virtual absl::Status StartActions(
      absl::Span<const ActionInstanceId> action_instance_ids,
      bool stop_active_actions) = 0;
  // Returns a Reaction if one has occurred since the last invocation of
  // PollReactions(). Non-blocking. Can be called concurrently with
  // AddActionsAndReactions(), RemoveActionsAndReactions(),
  // RemoveAllActionsAndReactions(), StartAction(), and WriteToStreamingInput().
  // Returns various descriptive errors when the session has failed. Session
  // failures are only recoverable by creating a new session.
  virtual absl::StatusOr<std::optional<ReactionEvent>> PollReactions() = 0;

  // Writes `value` to a streaming input of the action with name `input_name`.
  // Caution: the implementation of the action may not immediately consume the
  // value.
  //
  // Can be called concurrently with AddActionsAndReactions(),
  // RemoveActionsAndReactions(), RemoveAllActionsAndReactions(), StartAction(),
  // and PollReactions().
  //
  // Returns an error if the input is unknown by the action, the value is
  // rejected by the action, or internal errors occur.
  virtual absl::Status WriteToStreamingInput(
      ActionInstanceId action_instance_id, absl::string_view input_name,
      const google::protobuf::Any& value) = 0;

  // If there is an output writer for `action_instance_id`, blocks until the
  // first piece of data (after the Action has become active) is written.
  // Returns immediately if any data is available, even if that data has been
  // returned already (check the timestamp!).
  //
  // Returns the latest value of the streaming output for `action_instance_id`.
  // Returns an error if `action_instance_id` is invalid, the corresponding
  // Action is not active, or the Action does not provide a streaming output.
  virtual absl::StatusOr<::intrinsic_proto::icon::StreamingOutput>
  GetLatestStreamingOutput(ActionInstanceId action_instance_id,
                           absl::Time deadline) = 0;

  // If there is a planned trajectory for Action with `action_instance_id`,
  // returns the planned trajectory. Returns `kFailedPrecondition` if there is
  // there is no trajectory for the given `action_instance_id`.
  virtual absl::StatusOr<::intrinsic_proto::icon::JointTrajectoryPVA>
  GetPlannedTrajectory(ActionInstanceId action_instance_id) = 0;

  // Returns the Parts that this Session is using.
  virtual PartGroup GetPartGroup() const = 0;

  // Returns the ActionInstanceIds of all Actions that are currently known to
  // the Session (i.e. those that have been added, but not yet removed).
  virtual absl::flat_hash_set<ActionInstanceId> GetActionInstanceIds()
      const = 0;
};

// Creates an SessionInterface for managing a session with access to the
// parts in `part_names`.
using CreateSession =
    std::function<absl::StatusOr<std::unique_ptr<SessionInterface>>(
        const absl::flat_hash_set<std::string>& part_names,
        SessionId session_id)>;

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_SERVER_SESSION_INTERFACE_H_
