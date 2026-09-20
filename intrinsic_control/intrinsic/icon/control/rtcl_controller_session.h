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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_SESSION_H_
#define INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_SESSION_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/fixed_array.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/container/node_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_group.h"
#include "intrinsic/icon/control/initialized_async_buffer.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_signal_types.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/control/rtcl_realtime_session.h"
#include "intrinsic/icon/control/rtcl_session_bridge_interface.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/icon/proto/streaming_output.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/util/thread/thread.h"

namespace intrinsic::icon {

// RtclControllerSession manages the non-realtime components of an ICON Session
// for an RtclController.
//
// In particular, it creates Action and Reaction instances on the heap, and
// passes pointers to them to the realtime thread using an
// RtclSessionBridgeInterface instance.
class RtclControllerSession final : public SessionInterface {
 public:
  // pub_sub may be nullptr, in which case this session will not publish its
  // streaming outputs (if it has any). Holds a reference to
  // `robot_status_buffer`, which must outlive this class.
  // The part_config_map holds the part configs for all parts which are
  // controlled by this session.
  RtclControllerSession(
      const intrinsic_proto::icon::v1::ServerConfig& server_config,
      const absl::flat_hash_map<std::string, size_t>&
          part_name_to_realtime_index,
      SessionId session_id,
      absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::PartConfig>
          part_config_map,
      std::unique_ptr<RtclSessionBridgeInterface> session_bridge,
      const RealtimeLogContext& context,
      InitializedAsyncBuffer<PublishOutput>& robot_status_buffer
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer
          ABSL_ATTRIBUTE_LIFETIME_BOUND);

  ~RtclControllerSession() override;

  // Creates instances of all Actions and Reactions in `actions_and_reactions`,
  // and installs them in the realtime thread. Blocks until all Actions and
  // Reactions have been created and successfully installed. Some Action
  // factories may do complicated setup / planning operations that take multiple
  // seconds to finish.
  //
  // Returns an error if the Session is not healthy.
  // Returns an error if any of the ActionDescription objects is invalid:
  //   * The Action type is unknown
  //   * The ActionInstanceId is already in use
  //   * The SlotPartMap is invalid
  //   * The Action factory fails
  // Returns an error if any of the Reaction protos cannot be converted to
  // RealtimeReaction objects.
  // Returns an error if installing the newly created Action/Reaction objects in
  // the realtime thread fails.
  absl::Status AddActionsAndReactions(
      const ActionsAndReactions& actions_and_reactions) override;

  // 1. Removes the requested Actions and Reactions from this Session.
  //    Implicitly deletes Reactions that refer to any of the deleted Actions.
  // 2. Creates an RtclRealtimeSession instance.
  // 3. Installs that RtclRealtimeSession pointer in the realtime thread.
  // 4. a) On success, deletes the RealtimeAction/RealtimeReaction instances for
  //       the Actions/Reactions that were removed (we're sure that they aren't
  //       used in the realtime thread any more) and returns OkStatus.
  //    b) On failure, returns an error. This effectively ends the Session, so
  //       it will be deleted, taking all RealtimeAction/RealtimeReaction
  //       instances with it. Also notifies the realtime thread to stop using
  //       any previously installed RtclRealtimeSession pointers, and blocks
  //       until the realtime thread has acknowledged this.
  //
  // The realtime thread will attempt to keep running any previously active
  // Action. If the current active Action is deleted, it will fall back to the
  // default Action for each Part.
  absl::Status RemoveActionsAndReactions(
      const ActionAndReactionIds& action_and_reaction_ids) override;

  // Same as above, but deletes all Actions/Reactions.
  absl::Status RemoveAllActionsAndReactions() override;

  // 1. Constructs a new RtclRealtimeSession instance that tells the realtime
  //    thread to start the Actions with IDs `action_instance_ids`.
  // 2. Installs that RtclRealtimeSession.
  //
  // If `stop_active_actions` is true, all currently active actions will be
  // stopped.
  //
  // If `stop_active_actions` is false, the actions with `action_instance_ids`
  // will be started in parallel (active actions with an overlapping part set
  // compared to any action from the actions referenced by `action_instance_ids`
  // will be stopped. Unused parts will switch to the safety action).
  //
  // Does not touch any Action instances.
  //
  // Returns an error if the Session is not healthy, or if `action_instance_id`
  // is invalid.
  absl::Status StartActions(
      absl::Span<const ActionInstanceId> action_instance_ids,
      bool stop_active_actions) override;

  // Polls the RtclSessionBridgeInterface for reactions.
  absl::StatusOr<std::optional<ReactionEvent>> PollReactions() override;

  // Writes `input` to the streaming input `input_name` for
  // `action_instance_id`.
  //
  // Returns an error if the corresponding streaming input does not exist.
  // Returns an error if the streaming input parser for this input fails.
  absl::Status WriteToStreamingInput(
      ActionInstanceId action_instance_id, absl::string_view input_name,
      const google::protobuf::Any& input) override;

  // Returns the latest streaming output value for `action_instance_id`.
  //
  // Returns an error if `action_instance_id` does not have a streaming output.
  // Returns an error if the Action does not publish an initial streaming output
  // value before `deadline`.
  // Returns an error if the streaming output converter for `action_instance_id`
  // fails.
  absl::StatusOr<::intrinsic_proto::icon::StreamingOutput>
  GetLatestStreamingOutput(ActionInstanceId action_instance_id,
                           absl::Time deadline) override;

  absl::StatusOr<::intrinsic_proto::icon::JointTrajectoryPVA>
  GetPlannedTrajectory(ActionInstanceId action_instance_id) override;

  // Returns the PartGroup for this Session. Actions in this Session can only
  // access the Parts in this PartGroup.
  PartGroup GetPartGroup() const override;

  // Returns all ActionInstanceIds for this Session. Does not include the
  // ActionInstanceIds of any deleted Actions.
  absl::flat_hash_set<ActionInstanceId> GetActionInstanceIds() const override;

 private:
  // Used as an absl::Cleanup callback in most member functions, to tell the
  // realtime thread it should end the Session in case we encounter any errors.
  void FinishSession();

  absl::StatusOr<std::reference_wrapper<intrinsic::Publisher>>
  GetOrCreatePublisher(absl::string_view topic);

  mutable absl::Mutex session_mutex_;
  const intrinsic_proto::icon::v1::ServerConfig server_config_;
  const SessionId session_id_;
  const absl::flat_hash_map<std::string, size_t> part_name_to_realtime_index_;
  const absl::flat_hash_map<std::string, intrinsic_proto::icon::v1::PartConfig>
      part_name_to_config_;
  std::unique_ptr<RtclSessionBridgeInterface> session_bridge_
      ABSL_GUARDED_BY(session_mutex_);

  // Storage for RtclActionInstances - RtclControllerSession must not call any
  // of the methods, since they are meant for the realtime thread.
  //
  // The only way RtclControllerSession interacts with this is by adding and
  // removing RtclActionInstances (see above for when each of those happens).
  //
  // We store unique_ptr rather than RtclActionInstance so that we can remove
  // Actions by setting the corresponding unique_ptr to nullptr (in the
  // non-realtime thread). This way, indices remain the same even after deleting
  // Actions, which simplifies the logic both here and in the realtime thread.
  //
  // RtclControllerSession will never call any of the methods on these
  // instances, since all of them are meant to be exclusively invoked in
  // realtime.
  std::vector<std::unique_ptr<RtclActionInstance>> action_instance_arena_
      ABSL_GUARDED_BY(session_mutex_);
  // Helper structure that maps from ActionInstanceId to the index in
  // `action_instance_arena_`. This saves us from repeatedly having to
  // search through the list of Actions.
  absl::flat_hash_map<ActionInstanceId, size_t> action_id_to_index_
      ABSL_GUARDED_BY(session_mutex_);
  // Stores the name mapping from action_id to the type_name of that action.
  absl::flat_hash_map<ActionInstanceId, std::string> action_id_to_type_name_
      ABSL_GUARDED_BY(session_mutex_);

  // These two sets are used to ensure uniqueness of Action and Reaction IDs for
  // the duration of the Session.
  absl::flat_hash_set<ActionInstanceId> known_action_ids_
      ABSL_GUARDED_BY(session_mutex_);
  absl::flat_hash_set<ReactionId> known_reaction_ids_
      ABSL_GUARDED_BY(session_mutex_);

  // Holds the RealtimeReaction objects. In contrast to RtclActionInstances,
  // RealtimeReactions are pure data, so they're cheap and easy to copy, and
  // their indices do not matter, so we store them in a map and copy them when
  // we create an RtclRealtimeSession to install in the realtime thread.
  absl::flat_hash_map<ReactionId, RealtimeReaction> reaction_instances_by_id_
      ABSL_GUARDED_BY(session_mutex_);
  // StreamingIoStorage instances for each active RtclActionInstance - we use
  // these to interact with an Action's streaming inputs and output.
  absl::flat_hash_map<ActionInstanceId, std::unique_ptr<StreamingIoStorage>>
      streaming_io_storage_instances_ ABSL_GUARDED_BY(session_mutex_);
  // Maps from signal names to RealtimeSignalIds for each Action. We use these
  // to map the signal names used in Reactions to the correct ID for the
  // realtime thread to use.
  absl::flat_hash_map<ActionInstanceId,
                      absl::flat_hash_map<std::string, RealtimeSignalId>>
      action_id_to_realtime_signal_map_ ABSL_GUARDED_BY(session_mutex_);

  // At any point, this holds the RtclRealtimeSession that was last handed to
  // session_bridge_. RtclRealtimeSession holds pointers to RtclActionInstances
  // and RealtimeReactions. That's why we declare it after the containers
  // holding those, so destruction happens in the correct order:
  // RtclRealtimeSession is destroyed first, before any of the pointers it holds
  // become invalid.
  std::unique_ptr<RtclRealtimeSession> installed_realtime_session_
      ABSL_GUARDED_BY(session_mutex_);

  absl::flat_hash_map<ActionInstanceId, JointTrajectoryPVA> trajectory_map_
      ABSL_GUARDED_BY(session_mutex_);

  // Variables to allow this session to publish output for all its actions.
  absl::Mutex publishers_mutex_ ABSL_ACQUIRED_AFTER(session_mutex_);
  PubSub pub_sub_;
  absl::node_hash_map<std::string, intrinsic::Publisher> publishers_
      ABSL_GUARDED_BY(publishers_mutex_);
  absl::Notification shutdown_notification_;
  Thread publishing_thread_;
  InitializedAsyncBuffer<PublishOutput>& robot_status_buffer_;
  InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer_;
  RealtimeLogContext log_context_;
  void PublishOutputStreams();
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_SESSION_H_
