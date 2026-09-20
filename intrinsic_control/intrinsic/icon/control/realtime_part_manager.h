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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_PART_MANAGER_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_PART_MANAGER_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include "absl/container/fixed_array.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/control/initialized_async_buffer.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/part_property_registry.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/parts/realtime_part.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/realtime_state_manager_interface.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/control/rtcl_initialization.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/platform/common/buffers/realtime_write_queue.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/invalid_until_set.h"

namespace intrinsic::icon {

// RealtimePartManager owns all Parts for an RtclController, and provides an
// abstracted API to use them in cyclic realtime. In particular, this class
// collects the realtime status (including Part Property values) from Parts and
// runs the safety Action for each Part if necessary.
class RealtimePartManager {
 private:
  struct PartInfo {
    RealtimePart part;
    // RealtimePartManager must hold, but not modify, this pointer, so that the
    // *realtime* streaming IO storage instance that `safety_action` uses can
    // refer to it.
    std::unique_ptr<const StreamingIoStorage> safety_action_io_storage;
    RtclActionInstance safety_action;
    std::optional<ActionInstanceId> controlled_by = std::nullopt;
    bool safety_action_active = false;
    RealtimeLogContext context;
  };

 public:
  struct AllPartProperties {
    // The control timestamp at the moment that `properties` below was written.
    // This is not a wall clock, but the time since the ICON server was started.
    // As such, it's not an absl::Time (which should always refer to an actual
    // time/date), and can advance at a slower or faster pace than
    // `timestamp_wall_`, for example when running in sim.
    absl::Duration timestamp_control;
    // The wall time at the moment that `properties` below was written. Useful
    // for finding and ordering log data.
    absl::Time timestamp_wall;

    // The actual property values. Each entry of the outer FixedArray
    // corresponds to a part, always in the same part index order used
    // throughout RealtimePartManager.
    //
    // The inner FixedArray contains the values of all part properties for the
    // corresponding part. Use `PartPropertyToIndexMap()` to find the index for
    // a given part and property name.
    FixedVector<absl::FixedArray<PartPropertyValue>, kMaxRealtimeParts>
        properties;
  };

  class SlotMap final : public RealtimeSlotMapInterface {
   public:
    SlotMap(const absl::FixedArray<bool>* part_visibility_by_index,
            intrinsic::FixedVector<PartInfo, kMaxRealtimeParts>& part_infos);

    FeatureInterfaceRegistry* GetMutableRegistryForSlot(
        RealtimeSlotId slot_id) override INTRINSIC_CHECK_REALTIME_SAFE;

    const FeatureInterfaceRegistry* GetRegistryForSlot(
        RealtimeSlotId slot_id) const override INTRINSIC_CHECK_REALTIME_SAFE;

   private:
    const absl::FixedArray<bool>& part_visibility_by_index_;
    intrinsic::FixedVector<PartInfo, kMaxRealtimeParts>& part_infos_;
  };

  // Consumes `parts_and_safety_actions`.
  explicit RealtimePartManager(
      intrinsic::FixedVector<PartAndSafetyAction, kMaxRealtimeParts>
          parts_and_safety_actions,
      const intrinsic::FixedVector<std::vector<PartPropertyInitialData>,
                                   kMaxRealtimeParts>&
          part_property_data_in_index_order);

  // Returns a RealtimeSlotMapInterface instance that only allows access to
  // those Parts marked as visible in `part_visibility_by_index`. Indices in
  // `part_visibility_by_index` must map 1:1 to the indices of realtime Parts
  // (same order as in the `parts_and_safety_actions` parameter to
  // RealtimePartManager's ctor above),
  //
  // The return value must not outlive either `part_visibility_by_index` or this
  // `RealtimePartManager`!
  SlotMap GetSlotMapForAction(
      const absl::FixedArray<bool>* part_visibility_by_index)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Begins an ICON cycle (as far as Parts are concerned), by
  //
  // * Handling ongoing OperationalStateCommand
  //   (Enable/Disable/ClearFault)
  // * Resetting per-cycle accounting, such as which Parts have had an Action
  //   run on them this cycle
  // * Calling ReadStatus() on each Part
  // * Pushing the current status of each Part into
  //   * A queue (see `PartStatusReader()` below)
  //   * An InitializedAsyncBuffer (see `CurrentRobotStatusBuffer()` below)
  //
  // Returns the current robot status if no errors occurred.
  //
  // WARNING: A non-OK return value should be treated as a
  // non-recoverable error. At most, call FinishCycle() to attempt to cleanly
  // shut down Parts.
  RealtimeStatusOr<AggregatedRobotStatus> BeginCycle(
      intrinsic::Time cycle_start) INTRINSIC_CHECK_REALTIME_SAFE;

  // Marks the Part at `part_index` as controlled by the Action `action_id`.
  //
  // Must be called *before* calling the Control() method of the current Action.
  //
  // When FinishCycle() is called, RealtimePartManager runs the safety Action
  // for any Parts that *haven't* been marked as controlled by any Action. In
  // case of a safety event, RealtimePartManager may instead run the Part's
  // safety action, regardless of whether the part is controlled by any other
  // Action or running its safety Action.
  //
  // Returns an error if called more than once with the same `part_index` (per
  // cycle).
  // Returns an error if the Part at `part_index` is running its safety Action.
  RealtimeStatus MarkPartAsControlledByAction(size_t part_index,
                                              ActionInstanceId action_id)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Unmarks the Part at `part_index` as controlled by any Action.
  //
  // When FinishCycle() is called, RealtimePartManager runs the safety Action
  // for any Parts that are not marked as controlled by any Action.
  //
  // The state of the safety Action is not affected, i.e. if the safety Action
  // is running, it will continue to run. If not, it will be started.
  //
  // If the same action is marked and unmarked in the same cycle,
  // RealtimePartManager behaves as if its status had not changed at all. If the
  // safety action is already running, then RealtimePartManager does not start
  // it anew.
  RealtimeStatus UnmarkPartAsControlledByAction(size_t part_index)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Sets the log context for one part, defined by its `part_index`.
  // The part status for this part will be tagged with this context.
  // Set to empty context to reset the parts context.
  RealtimeStatus SetPartContext(size_t part_index,
                                const RealtimeLogContext& context)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Finishes the cycle:
  //
  // * Invokes the safety Action for any Parts that are Enabled, but not
  //   controlled by an Action this cycle.
  // * Calls ApplyCommand() on each Part.
  // * Uses `state_manager` to pause state transitions when safety actions are
  //   ongoing.
  //
  // Returns an error if any of the safety Actions fail, or if ApplyCommand()
  // fails for any Part.
  //
  // WARNING: Any non-OK return value from this should be treated as a
  // non-recoverable error.
  RealtimeStatus FinishCycle(RealtimeStateManagerInterface& state_manager)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns a reference to a reader for the Robot Status queue. Use this to
  // publish a continuous stream of Robot Status values as they become
  // available, but be careful to only have a single consumer (or not everyone
  // will get every status)! Multiple consumers should use
  // CurrentRobotStatusBuffer() below.
  RealtimeWriteQueue<PublishOutput>::NonRtReader& RobotStatusReader()
      INTRINSIC_NON_REALTIME_ONLY;

  // Returns a reference to an InitializedAsyncBuffer with the latest Robot
  // Status. The value of the buffer changes on every call of BeginCycle().
  // Use this for multiple consumers (reading from the buffer does not remove
  // the current value).
  InitializedAsyncBuffer<PublishOutput>& CurrentRobotStatusBuffer()
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns an AsyncBuffer for part properties reported from the realtime
  // thread.
  AsyncBuffer<AllPartProperties>& PartPropertiesRtToNonRtBuffer()
      INTRINSIC_NON_REALTIME_ONLY;

  // Returns an AsyncBuffer for part properties that the realtime thread reads
  // from. Write newly received part property values into this and commit, and
  // the realtime thread will apply the values at the next opportunity.
  // Each entry of the outer FixedArray corresponds to a part, always in the
  // same part index order used throughout this class.
  //
  // The inner FixedArray contains the values of all part properties for the
  // corresponding part. Use `PartPropertyToIndexMap()` to find the index for a
  // given part and property name.
  AsyncBuffer<AllPartProperties>& PartPropertiesNonRtToRtBuffer()
      INTRINSIC_NON_REALTIME_ONLY;

  // Returns an AsyncBuffer for part operational states reported from the
  // realtime thread.
  AsyncBuffer<FixedVector<OperationalState, kMaxRealtimeParts>>&
  PartStatesBuffer() INTRINSIC_NON_REALTIME_ONLY {
    return part_states_buffer_;
  }

  // Returns the operational status of a part. Returns OutOfRange when the part
  // at `part_index` doesn't exist. Forwards errors from the
  // RealtimePartInterface implementation.
  RealtimeStatusOr<RealtimeOperationalStatus> GetPartState(
      size_t part_index) const INTRINSIC_CHECK_REALTIME_SAFE;

  size_t GetNumParts() const INTRINSIC_CHECK_REALTIME_SAFE;

  std::optional<absl::string_view> GetPartName(size_t part_index) const
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns the group of hardware modules that `part_index` depends on, e.g.
  // the group of operational or cell control hardware modules.
  RealtimeStatusOr<HardwareGroupSet> GetHardwareDependencies(
      size_t part_index) const INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns an error if any Part is currently running its safety action, and
  // that safety action is not done yet.
  RealtimeStatus CheckUnfinishedSafetyAction(size_t part_index) const
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Sets the SafetyStatus that is exported to the non-realtime system.
  // Converts the flatbuffer SafetyStatusMessage message to a message that can
  // be passed into non-realtime using the part_status_queue_.
  void SetSafetyStatus(const intrinsic_fbs::SafetyStatusMessage& safety_status)
      INTRINSIC_CHECK_REALTIME_SAFE;

 private:
  static intrinsic::FixedVector<PartInfo, kMaxRealtimeParts> MakePartInfoArray(
      intrinsic::FixedVector<PartAndSafetyAction, kMaxRealtimeParts>);

  // Evaluates whether to run the part's safety action and updates `part_info`
  // accordingly. Forwards any errors from the safety action's OnEnter()
  // method.
  RealtimeStatus EnterSafetyActionIfRequired(const RealtimeSlotMap& slot_map,
                                             PartInfo& part_info)
      INTRINSIC_CHECK_REALTIME_SAFE;

  enum class SafetyActionStatus {
    kUnknown,
    // Safety action was not required to run this cycle. This means that both of
    // the following are true:
    // * There is no active safety event for the corresponding part.
    // * There *is* currently an active session with an action controlling the
    //   corresponding part.
    kNotRequired,
    // The safety action ran this cycle, but did not report kIsDone == true. It
    // will continue to run until it does report kIsDone == true.
    kRunning,
    // The safety action ran this cycle and reported kIsDone == true.
    kDone,
  };

  // Runs the safety action if either
  // * there is an active safety request for the part corresponding to
  //   `part_info`.
  // * there is no safety request, no session has marked the part as controlled
  //   either.
  //
  // Forces the speed override for the safety action to 1.0.
  RealtimeStatusOr<SafetyActionStatus> RunSafetyActionIfRequired(
      PartInfo& part_info) INTRINSIC_CHECK_REALTIME_SAFE;

  // This pulls double duty as a flag that indicates which of BeginCycle() and
  // FinishCycle() was last called.
  // BeginCycle() sets a value, and FinishCycle() clears it.
  InvalidUntilSet<intrinsic::Time> cycle_start_ = std::nullopt;
  AsyncBuffer<FixedVector<OperationalState, kMaxRealtimeParts>>
      part_states_buffer_;
  intrinsic::FixedVector<PartInfo, kMaxRealtimeParts> part_infos_;
  AsyncBuffer<AllPartProperties> part_properties_rt_to_non_rt_buffer_;
  AllPartProperties* current_cycle_part_properties_read_ = nullptr;
  AsyncBuffer<AllPartProperties> part_properties_non_rt_to_rt_buffer_;
  AllPartProperties* current_cycle_part_properties_write_ = nullptr;

  // We need this member to partially fill it in and return it from
  // BeginCycle() because we need it to evaluate reactions. Just before the end
  // of a cycle, we fully fill it in FinishCycle() for publishing (also in
  // FinishCycle()).
  InvalidUntilSet<PublishOutput> current_robot_status_;
  RealtimeWriteQueue<PublishOutput> robot_status_queue_;
  InitializedAsyncBuffer<PublishOutput> current_robot_status_buffer_;

  // The current status of the safety system. The SafetyStatus is initialized to
  // correspond to the intrinsic_proto::icon::SafetyStatus default values.
  SafetyStatus current_safety_status_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_PART_MANAGER_H_
