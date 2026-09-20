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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_ACTION_INSTANCE_H_
#define INTRINSIC_ICON_CONTROL_RTCL_ACTION_INSTANCE_H_

#include <memory>

#include "absl/container/fixed_array.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/realtime_signal_storage.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/streaming_io_storage.h"

namespace intrinsic::icon {

// RealtimeActionInstance contains all information to execute an action in
// realtime. This struct is a server implementation detail and does not concern
// Action authors.
struct RtclActionInstance {
  std::unique_ptr<RealtimeStreamingIoStorage> streaming_io_storage;
  std::unique_ptr<RealtimeSignalStorage> realtime_signal_storage;
  std::unique_ptr<RtclActionInterface> action;
  // Each Action only has access to a certain set of Slots, each of which is
  // identified by a string name in non-realtime (i.e. the Action factory), and
  // a RealtimeSlotId in realtime. RealtimeSlotIds are just indices into the
  // array of Parts in the realtime thread.
  //
  // For faster (guaranteed O(1)) lookup of Slot visibility, we don't store
  // the indices of Slots available to the Action, but a dense array of boolean
  // values indicating whether each Part is visible to the Action.
  // The realtime thread uses this to provide a RealtimeSlotMap that only allows
  // access to those Slots.
  std::unique_ptr<absl::FixedArray<bool>> visibility_by_slot_id;
  ActionInstanceId id;

  // An Action can register supported BehaviorOverrides in the ActionSignature.
  // The information is used to request temporary behavior overrides, or close
  // the Session.
  //
  // For faster (guaranteed O(1)) lookup of the BehaviorOverride support, they
  // are stored as a dense array of boolean values.
  // A BehaviorOverride is supported when the array index of the enum value is
  // `true`.
  // e.g. `supported_behavior_overrides_by_enum_value[static_cast<size_t> enum]
  // == true;`
  // Index 0 (= BEHAVIOR_OVERRIDE_REQUEST_UNKNOWN) is always `true`.
  std::unique_ptr<absl::FixedArray<bool>>
      supported_behavior_overrides_by_enum_value;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_ACTION_INSTANCE_H_
