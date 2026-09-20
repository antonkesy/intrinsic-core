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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_SESSION_CHANNELS_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_SESSION_CHANNELS_H_

#include "intrinsic/icon/control/realtime_function_bridge.h"
#include "intrinsic/icon/control/rtcl_realtime_session.h"
#include "intrinsic/icon/interprocess/binary_futex.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/platform/common/buffers/rt_queue.h"

namespace intrinsic::icon {

// Bundles all the Session-related communication channels between the realtime
// and non-realtime threads.
struct RealtimeSessionChannels {
  // Used to install new RtclRealtimeSession objects. Ownership of
  // `session_ptr` remains with the non-realtime thread! Realtime has exclusive
  // write access to the pointer only until either
  // * the next call to install_session_data_bridge or
  // * a call to finish_session_bridge
  //
  // This isn't a plain AsyncBuffer because that class does not allow easy
  // acknowledgement from the receiver, and we can only delete Action
  // instances once we know the realtime thread is no longer using them.
  SimpleRealtimeFunctionBridge<RealtimeStatus(RtclRealtimeSession* session_ptr)>
      install_session_data_bridge;
  // RtclControllerSession destructor sets this to
  // * signal the realtime thread to stop using any RtclRealtimeSession with
  // `session_id`
  // * mark the resources used by the Session as free to use for another one.
  //
  // There should only be one such RtclRealtimeSession: The one that we last
  // passed to `install_session_`. The realtime thread acknowledges this by
  // posting `finish_done`.
  BinaryFutex finish_requested =
      BinaryFutex(/*posted=*/false, /*private_futex=*/true);
  // The realtime thread posts this in response to the non-realtime
  // thread posting `finish_requested`.
  // Once this is posted, the realtime thread may not refer to the
  // RealtimeSessionChannels object in any way.
  BinaryFutex finish_done =
      BinaryFutex(/*posted=*/false, /*private_futex=*/true);
  RealtimeQueue<ReactionEvent> reaction_queue;
  // The non-realtime thread acts as a consumer for this buffer – if it ever
  // contains a non-OK status, any method calls on ICON clients will
  // return that status instead of doing any work.
  AsyncBuffer<RealtimeStatus> session_status_buffer =
      AsyncBuffer<RealtimeStatus>(icon::OkStatus());
};
}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_SESSION_CHANNELS_H_
