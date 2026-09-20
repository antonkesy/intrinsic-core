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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_SESSION_BRIDGE_INTERFACE_H_
#define INTRINSIC_ICON_CONTROL_RTCL_SESSION_BRIDGE_INTERFACE_H_

#include <optional>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "intrinsic/icon/control/rtcl_realtime_session.h"
#include "intrinsic/icon/server/session_interface.h"

namespace intrinsic::icon {

// This is a virtual class that defines the API between a Session in the
// realtime thread and its counterpart in the non-realtime (gRPC server) thread.
//
// This should behave in an RAII way, i.e. make sure the realtime thread cleans
// up any resources related to the Session upon destruction.
class RtclSessionBridgeInterface {
 public:
  virtual ~RtclSessionBridgeInterface() = default;

  // Returns the current status of the Rtcl (realtime) Session. If this ever
  // contains a non-OK status, any method calls on RtclControllerSession return
  // that status instead of doing any work.
  virtual absl::Status GetSessionStatus() = 0;

  // Signals the realtime thread to stop using any RtclRealtimeSession with
  // `session_id`. There should only be one such RtclRealtimeSession: The one
  // that we last passed to `InstallSessionData()`.
  //
  // Blocks until the realtime thread notifies us that it's done finishing, i.e.
  // it will not access any of our RtclRealtimeSession pointers in the future.
  virtual bool FinishSession() = 0;

  // Returns any recent Reactions. Make sure to poll this frequently, because
  // otherwise the realtime thread is forced to drop events.
  virtual std::optional<ReactionEvent> PollReactions() = 0;

  // Installs `session_ptr` in the realtime thread. We need to install a new
  // RtclRealtimeSession pointer whenever we want to
  // * add or remove an Action/Reaction
  // * start an Action
  // * end the Session
  //
  // The caller must ensure that `session_ptr` remains valid until either
  // * The end of this call to InstallSessionData(), if it returns a non-OK
  //   status.
  // * The next call to InstallSessionData().
  // * A call to FinishSession().
  //
  // At no point does RtclSessionBridgeInterface assume ownership of
  // `session_ptr`. In particular, it MUST NOT attempt to delete or free it.
  //
  // Blocks until the realtime thread has acknowledged the call.
  virtual absl::Status InstallSessionData(RtclRealtimeSession* session_ptr) = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_SESSION_BRIDGE_INTERFACE_H_
