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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_SESSION_BRIDGE_H_
#define INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_SESSION_BRIDGE_H_

#include <memory>
#include <optional>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/optional.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/realtime_function_bridge.h"
#include "intrinsic/icon/control/realtime_session_channels.h"
#include "intrinsic/icon/control/rtcl_realtime_session.h"
#include "intrinsic/icon/control/rtcl_session_bridge_interface.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/platform/common/buffers/rt_queue.h"

namespace intrinsic::icon {

// RtclControllerSessionBridge communicates with a Session in an RtclController
// using a RealtimeSessionChannels object. There is a 1:1 relation between
// RealtimeSessionChannels and Sessions (and, by extesion,
// RtclControllerSessionBridge). That is, RealtimeSessionChannels are never
// reused across Sessions.
//
// All methods are thread safe.
class RtclControllerSessionBridge final : public RtclSessionBridgeInterface {
 public:
  // Creates an RtclControllerSessionBridge, taking ownership of `channels`.
  // Since `channels` is used in the realtime thread, we notify the realtime
  // thread (by setting finish_requested on `channels`), and wait for it to
  // acknowledge that notification (by watching the value of finish_done) before
  // we delete `channels`.
  RtclControllerSessionBridge(SessionId session_id,
                              std::unique_ptr<RealtimeSessionChannels> channels)
      : session_id_(session_id), channels_(std::move(channels)) {}

  ~RtclControllerSessionBridge() ABSL_LOCKS_EXCLUDED(mutex_) override;

  absl::Status GetSessionStatus() ABSL_LOCKS_EXCLUDED(mutex_) override;
  bool FinishSession() ABSL_LOCKS_EXCLUDED(mutex_) override;
  std::optional<ReactionEvent> PollReactions()
      ABSL_LOCKS_EXCLUDED(mutex_) override;
  absl::Status InstallSessionData(RtclRealtimeSession* session_ptr)
      ABSL_LOCKS_EXCLUDED(mutex_) override;

 private:
  absl::Status CheckSessionRunning() const ABSL_SHARED_LOCKS_REQUIRED(mutex_);

  mutable absl::Mutex mutex_;
  SessionId session_id_;
  std::unique_ptr<RealtimeSessionChannels> channels_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_CONTROLLER_SESSION_BRIDGE_H_
