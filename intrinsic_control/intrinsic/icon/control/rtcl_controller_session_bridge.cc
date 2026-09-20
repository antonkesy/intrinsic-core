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

#include "intrinsic/icon/control/rtcl_controller_session_bridge.h"

#include <optional>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/realtime_session_channels.h"
#include "intrinsic/icon/control/rtcl_realtime_session.h"
#include "intrinsic/icon/server/session_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// Generous timeout to wait for the realtime thread to finish a session to avoid
// waiting forever.
constexpr auto kFinishDoneTimeout = absl::Seconds(60);

RtclControllerSessionBridge::~RtclControllerSessionBridge() {
  (void)FinishSession();
}

absl::Status RtclControllerSessionBridge::CheckSessionRunning() const {
  if (channels_ == nullptr) {
    return absl::FailedPreconditionError(
        absl::StrCat("Session ", session_id_.value(), " has ended already."));
  }
  return absl::OkStatus();
}

absl::Status RtclControllerSessionBridge::GetSessionStatus() {
  absl::MutexLock l(mutex_);
  INTR_RETURN_IF_ERROR(CheckSessionRunning());
  RealtimeStatus* session_status = nullptr;
  channels_->session_status_buffer.GetActiveBuffer(&session_status);
  return *session_status;
}

bool RtclControllerSessionBridge::FinishSession() {
  absl::MutexLock l(mutex_);
  if (channels_ == nullptr) {
    return false;
  }
  if (auto status = channels_->finish_requested.Post(); !status.ok()) {
    LOG(ERROR) << "Failed to post finish_requested: " << status.message();
  };

  if (auto status = channels_->finish_done.WaitFor(kFinishDoneTimeout);
      !status.ok()) {
    LOG(ERROR) << "Failed to wait for finish_done: " << status.message();
    // Still destroy the channels – we don't want the RealtimeSession to be able
    // to communicate after calling FinishSession(), success or not.
    channels_ = nullptr;
    return false;
  }
  channels_ = nullptr;
  return true;
}

std::optional<ReactionEvent> RtclControllerSessionBridge::PollReactions() {
  absl::MutexLock l(mutex_);
  if (absl::Status session_ended_status = CheckSessionRunning();
      !session_ended_status.ok()) {
    LOG_EVERY_N_SEC(ERROR, 1) << session_ended_status.message();
    return std::nullopt;
  }
  return channels_->reaction_queue.reader()->Pop();
}

absl::Status RtclControllerSessionBridge::InstallSessionData(
    RtclRealtimeSession* session_ptr) {
  absl::MutexLock l(mutex_);
  INTR_RETURN_IF_ERROR(CheckSessionRunning());
  INTR_ASSIGN_OR_RETURN(
      RealtimeStatus status,
      channels_->install_session_data_bridge.Call(session_ptr));
  return status;
}

}  // namespace intrinsic::icon
