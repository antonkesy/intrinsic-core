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

#ifndef INTRINSIC_ICON_SERVER_SESSION_INSTANCE_H_
#define INTRINSIC_ICON_SERVER_SESSION_INSTANCE_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "grpcpp/server_context.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/server/session_interface.h"

namespace intrinsic::icon {

// Contains all server-side state associated with a session.
class SessionInstance {
 public:
  SessionInstance(SessionId id, std::unique_ptr<SessionInterface> interface,
                  grpc::ServerContext* open_session_stream);

  SessionId GetId() const { return id_; }

  SessionInterface& GetInterface() { return *interface_; }

 public:
  // Ensure only one WatchReactions stream polls and publishes reactions (to
  // avoid reactions getting dropped).
  // If you also need ApplicationLayerServer::session_map_mutex_, acquire that
  // first to avoid deadlocks. (OpenSession and WatchReactions methods need
  // both mutexes to create and delete a SessionInstance.)
  absl::Mutex watch_reactions_;
  // Request closing the WatchReactions grpc stream.
  absl::Notification stop_watch_reactions_;
  // Stream to cancel on server-side errors.
  grpc::ServerContext* open_session_stream_;
  // Do not cancel OpenSession before it writes error message.
  absl::Notification error_write_done_;

 private:
  SessionId id_;
  std::unique_ptr<SessionInterface> interface_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_SESSION_INSTANCE_H_
