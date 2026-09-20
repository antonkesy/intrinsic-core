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

#include "intrinsic/icon/server/session_instance.h"

#include <memory>
#include <utility>

#include "grpcpp/server_context.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/server/session_interface.h"

namespace intrinsic::icon {

SessionInstance::SessionInstance(
    SessionId id, std::unique_ptr<SessionInterface> session_interface,
    grpc::ServerContext* open_session_stream)
    : open_session_stream_(open_session_stream),
      id_(id),
      interface_(std::move(session_interface)) {}

}  // namespace intrinsic::icon
