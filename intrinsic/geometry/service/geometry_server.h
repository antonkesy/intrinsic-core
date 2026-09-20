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

#ifndef INTRINSIC_GEOMETRY_SERVICE_GEOMETRY_SERVER_H_
#define INTRINSIC_GEOMETRY_SERVICE_GEOMETRY_SERVER_H_

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "grpcpp/server.h"
#include "intrinsic/geometry/service/geometry_service_impl.h"
#include "intrinsic/longrunning/cc/operation_scheduler_interface.h"
#include "intrinsic/longrunning/cc/operations_service_impl.h"

namespace intrinsic::geo {
// Options for starting the GeometryServer.
struct GeometryServerOptions {
  int port = 10001;
  std::string cas_service_address;
  int geometry_cas_cache_size = 1000;
  absl::Duration shutdown_grace_period = absl::Seconds(10);

  // This notification will be notified when the signal handlers are
  // registered and the server is ready to handle requests.
  absl::Notification& handlers_registered;
};

// Runs the Geometry gRPC server.
// This function will block until the server is shut down.
absl::Status RunGeometryServer(const GeometryServerOptions& options);

}  // namespace intrinsic::geo
#endif  // INTRINSIC_GEOMETRY_SERVICE_GEOMETRY_SERVER_H_
