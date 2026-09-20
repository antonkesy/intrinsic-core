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

#include <memory>
#include <utility>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "intrinsic/geometry/service/geometry_server.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/stats/opencensus.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/macros.h"

ABSL_FLAG(int32_t, port, 10001, "port to listen on for geometry service");
ABSL_FLAG(std::string, cas_service_address, "",
          "Address of the content-addressable storage (CAS) service.");
ABSL_FLAG(int32_t, geometry_cas_cache_size, 1000,
          "The maximum number of geometries to cache from the cas service.");
ABSL_FLAG(absl::Duration, shutdown_grace_period, absl::Seconds(10),
          "The maximum time to wait for the server to shutdown.");

int main(int argc, char** argv) {
  InitIntrinsic(argv[0], argc, argv);
  intrinsic::OpenCensusPlugin open_census;

  absl::Notification handlers_registered;
  intrinsic::geo::GeometryServerOptions options = {
      .port = absl::GetFlag(FLAGS_port),
      .cas_service_address = absl::GetFlag(FLAGS_cas_service_address),
      .geometry_cas_cache_size = absl::GetFlag(FLAGS_geometry_cas_cache_size),
      .shutdown_grace_period = absl::GetFlag(FLAGS_shutdown_grace_period),
      .handlers_registered = handlers_registered,
  };

  QCHECK_OK(intrinsic::geo::RunGeometryServer(options));

  return 0;
}
