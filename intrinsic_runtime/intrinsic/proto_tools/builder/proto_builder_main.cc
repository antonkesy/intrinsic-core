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

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>

#include "absl/flags/flag.h"
#include "absl/strings/str_format.h"
#include "grpc/grpc.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server.h"
#include "grpcpp/server_builder.h"
#include "intrinsic_runtime/intrinsic/proto_tools/builder/proto_builder_service.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"

ABSL_FLAG(int32_t, port, 8080, "Port to listen on for gRPC service.");

int main(int argc, char* argv[]) {
  InitIntrinsic(argv[0], argc, argv);

  const std::string server_address =
      absl::StrFormat("[::]:%d", absl::GetFlag(FLAGS_port));

  grpc::ServerBuilder builder;
  builder.AddListeningPort(server_address,
                           grpc::InsecureServerCredentials());  // NOLINT
  // "0" means no port reuse. Allowing other servers on the same port could
  // introduce hard-to-debug behavior or flaky tests.
  builder.AddChannelArgument(GRPC_ARG_ALLOW_REUSEPORT, 0);

  intrinsic::executive::ProtoBuilderService service;
  builder.RegisterService(&service);

  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  server->Wait();

  return EXIT_SUCCESS;
}
