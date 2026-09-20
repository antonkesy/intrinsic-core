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

#ifndef INTRINSIC_SIMULATION_GAZEBO_GRPC_PROXY_GRPC_RELAY_H_
#define INTRINSIC_SIMULATION_GAZEBO_GRPC_PROXY_GRPC_RELAY_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "grpcpp/generic/callback_generic_service.h"
#include "grpcpp/grpcpp.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/grpc/connection_params.h"

namespace intrinsic {
namespace simulation {

// A gRPC relay server that forwards requests to a remote gRPC server.
// This class is agnostic to the services being relayed. It uses the
// grpc::ByteBuffer pattern to transparently relay data without
// serialization/deserialization.
class GrpcRelay {
 public:
  ~GrpcRelay();

  // Creates a new GrpcRelay instance and starts a gRPC server on the given
  // port. Relays to the remote server specified in connection params.
  static absl::StatusOr<std::unique_ptr<GrpcRelay>> CreateAndStart(
      int port, const ConnectionParams& connection_params,
      absl::Duration connect_timeout);

  // Variant that takes a connected `Channel` as input.
  static absl::StatusOr<std::unique_ptr<GrpcRelay>> CreateAndStart(
      int port, std::shared_ptr<Channel> channel);

  // Shuts down the gRPC server.
  void Shutdown();

 private:
  explicit GrpcRelay(std::shared_ptr<Channel> channel);

  std::shared_ptr<Channel> channel_;
  std::unique_ptr<grpc::CallbackGenericService> generic_service_;
  std::unique_ptr<grpc::Server> server_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_GRPC_PROXY_GRPC_RELAY_H_
