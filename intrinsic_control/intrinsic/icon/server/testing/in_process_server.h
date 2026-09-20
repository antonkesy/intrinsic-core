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

#ifndef INTRINSIC_ICON_SERVER_TESTING_IN_PROCESS_SERVER_H_
#define INTRINSIC_ICON_SERVER_TESTING_IN_PROCESS_SERVER_H_

#include <memory>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/channel.h"
#include "intrinsic/icon/proto/v1/service.grpc.pb.h"
#include "intrinsic/icon/server/grpc_envelope.h"
#include "intrinsic/icon/server/robot_connection_interface.h"

// Provides an in-process ICON Application Layer server for testing purposes.
// This uses a GrpcEnvelope to achieve behavior that's as close as possible to a
// "real" ICON server.
//
// Example usage of an in-process server with a single action:
//
// InProcessApplicationLayerServer in_process_server({"robot_arm",
//                                                    "suction_gripper"},
//                                                   {NoopActionType()});
// Client client(in_process_server.MakeStub());

namespace intrinsic {
namespace icon {

// In-process ICON Application Layer Server. Can either use an in-process
// channel or an assigned port.
class InProcessApplicationLayerServer {
 public:
  struct Options {
    // If true, uses an in-process channel. Otherwise, listens for requests on
    // `port` using LocalServerCredentials.
    bool use_in_process_channel;
    // Port to use when `use_in_process_channel` is false. Ignored if
    // `use_in_process_channel` is true.
    int port;
  };

  static Options DefaultOptions() {
    return Options{
        .use_in_process_channel = true,
        .port = 0,
    };
  }

  // Constructs an in-process ICON Application Layer Server with the provided
  // `robot_connection`. `robot_connection` must outlive the server!
  explicit InProcessApplicationLayerServer(
      RobotConnectionInterface& robot_connection,
      Options options = DefaultOptions());

  // Creates a stub that interacts with this server.
  std::unique_ptr<intrinsic_proto::icon::v1::IconApi::StubInterface> MakeStub()
      const;

  std::shared_ptr<grpc::Channel> GetChannel() const;

  // Returns the number of times  `grpc_envelope_` has built a new ICON service.
  // This happens when a client calls RestartServer(), or when a client calls
  // ClearFaults() after ICON encountered a fatal fault.
  int IconImplFactoryCalls() const ABSL_LOCKS_EXCLUDED(factory_calls_mutex_);

 private:
  mutable absl::Mutex factory_calls_mutex_;
  // Counts the number of times `grpc_envelope_` has built a new ICON
  // service. This happens when a client calls RestartServer(), or when a
  // client calls ClearFaults() after ICON encountered a fatal fault.
  int icon_impl_factory_calls_ ABSL_GUARDED_BY(factory_calls_mutex_) = 0;
  std::unique_ptr<GrpcEnvelope> grpc_envelope_;
  std::shared_ptr<grpc::Channel> channel_;
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_SERVER_TESTING_IN_PROCESS_SERVER_H_
