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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GPIO_GPIO_SERVICE_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GPIO_GPIO_SERVICE_H_

#include <memory>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/server.h"
#include "grpcpp/support/status.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/simulation/gazebo/plugins/gpio/gpio_connection_interface.h"

namespace intrinsic {
namespace simulation {

class GPIOService final
    : public ::intrinsic_proto::gpio::v1::GPIOService::Service {
 public:
  ::grpc::Status GetSignalDescriptions(
      ::grpc::ServerContext* context,
      const intrinsic_proto::gpio::v1::GetSignalDescriptionsRequest* request,
      intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse* response)
      override ABSL_LOCKS_EXCLUDED(connections_mutex_);
  ::grpc::Status ReadSignals(
      ::grpc::ServerContext* context,
      const intrinsic_proto::gpio::v1::ReadSignalsRequest* request,
      intrinsic_proto::gpio::v1::ReadSignalsResponse* response) override
      ABSL_LOCKS_EXCLUDED(connections_mutex_);
  ::grpc::Status WaitForValue(
      ::grpc::ServerContext* context,
      const intrinsic_proto::gpio::v1::WaitForValueRequest* request,
      intrinsic_proto::gpio::v1::WaitForValueResponse* response) override;
  ::grpc::Status OpenWriteSession(
      ::grpc::ServerContext* context,
      ::grpc::ServerReaderWriter<
          intrinsic_proto::gpio::v1::OpenWriteSessionResponse,
          intrinsic_proto::gpio::v1::OpenWriteSessionRequest>* stream) override
      ABSL_LOCKS_EXCLUDED(claimed_signals_mutex_);

  // Registers plugin_connection to this service to serve the gRPC interface.
  // Expects UnregisterPluginConnection to be called before plugin_connection
  // goes out of scope.
  absl::Status RegisterPluginConnection(
      GPIOConnectionInterface* plugin_connection)
      ABSL_LOCKS_EXCLUDED(connections_mutex_);

  absl::Status UnregisterPluginConnection(absl::string_view plugin_handle)
      ABSL_LOCKS_EXCLUDED(connections_mutex_);

  // Creates and starts a singleton gRPC service.
  static GPIOService& StartSingleton();

 private:
  // Polls all signal values based on request. Returns an absl::NotFoundError
  // when the request is not matched by current signal values.
  absl::StatusOr<intrinsic_proto::gpio::v1::WaitForValueResponse>
  PollMatchingValues(
      const intrinsic_proto::gpio::v1::WaitForValueRequest& request)
      ABSL_LOCKS_EXCLUDED(connections_mutex_);

  // Attempts to claim signals provided for writing. Returns an error if the
  // signals are already claimed.
  absl::Status ClaimSignals(const absl::flat_hash_set<std::string>& signals)
      ABSL_LOCKS_EXCLUDED(claimed_signals_mutex_);

  // Attempts to release the signals from being claimed for writing.
  void ReleaseSignals(const absl::flat_hash_set<std::string>& signals)
      ABSL_LOCKS_EXCLUDED(claimed_signals_mutex_);

  // Attempts to write signals to the underlying gripper connections.
  // Returns a NotFound error if the signal set does not match any of the
  // underlying connections. Returns an internal error if the signal write
  // failed with the signals.
  absl::Status WriteSignals(
      const intrinsic_proto::gpio::v1::WriteSignalsRequest& request)
      ABSL_LOCKS_EXCLUDED(connections_mutex_);

  void StartServer();

  absl::Mutex connections_mutex_;
  absl::flat_hash_map<std::string, GPIOConnectionInterface*> plugin_connections_
      ABSL_GUARDED_BY(connections_mutex_);

  absl::Mutex claimed_signals_mutex_;
  absl::flat_hash_set<std::string> claimed_signals_
      ABSL_GUARDED_BY(claimed_signals_mutex_);

  std::unique_ptr<::grpc::Server> server_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GPIO_GPIO_SERVICE_H_
