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

#ifndef INTRINSIC_HARDWARE_GRIPPER_SERVICE_SIM_SIM_PINCH_GRIPPER_IMPL_H__
#define INTRINSIC_HARDWARE_GRIPPER_SERVICE_SIM_SIM_PINCH_GRIPPER_IMPL_H__

#include <memory>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.grpc.pb.h"
#include "intrinsic/hardware/gripper/service/proto/pinch_gripper_server.pb.h"
#include "intrinsic/resources/health/health_state_machine.h"
#include "intrinsic/resources/health/operational_status.h"

namespace intrinsic::gripper::simulation {

// Class that implements methods to expose gRPC services for pinch gripper in
// physics sim. The methods that interact with the gripper simply forward the
// requests to the pinch gripper gRPC service running as part of the simulator.
// intrinsic/simulation/gazebo/plugins/grippers/pinch_gripper_server_impl.h

// The main advantage is that all the client requests are first sent to the
// pinch gripper resource instance and then forwarded either to real hardware
// (e.g. modbus tcp) or to sim. Additionally, extra resource-specific
// functionality like health service only need to be implemented once as part of
// the resource type.
class SimPinchGripperImpl {
 public:
  explicit SimPinchGripperImpl(
      const intrinsic_proto::gripper::PinchGripperConfig& config,
      std::unique_ptr<
          intrinsic_proto::gripper::PinchGripperServer::StubInterface>
          stub)
      : stub_(std::move(stub)), config_(config) {}

  ~SimPinchGripperImpl() = default;

  absl::Status CreatePinchGripper(
      const intrinsic_proto::gripper::CreatePinchGripperRequest& request,
      intrinsic_proto::gripper::CreatePinchGripperResponse* response)
      ABSL_LOCKS_EXCLUDED(mutex_);

  absl::Status ResetPinchGripper(
      const intrinsic_proto::gripper::ResetPinchGripperRequest& request,
      intrinsic_proto::gripper::ResetPinchGripperResponse* response)
      ABSL_LOCKS_EXCLUDED(mutex_);

  absl::Status CommandPinchGripper(
      const intrinsic_proto::gripper::CommandPinchGripperRequest& request,
      intrinsic_proto::gripper::CommandPinchGripperResponse* response)
      ABSL_LOCKS_EXCLUDED(mutex_);

  absl::Status GetPinchGripperStatus(
      const intrinsic_proto::gripper::GetPinchGripperStatusRequest& request,
      intrinsic_proto::gripper::GetPinchGripperStatusResponse* response)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Initializes the gripper and, if successful, enables it.
  absl::Status InitializeAndEnable() ABSL_LOCKS_EXCLUDED(mutex_);

  // Returns the current ServiceState of the instance.
  intrinsic_proto::services::v1::SelfState GetState() const
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Attempts to clear the faults and disables `CommandPinchGripper` rpc.
  absl::Status ClearFaults() ABSL_LOCKS_EXCLUDED(mutex_);

  // Enables `CommandPinchGripper` rpc.
  absl::Status Enable() ABSL_LOCKS_EXCLUDED(mutex_);

  // Disables `CommandPinchGripper` rpc.
  absl::Status Disable() ABSL_LOCKS_EXCLUDED(mutex_);

 private:
  // gRPC client to forward requests to the pinch gripper running within the
  // simulator.
  std::unique_ptr<intrinsic_proto::gripper::PinchGripperServer::StubInterface>
      stub_;

  const intrinsic_proto::gripper::PinchGripperConfig config_;

  mutable absl::Mutex mutex_;

  // Starts the state machine in an unspecified state.
  intrinsic::resources::HealthStateMachine state_machine_
      ABSL_GUARDED_BY(mutex_) = intrinsic::resources::HealthStateMachine(
          intrinsic::resources::OperationalState::kUnspecified, "gripper");
};

}  // namespace intrinsic::gripper::simulation

#endif  // INTRINSIC_HARDWARE_GRIPPER_SERVICE_SIM_SIM_PINCH_GRIPPER_IMPL_H__
