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

#include "intrinsic/simulation/service/simulator_world_sync_v1.h"

#include "intrinsic/util/status/status_conversion_grpc.h"

namespace intrinsic {
namespace simulation {

::grpc::Status SimulatorWorldSyncV1::SyncWorld(
    ::grpc::ServerContext* context,
    ::grpc::ServerReaderWriter<
        ::intrinsic_proto::simulation::v1::WorldMessage,
        ::intrinsic_proto::simulation::v1::SimulatorSceneMessage>* stream) {
  return ToGrpcStatus(impl_->SyncSimulatorWorld(context, stream));
}

::grpc::Status SimulatorWorldSyncV1::GetSimulatorWorldInfo(
    ::grpc::ServerContext* context,
    const ::intrinsic_proto::simulation::v1::GetSimulatorWorldInfoRequest*
        request,
    ::intrinsic_proto::simulation::v1::SimulatorWorldInfo* response) {
  return ToGrpcStatus(impl_->GetSimulatorWorldInfo(context, request, response));
}

}  // namespace simulation
}  // namespace intrinsic
