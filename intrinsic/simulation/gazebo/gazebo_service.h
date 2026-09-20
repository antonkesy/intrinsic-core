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

#ifndef INTRINSIC_SIMULATION_GAZEBO_GAZEBO_SERVICE_H_
#define INTRINSIC_SIMULATION_GAZEBO_GAZEBO_SERVICE_H_

#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/simulation/gazebo/gz_topic_info.h"
#include "intrinsic/simulation/gazebo/proto/v1/gazebo_service.grpc.pb.h"
#include "intrinsic/simulation/gazebo/proto/v1/gazebo_service.pb.h"

namespace intrinsic {
namespace simulation {

class GazeboService final
    : public intrinsic_proto::simulation::v1::GazeboService::Service {
 public:
  explicit GazeboService(std::string gz_transport_partition = "")
      : gz_transport_partition_(std::move(gz_transport_partition)) {}

  ::grpc::Status GetIPForTransportRelay(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::v1::GetIPRequest* request,
      intrinsic_proto::simulation::v1::GetIPResponse* response) override;

  ::grpc::Status ListTopics(
      ::grpc::ServerContext* context,
      const intrinsic_proto::simulation::v1::ListTopicsRequest* request,
      intrinsic_proto::simulation::v1::Topics* response) override
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Sets the topic mappings for objects and world plugins to be served by
  // ListTopics. std::map is used to preserve deterministic ordering in
  // ListTopics responses.
  absl::Status SetTopics(
      const std::map<std::string, std::vector<GzTopicInfo>>& object_topics,
      const std::map<std::string, std::vector<GzTopicInfo>>&
          world_plugin_topics = {}) ABSL_LOCKS_EXCLUDED(mutex_);

 private:
  absl::Mutex mutex_;
  std::map<std::string, intrinsic_proto::simulation::v1::Topics> object_topics_
      ABSL_GUARDED_BY(mutex_);
  std::map<std::string, intrinsic_proto::simulation::v1::Topics>
      world_plugin_topics_ ABSL_GUARDED_BY(mutex_);
  std::string gz_transport_partition_;
};
}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_GAZEBO_SERVICE_H_
