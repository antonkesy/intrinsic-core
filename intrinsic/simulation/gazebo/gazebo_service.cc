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

#include "intrinsic/simulation/gazebo/gazebo_service.h"

#include "absl/synchronization/mutex.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "gz/transport/NetUtils.hh"
#include "intrinsic/simulation/gazebo/proto/v1/gazebo_service.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace simulation {

absl::Status GazeboService::SetTopics(
    const std::map<std::string, std::vector<GzTopicInfo>>& object_topics,
    const std::map<std::string, std::vector<GzTopicInfo>>&
        world_plugin_topics) {
  std::map<std::string, intrinsic_proto::simulation::v1::Topics>
      object_topic_protos;
  for (const auto& [name, list] : object_topics) {
    if (list.empty()) {
      continue;
    }
    INTR_ASSIGN_OR_RETURN(auto topics_proto, ToProto(list),
                          _ << "Object: " << name);
    for (auto& topic : *topics_proto.mutable_topics()) {
      topic.mutable_object_info()->set_name(name);
    }
    object_topic_protos[name] = std::move(topics_proto);
  }

  std::map<std::string, intrinsic_proto::simulation::v1::Topics>
      world_plugin_topic_protos;
  for (const auto& [name, list] : world_plugin_topics) {
    if (list.empty()) {
      continue;
    }
    INTR_ASSIGN_OR_RETURN(auto topics_proto, ToProto(list),
                          _ << "World Plugin: " << name);
    for (const auto& topic : topics_proto.topics()) {
      if (topic.has_object_info()) {
        return intrinsic::InvalidArgumentErrorBuilder()
               << "World plugin topics cannot have object_info set.; World "
                  "Plugin: "
               << name;
      }
    }
    world_plugin_topic_protos[name] = std::move(topics_proto);
  }

  absl::MutexLock lock(&mutex_);
  object_topics_ = std::move(object_topic_protos);
  world_plugin_topics_ = std::move(world_plugin_topic_protos);
  return absl::OkStatus();
}

::grpc::Status GazeboService::ListTopics(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::v1::ListTopicsRequest* request,
    intrinsic_proto::simulation::v1::Topics* response) {
  absl::ReaderMutexLock lock(&mutex_);

  auto filter_type = request->entity_type_filter();
  const bool include_models =
      filter_type == intrinsic_proto::simulation::v1::ALL ||
      filter_type == intrinsic_proto::simulation::v1::MODELS ||
      filter_type ==
          intrinsic_proto::simulation::v1::TOPIC_ENTITY_TYPE_FILTER_UNSPECIFIED;
  const bool include_sensors =
      filter_type == intrinsic_proto::simulation::v1::ALL ||
      filter_type == intrinsic_proto::simulation::v1::SENSORS ||
      filter_type ==
          intrinsic_proto::simulation::v1::TOPIC_ENTITY_TYPE_FILTER_UNSPECIFIED;
  const bool include_all = include_models && include_sensors;

  auto add_matching_topics =
      [&include_all, &include_models, &include_sensors,
       &response](const intrinsic_proto::simulation::v1::Topics& topics_proto) {
        for (const auto& topic : topics_proto.topics()) {
          if (include_all) {
            *response->add_topics() = topic;
          } else if ((topic.has_plugin_info() && include_models) ||
                     (topic.has_sensor_info() && include_sensors)) {
            *response->add_topics() = topic;
          }
        }
      };

  if (request->has_object_name()) {
    auto it = object_topics_.find(request->object_name());
    if (it != object_topics_.end()) {
      add_matching_topics(it->second);
    }
  } else {
    for (const auto& [name, topics_proto] : object_topics_) {
      add_matching_topics(topics_proto);
    }
    if (include_all) {
      for (const auto& [name, topics_proto] : world_plugin_topics_) {
        add_matching_topics(topics_proto);
      }
    }
  }

  return ::grpc::Status::OK;
}

::grpc::Status GazeboService::GetIPForTransportRelay(
    ::grpc::ServerContext* context,
    const intrinsic_proto::simulation::v1::GetIPRequest* request,
    intrinsic_proto::simulation::v1::GetIPResponse* response) {
  std::string ip = gz::transport::determineHost();
  if (ip.empty()) {
    return ToGrpcStatus(absl::InternalError("Failed to determine host IP."));
  }
  response->set_ip_address(ip);
  response->set_partition(gz_transport_partition_);
  return ::grpc::Status::OK;
}

}  // namespace simulation
}  // namespace intrinsic
