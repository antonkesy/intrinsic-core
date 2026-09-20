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

#include "intrinsic/simulation/gazebo/gazebo_performance_logger.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "google/protobuf/duration.pb.h"
#include "gz/msgs/MessageTypes.hh"
#include "gz/transport/SubscribeOptions.hh"
#include "intrinsic/logging/data_logger_client.h"
#include "intrinsic/logging/proto/log_item.pb.h"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/simulation/gazebo/proto/introspection.pb.h"
#include "intrinsic/util/proto_time.h"

namespace intrinsic {
namespace simulation {

// Private data for the callback, held in a shared_ptr for thread safety.
struct GazeboPerformanceLogger::CallbackData {
  Publisher pub;
  const std::string logger_event_source;
};

absl::StatusOr<std::unique_ptr<GazeboPerformanceLogger>>
GazeboPerformanceLogger::CreateAndStart(
    PubSub* absl_nonnull pubsub, const GazeboPerformanceLoggerConfig& config) {
  INTR_ASSIGN_OR_RETURN(
      Publisher pub, pubsub->CreatePublisher(config.pubsub_publish_topic, {}));

  auto callback_data = std::make_shared<CallbackData>(
      CallbackData{std::move(pub), config.logger_event_source});

  auto logger =
      absl::WrapUnique(new GazeboPerformanceLogger(config.stats_topic));
  logger->callback_data_ = callback_data;

  return logger;
}

GazeboPerformanceLogger::GazeboPerformanceLogger(const std::string& stats_topic)
    : stats_topic_(stats_topic) {}

}  // namespace simulation
}  // namespace intrinsic
