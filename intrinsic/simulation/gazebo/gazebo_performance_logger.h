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

#ifndef INTRINSIC_SIMULATION_GAZEBO_GAZEBO_PERFORMANCE_LOGGER_H_
#define INTRINSIC_SIMULATION_GAZEBO_GAZEBO_PERFORMANCE_LOGGER_H_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "gz/transport/Node.hh"
#include "intrinsic/platform/pubsub/pubsub.h"

namespace intrinsic {
namespace simulation {

// Configuration for the GazeboPerformanceLogger.
struct GazeboPerformanceLoggerConfig {
  // The Gazebo topic to subscribe to for WorldStatistics messages.
  std::string stats_topic = "/stats";

  // The Intrinsic PubSub topic to publish performance metrics to.
  std::string pubsub_publish_topic = "simulation/metrics";

  // The DataLogger event source string to use for logging historical
  // performance data.
  std::string logger_event_source = "simulator.performance";

  // The desired frequency (messages per second) for publishing performance
  // messages.
  int messages_per_sec = 2;
};

// Logs Gazebo performance statistics for a running Gazebo simulation server.
// Logs current performance data to pubsub, and historical performance data to
// the data logger.
// Expects `StartUpIntrinsicLoggerViaGrpc` to have been called before this class
// is instantiated.
class GazeboPerformanceLogger {
 public:
  // Creates and starts a GazeboPerformanceLogger instance.
  static absl::StatusOr<std::unique_ptr<GazeboPerformanceLogger>>
  CreateAndStart(PubSub* absl_nonnull pubsub,
                 const GazeboPerformanceLoggerConfig& config);

  GazeboPerformanceLogger(const GazeboPerformanceLogger&) = delete;
  GazeboPerformanceLogger& operator=(const GazeboPerformanceLogger&) = delete;
  GazeboPerformanceLogger(GazeboPerformanceLogger&&) = delete;
  GazeboPerformanceLogger& operator=(GazeboPerformanceLogger&&) = delete;

 private:
  struct CallbackData;

  explicit GazeboPerformanceLogger(const std::string& stats_topic);

  gz::transport::Node node_;
  const std::string stats_topic_;
  std::shared_ptr<CallbackData> callback_data_;
  gz::transport::Node::Subscriber subscriber_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_GAZEBO_PERFORMANCE_LOGGER_H_
