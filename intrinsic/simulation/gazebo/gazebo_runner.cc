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

#include "intrinsic/simulation/gazebo/gazebo_runner.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "google/protobuf/message.h"
#include "gz/common/Profiler.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/System.hh"
#include "gz/transport/Node.hh"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/stop_token.h"
#include "intrinsic/util/thread/thread.h"

ABSL_FLAG(bool, fatal_heartbeat_check, false,
          "If true, the heartbeat checker will stop the process and print a "
          "backtrace if the server stopped stepping in running state. "
          "Otherwise just a log will be printed.");

namespace intrinsic {
namespace simulation {
absl::StatusOr<std::unique_ptr<GazeboRunner>> GazeboRunner::Create(
    const gz::sim::ServerConfig& config, PubSub* pubsub,
    absl::Duration step_size, absl::Duration heartbeat_interval) {
  auto server = absl::WrapUnique(
      new GazeboRunner(config, pubsub, heartbeat_interval, step_size));
  INTR_RETURN_IF_ERROR(server->InitStatus());
  return server;
}

GazeboRunner::GazeboRunner(const gz::sim::ServerConfig& config, PubSub* pubsub,
                           absl::Duration heartbeat_interval,
                           absl::Duration step_size)
    : gz_server_(config),
      heartbeat_interval_(heartbeat_interval),
      pubsub_(pubsub),
      step_size_(step_size) {}

GazeboRunner::~GazeboRunner() {
  LOG(INFO) << "Simulation server is shutting down.";
  Stop();
}

absl::Status GazeboRunner::InitStatus() {
  if (gz::sim::Server::Status server_status = gz_server_.GetStatus();
      server_status == gz::sim::Server::Status::EXITED) {
    return AbortedErrorBuilder()
           << "Server failed to initialize from given SDF, please check logs.";
  }
  return absl::OkStatus();
}

absl::StatusOr<uint64_t> GazeboRunner::WaitForAtLeastIterations(
    int iterations) {
  return WaitForAtLeastIterationsUntil(iterations, absl::InfiniteFuture());
}

absl::StatusOr<uint64_t> GazeboRunner::WaitForAtLeastIterationsUntil(
    int iterations, absl::Time deadline) {
  if (!IsRunning()) {
    return absl::FailedPreconditionError("GazeboRunner is not yet running.");
  }
  INTR_RET_CHECK(gz_server_.IterationCount().has_value());
  const uint64_t start_iterations = *gz_server_.IterationCount();
  uint64_t iterations_completed = 0;
  while (iterations_completed < iterations) {
    if (!IsRunning()) {
      return absl::UnavailableError(
          "GazeboRunner was stopped before the required number of "
          "iterations could be completed.");
    }
    if (absl::Now() > deadline) {
      return DeadlineExceededErrorBuilder()
             << "Exceeded deadline after " << iterations_completed
             << " iterations.";
    }

    absl::Duration sleep_time =
        (iterations - iterations_completed) * step_size_;
    if (absl::Now() + sleep_time > deadline) {
      sleep_time = deadline - absl::Now();
    }
    absl::SleepFor(sleep_time);

    // `IterationCount()` is updated in RunLoop.
    if (*gz_server_.IterationCount() < start_iterations) {
      return absl::UnavailableError(
          "GazeboRunner was reset before the required number of iterations "
          "could be completed");
    }
    iterations_completed = *gz_server_.IterationCount() - start_iterations;
  }

  // Wait for the RunLoop to finish a step. This is required because
  // `IterationCount()` will be updated in the Gazebo server before the RunLoop
  // actually finishes a step.
  absl::MutexLock lock(sim_step_mutex_);
  finish_sim_step_.WaitWithDeadline(&sim_step_mutex_, deadline);
  return *gz_server_.IterationCount();
}

absl::Status GazeboRunner::Run(bool start_paused) {
  if (IsRunning()) {
    return absl::FailedPreconditionError(
        "Gazebo is already running, did you call Stop() before?.");
  }

  LOG(INFO) << "Starting Gazebo simulation server.";
  {
    absl::MutexLock lock(sim_step_mutex_);
    runner_paused_ = start_paused;
  }

  // We use bind front here, to create a pointer to a member function.
  // In order to support cooperative cancellation, the thread class needs a
  // function which takes a StopToken as the first argument.
  runloop_thread_ = Thread(std::bind_front(&GazeboRunner::RunLoop, this));

  if (pubsub_) {
    // Same rationale as above applies here to support cooperative cancellation.
    dds_forwarding_thread_ =
        Thread(std::bind_front(&GazeboRunner::InternalDdsForwardLoop, this));
  }

  // Start heartbeat thread to periodically monitor whether the server was
  // stepped.
  heartbeat_thread_ = Thread([this](StopToken stop_token) {
    if (heartbeat_interval_ <= absl::ZeroDuration()) {
      LOG(WARNING) << "Heartbeat interval " << heartbeat_interval_
                   << " is not positive, skipping heartbeat check.";
      return;
    }
    LOG(INFO) << "Starting heartbeat thread.";
    uint64_t iterations = 0;
    absl::Time next_heartbeat_time = absl::Now() + heartbeat_interval_;
    while (!stop_token.stop_requested() && IsRunning()) {
      // Sleep for a small duration so that we can return early if the server
      // has been stopped.
      absl::SleepFor(absl::Milliseconds(100));
      if (absl::Now() < next_heartbeat_time) {
        continue;
      }
      next_heartbeat_time = absl::Now() + heartbeat_interval_;
      if (auto new_iterations =
              gz_server_.IterationCount().value_or(iterations);
          new_iterations == iterations) {
        LOG(ERROR) << "No change in iterations after " << heartbeat_interval_
                   << ", server may be stuck.";
        LOG_IF(FATAL, absl::GetFlag(FLAGS_fatal_heartbeat_check) && IsRunning())
            << "Fatal heartbeat failure.";
      } else {
        iterations = new_iterations;
      }
    }
    LOG(INFO) << "Exiting heartbeat thread.";
  });

  // Ensure that the server was stepped once before returning.
  // TODO(b/425081968): Remove this once ICON can be brought up reliably without
  // waiting for a step here in e2e tests.
  absl::MutexLock lock(sim_step_mutex_);
  finish_sim_step_.Wait(&sim_step_mutex_);

  LOG(INFO) << "Server started successfully.";
  return absl::OkStatus();
}

bool GazeboRunner::IsRunning() const {
  // We need to be a bit careful here, since Joinable() actually means the
  // thread is or was running and noone has yet called Join() on it.
  return runloop_thread_.joinable();
}

void GazeboRunner::RunLoop(StopToken stop_token) {
  // Run until simulation is stopped
  while (!stop_token.stop_requested()) {
    // LINT.IfChange
    GZ_PROFILE("GazeboRunner::RunLoopOnce");
    // LINT.ThenChange(//intrinsic/simulation/gazebo/profiler.cc)

    bool run_paused;
    {
      absl::MutexLock lock(sim_step_mutex_);
      run_paused = runner_paused_;
    }

    // Do not hold the mutex while calling `RunOnce` to allow other threads to
    // access variables protected by the mutex.
    LOG_IF_EVERY_N_SEC(ERROR, !gz_server_.RunOnce(run_paused), 1)
        << "Failed to run the gazebo server once!";

    absl::MutexLock lock(sim_step_mutex_);
    finish_sim_step_.SignalAll();
  }
  LOG(INFO) << "Simulation RunLoop finished.";
}

void GazeboRunner::Pause() {
  absl::MutexLock lock(sim_step_mutex_);
  runner_paused_ = true;
  LOG(INFO) << "Set runner to run Gazebo paused";
}

void GazeboRunner::Unpause() {
  absl::MutexLock lock(sim_step_mutex_);
  runner_paused_ = false;
  LOG(INFO) << "Set runner to run Gazebo unpaused";
}

bool GazeboRunner::IsPaused() const {
  if (IsRunning()) {
    absl::MutexLock lock(sim_step_mutex_);
    return runner_paused_;
  } else {
    return true;
  }
}

void GazeboRunner::InternalDdsForwardLoop(StopToken stop_token) {
  absl::Time last_time_checked_topics = absl::InfinitePast();
  const absl::Duration time_between_checks = absl::Milliseconds(500);

  absl::flat_hash_set<std::string> gazebo_topics;

  gz::transport::Node node;
  while (!stop_token.stop_requested()) {
    if (absl::Now() - last_time_checked_topics < time_between_checks) {
      // Sleep for the remaining duration so that we don't spin unnecessarily.
      absl::SleepFor(last_time_checked_topics + time_between_checks -
                     absl::Now());
      continue;
    }

    // See if there have been any topics added that we need to forward to
    // internal DDS.
    std::vector<std::string> topics;
    node.TopicList(topics);

    for (const std::string& topic : std::as_const(topics)) {
      // If the observed topic is not a topic we care about, continue.
      if (!topics_to_forward_.contains(topic)) {
        continue;
      }

      if (gazebo_topics.contains(topic)) {
        continue;
      } else {
        gazebo_topics.insert(topic);
      }

      const std::string dds_topic = "/simulation/gazebo" + topic;

      CHECK(pubsub_ != nullptr);
      auto publisher = pubsub_->CreatePublisher(dds_topic, TopicConfig());
      LOG_IF(FATAL, !publisher.ok())
          << "Failed to create publisher: " << publisher.status();

      // The std::ref below is the reason why we need a node_hash_map here.
      std::function<void(const google::protobuf::Message&)> forward_fn =
          [publisher = std::make_shared<Publisher>(std::move(*publisher))](
              const google::protobuf::Message& msg) {
            auto pub_status = publisher->Publish(msg);
            LOG_IF(ERROR, !pub_status.ok())
                << "Error forwarding message on topic '"
                << publisher->TopicName() << "' from gazebo: " << pub_status;
          };

      // We can pass a temporary to Node::Subscribe() since Gazebo will
      // internally copy the callback function.
      if (!node.Subscribe(topic, forward_fn)) {
        LOG(ERROR) << "Error forwarding gazebo topic '" << topic
                   << "' to internal DDS as '" << dds_topic << "'!";
      } else {
        LOG(INFO) << "Forwarding gazebo topic '" << topic
                  << "' to internal DDS as '" << dds_topic << "'";
      }
    }

    last_time_checked_topics = absl::Now();
  }
}

void GazeboRunner::Stop() {
  LOG(INFO) << "Stopping simulation server";
  if (runloop_thread_.joinable()) {
    runloop_thread_.request_stop();
    runloop_thread_.join();
  }

  if (heartbeat_thread_.joinable()) {
    LOG(INFO) << "Joining heartbeat thread";
    heartbeat_thread_.request_stop();
    heartbeat_thread_.join();
  }

  if (dds_forwarding_thread_.joinable()) {
    dds_forwarding_thread_.request_stop();
    dds_forwarding_thread_.join();
  }
}

absl::Status GazeboRunner::AddForwardingTopic(std::string topic) {
  if (pubsub_ == nullptr) {
    return absl::FailedPreconditionError(
        "Pubsub is not available in simulation server");
  }
  topics_to_forward_.insert(std::move(topic));
  return absl::OkStatus();
}

absl::Status GazeboRunner::AddSystem(
    const std::shared_ptr<gz::sim::System>& system) {
  if (IsRunning()) {
    return absl::FailedPreconditionError(
        "Can't add system since Gazebo is already running.");
  }

  std::optional<bool> add_system_result = gz_server_.AddSystem(system);
  INTR_RET_CHECK(add_system_result.has_value() && *add_system_result);

  return absl::OkStatus();
}

absl::Status GazeboRunner::AddSystem(
    const std::shared_ptr<gz::sim::System>& system, gz::sim::Entity entity) {
  if (IsRunning()) {
    return absl::FailedPreconditionError(
        "Can't add system since Gazebo is already running.");
  }

  std::optional<bool> add_system_result =
      gz_server_.AddSystem(system, entity, /*_sdf=*/std::nullopt);
  INTR_RET_CHECK(add_system_result.has_value() && *add_system_result);
  return absl::OkStatus();
}

absl::StatusOr<uint64_t> GazeboRunner::IterationCount() const {
  std::optional<uint64_t> iteration_count = gz_server_.IterationCount();
  INTR_RET_CHECK(iteration_count.has_value());
  return *iteration_count;
}

absl::StatusOr<size_t> GazeboRunner::EntityCount() const {
  std::optional<size_t> entity_count = gz_server_.EntityCount();
  INTR_RET_CHECK(entity_count.has_value());
  return *entity_count;
}

bool GazeboRunner::HasEntity(std::string_view name) const {
  return gz_server_.HasEntity(std::string(name));
}

}  // namespace simulation
}  // namespace intrinsic
