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

#ifndef INTRINSIC_SIMULATION_GAZEBO_GAZEBO_RUNNER_H_
#define INTRINSIC_SIMULATION_GAZEBO_GAZEBO_RUNNER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_set.h"
#include "absl/flags/declare.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/System.hh"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/util/thread/stop_token.h"
#include "intrinsic/util/thread/thread.h"

ABSL_DECLARE_FLAG(bool, fatal_heartbeat_check);

namespace intrinsic {
namespace simulation {
// GazeboRunner class wraps a Gazebo server instance and provides a control
// interface for the server.
class GazeboRunner {
 public:
  // Defined in the SDF 1.12 spec.
  // https://github.com/gazebosim/sdformat/blob/ed018d8758f475c3f0ec8247a5616b380f564856/sdf/1.12/physics.sdf#L17
  static constexpr absl::Duration kSdfDefaultStepSize = absl::Milliseconds(1);

  // Create a new `GazeboRunner` instance.
  // The input pubsub instance should outlive this server instance. If nullptr
  // is passed, topics cannot be forwarded from the simulator transport to
  // platform pubsub.
  // Returns `AbortedError` if the server could not be configured as requested.
  static absl::StatusOr<std::unique_ptr<GazeboRunner>> Create(
      const gz::sim::ServerConfig& config, PubSub* pubsub = nullptr,
      absl::Duration step_size = kSdfDefaultStepSize,
      absl::Duration heartbeat_interval = absl::Seconds(30));

  virtual ~GazeboRunner();

  // Wait for at least a specified number of simulation iterations to be
  // completed. Returns the number of iterations completed since simulation
  // start.
  virtual absl::StatusOr<uint64_t> WaitForAtLeastIterations(int iterations);

  // Wait for at least a specified number of simulation iterations to be
  // completed with a deadline. Returns the number of iterations completed since
  // simulation start. If the deadline is reached before the required number of
  // iterations are completed, a DeadlineExceededError is returned.
  virtual absl::StatusOr<uint64_t> WaitForAtLeastIterationsUntil(
      int iterations, absl::Time deadline);

  // Get simulation time step size.
  virtual absl::Duration TimeStepSize() const { return step_size_; }

  // Run the simulation loop. Non-blocking call.
  // Returns FailedPreconditionError if the server is already running.
  virtual absl::Status Run(bool start_paused);

  virtual bool IsRunning() const;

  // Stop the simulation runloop if it is running.
  virtual void Stop();

  // Pause the simulation.
  // - If the gazebo server is already running, it continues to run but
  // simulation time does not increase.
  // - If the gazebo server is not running, then this call is a no-op (when
  // `Run()` is called, the passed value for `start_paused` will be used).
  virtual void Pause();

  // Unpause the simulation.
  // - If the gazebo server is running paused, it continues to run and the
  // simulation time increases.
  // - If the gazebo server is not running, then this call is a no-op (when
  // `Run()` is called, the passed value for `start_paused` will be used).
  virtual void Unpause();

  // If `IsRunning()` is false, this method will always return true.
  // If gazebo server is running, returns whether it is paused.
  virtual bool IsPaused() const;

  // Configures the server to forward the given topic from Gazebo's internal
  // transport layer to our internal DDS. These topics are forwarded using a
  // thread that monitors when new topics are advertised on Gazebo's transport
  // layer.
  absl::Status AddForwardingTopic(std::string topic);

  // Add a system to the Gazebo server under the world entity.
  // Returns FailedPreconditionError if the server is already running.
  // Returns InternalError if the system could not be added.
  virtual absl::Status AddSystem(
      const std::shared_ptr<gz::sim::System>& system);

  // Add a system to the Gazebo server for the specified entity.
  // Returns FailedPreconditionError if the server is already running.
  // Returns InternalError if the system could not be added.
  virtual absl::Status AddSystem(const std::shared_ptr<gz::sim::System>& system,
                                 gz::sim::Entity entity);

  // Get the current iteration count of the Gazebo server.
  // Returns InternalError if the iteration count is not available.
  virtual absl::StatusOr<uint64_t> IterationCount() const;

  // Get the current entity count of the Gazebo server.
  // Returns InternalError if the entity count is not available.
  virtual absl::StatusOr<size_t> EntityCount() const;

  // Check if the server has a named entity.
  virtual bool HasEntity(std::string_view name) const;

 protected:
  explicit GazeboRunner(const gz::sim::ServerConfig& config, PubSub* pubsub,
                        absl::Duration heartbeat_interval,
                        absl::Duration step_size);

  // Called after construction in the `Create()` factory method to verify that
  // the Gazebo server was initialized successfully.
  // Derived classes must call `INTR_RETURN_IF_ERROR(InitStatus())` after
  // construction either in their own factory method or call
  // `CHECK_OK(InitStatus())` in their constructor for tests.
  absl::Status InitStatus();

 private:
  void RunLoop(StopToken stop_token);

  void InternalDdsForwardLoop(StopToken stop_token);

  // Gazebo server instance
  gz::sim::Server gz_server_;

  // Mutex to synchronize simulation stepping with external control.
  mutable absl::Mutex sim_step_mutex_;

  // Conditional variable to notify that simulation has been stepped once
  absl::CondVar finish_sim_step_ ABSL_GUARDED_BY(sim_step_mutex_);

  // Whether the simulation was commanded to run paused.
  bool runner_paused_ ABSL_GUARDED_BY(sim_step_mutex_) = false;

  Thread runloop_thread_;

  Thread heartbeat_thread_;

  const absl::Duration heartbeat_interval_;

  // We maintain a thread that will constantly monitor Gazebo's internal
  // topic namespaces and forward them to our internal DDS service. This thread
  // has the same lifetime as the runloop_thread_ above.
  Thread dds_forwarding_thread_;

  absl::flat_hash_set<std::string> topics_to_forward_;

  // The PubSub instance is not owned by the simulation server.
  PubSub* pubsub_;

  // Step size of the simulation. This is assigned during world to sdf
  // conversion and assumed to be fixed for the lifetime of this simulation.
  const absl::Duration step_size_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_GAZEBO_RUNNER_H_
