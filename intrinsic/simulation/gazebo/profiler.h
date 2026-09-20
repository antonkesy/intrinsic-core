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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PROFILER_H_
#define INTRINSIC_SIMULATION_GAZEBO_PROFILER_H_

#include <pthread.h>

#include <cstdint>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/flags/declare.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "gz/common/ProfilerImpl.hh"
#include "intrinsic/simulation/gazebo/proto/introspection.pb.h"

ABSL_DECLARE_FLAG(uint64_t, cycles_to_compute_sim_performance);

namespace intrinsic {
namespace simulation {

namespace internal {
// Forward declare internal class.
class CallTimeAccumulator;
}  // namespace internal

// This class implements a Gazebo Profiler that computes cumulative statistics
// for a set of profile names hardcoded in the source file. If verbose logging
// with `--v=1` is enabled, the class will periodically log the cumulative
// statistics computed over an interval set by the
// `--cycles_to_compute_sim_performance` flag.
// The class is limited to profiling calls on the simulation server runloop
// thread.
// There are a few limitations on the methods that can be profiled:
// - The method should not be recursive.
// - The method should be scoped under a single parent method that is also
//   tracked in the profiler.
// - Only methods that are profiled with GZ_PROFILE are supported. Code blocks
//   that are profiled with GZ_PROFILE_BEGIN and GZ_PROFILE_END are not
//   supported.

class Profiler final : public gz::common::ProfilerImpl {
 public:
  // Create a new Profiler and register it with the Gazebo singleton profiler.
  // Note that the pointer is owned by the Gazebo singleton profiler class.
  // So if the call returns successfully, the pointer stays valid until the
  // program terminates.
  static absl::StatusOr<Profiler*> CreateAndRegisterProfiler(
      absl::Duration simulation_step_size = absl::ZeroDuration());

  ~Profiler() final;

  std::string Name() const final { return "SimServerProfiler"; }

  void SetThreadName(const char* name) final;

  void LogText(const char* text) final {}

  void BeginSample(const char* name, uint32_t* hash) final;

  void EndSample() final;

  // Update the simulation step size used to compute RTF.
  void SetSimulationStepSize(absl::Duration simulation_step_size);

  // Reset the accumulated times, particularly when the simulation server itself
  // is reset.
  // Note: Reset() should only be called when the simulation server is not
  // running as it is not thread-safe. Calling it when the server is running can
  // result in undefined behavior.
  void Reset();

  // Get the most up-to-date simulator performance metrics.
  absl::StatusOr<intrinsic_proto::simulation::SimulatorPerformanceMeasures>
  GetPerformanceMeasures() const;

  // Human readable string containing last generated profile snapshot (generated
  // over cycles_to_compute_sim_performance cycles).
  std::string DebugString() const;

  // Human readable profile string for just the last sim cycle.
  std::string LastCycleDebugString() const;

 private:
  Profiler(uint update_interval_in_cycles, absl::Duration simulation_step_size,
           pthread_key_t thread_key);

  // Number of sim runloop cycles to wait before updating the cumulative
  // statistics.
  const uint64_t update_interval_in_cycles_;

  // Size of a single simulation step.
  absl::Duration simulation_step_size_ = absl::ZeroDuration();

  // Key previously created by pthread_key_create. Used to store thread-specific
  // data.
  const pthread_key_t thread_key_;

  // ----------- Internal tracking methods and data structures -----------
  // The class uses several data structures to efficiently track call times
  // for the given profile names across multiple threads.

  // ---- Call ID ----
  // Each profile name is assigned a unique call ID that is stored at the call
  // site and passed to BeginSample.
  absl::flat_hash_map<std::string, uint32_t> call_name_to_id_;

  // Call ID for the sim runloop profile name.
  uint32_t sim_runloop_call_id_ = 0;

  // ---- Call time accumulators ----
  // A CallTimeAccumulator tracks the cumulative statistics for a single tracked
  // profile name on a single thread. The corresponding accumulator is updated
  // in the BeginSample and EndSample methods.
  std::vector<internal::CallTimeAccumulator> call_time_accumulators_;

  // Mutex used to synchronize adding accumulators to call_time_accumulators_.
  // Call_time_accumulators_ is not annotated with
  // ABSL_GUARDED_BY(call_time_accumulators_mutex_) because it accessed without
  // holding the mutex in the BeginSample and EndSample methods for performance.
  absl::Mutex call_time_accumulators_mutex_;

  // Reset the statistics tracked in each accumulator.
  void ResetAllCallTimeAccumulators();

  // An accumulator handle is a pair of call ID and thread ID.
  typedef std::pair<uint32_t, uint32_t> AccumulatorHandle;
  absl::flat_hash_map<AccumulatorHandle, uint32_t> accumulator_handle_to_id_;

  // Get the index of the accumulator for the given call ID and thread ID.
  // If the accumulator does not exist, it is created.
  uint32_t GetOrCreateAccumulatorIndex(uint32_t call_id, uint32_t thread_id,
                                       absl::string_view profile_name);

  // This version assumes that the accumulator already exists.
  uint32_t GetAccumulatorIndex(uint32_t call_id, uint32_t thread_id) const;

  // Map of all accumulators associated with a given call ID (across all
  // threads). Used to aggregate statistics from accumulators for a particular
  // profile name across all threads (e.g. postupdate calls for multiple
  // instances of the same System).
  absl::flat_hash_map<uint32_t, std::vector<uint32_t>>
      call_id_to_accumulator_indices_;

  // ---- Call stacks ----
  // A call stack is used to track the calls received in BeginSample in LIFO
  // order. Each call stack is associated with a particular thread name
  // annotated with GZ_PROFILE_THREAD_NAME (and not a thread instance).
  struct CallStackEntry {
    uint32_t call_id;

    // Accumulator index is only set if the call is tracked.
    uint32_t accumulator_index;
  };

  typedef std::stack<CallStackEntry, std::vector<CallStackEntry>> CallStack;

  std::vector<CallStack> call_stacks_;

  // Mutex used to synchronize adding call stacks. Call_stacks_ is not
  // annotated with ABSL_GUARDED_BY(call_stacks_mutex_) because it accessed
  // without holding the mutex in the BeginSample and EndSample methods for
  // performance.
  absl::Mutex call_stacks_mutex_;

  // Creates a call stack in call_stacks_ and returns its index.
  // Call stacks are only ever added to the profiler, never removed. So the
  // index is stable.
  // The index for the call stack associated with a particular thread is also
  // used as its thread ID.
  uint32_t CreateCallStack() ABSL_EXCLUSIVE_LOCKS_REQUIRED(call_stacks_mutex_);

  // Index of the call stack for the sim runloop thread.
  uint32_t sim_runloop_thread_stack_index_ = 0;

  // ---- Miscellaneous data for computing call statistics ----
  // Mapping of parent call ID to child call IDs. A std::vector is used to
  // preserve the order in which child calls were added. This makes the profiler
  // debug strings more readable.
  std::vector<std::pair<uint32_t, std::vector<uint32_t>>> child_profile_calls_;

  // Number of runloop cycles since last update.
  uint64_t cycle_count_since_update_ = 0;

  // Number of runloop cycles since last reset.
  uint64_t cycle_count_since_reset_ = 0;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PROFILER_H_
