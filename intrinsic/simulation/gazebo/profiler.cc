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

#include "intrinsic/simulation/gazebo/profiler.h"

#include <pthread.h>

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <iomanip>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/text_format.h"
#include "gz/common/Profiler.hh"
#include "intrinsic/simulation/gazebo/proto/introspection.pb.h"
#include "intrinsic/stats/metrics_utils.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/status_macros.h"
#include "opentelemetry/context/runtime_context.h"
#include "opentelemetry/metrics/meter.h"
#include "opentelemetry/metrics/sync_instruments.h"

ABSL_FLAG(uint64_t, cycles_to_compute_sim_performance, 500,
          "Number of simulation cycles over which to average the simulator "
          "performance metrics");

namespace intrinsic {
namespace simulation {

namespace {
constexpr absl::string_view kRealTimeFactorInstrumentName =
    "intrinsic/simulation_service/real_time_factor";

opentelemetry::metrics::Histogram<double>& RealTimeFactorHistogram() {
  static opentelemetry::metrics::Histogram<double>* histogram =
      stats::GetMeter()
          ->CreateDoubleHistogram(kRealTimeFactorInstrumentName,
                                  "Ratio of sim to wall clock time.", "ratio")
          .release();
  return *histogram;
}

absl::StatusOr<pthread_key_t> PthreadCreateKey() {
  pthread_key_t key;
  if (pthread_key_create(&key, /*__destr_function=*/nullptr) != 0) {
    return absl::InternalError("Failed to create pthread key.");
  }
  return key;
}
}  // namespace

namespace internal {

class CallTimeAccumulator {
 public:
  explicit CallTimeAccumulator(absl::string_view name);
  // Resets total and running count but leaves average and count at last update
  // unchanged.
  void Reset();
  void StartCall();
  void EndCall();
  void UpdateAverage();
  std::string Name() const { return name_; }
  absl::Duration Average() const { return average_; }
  absl::Duration Total() const { return total_; }
  absl::Time LastCallStart() const { return last_call_start_; }
  absl::Time LastCallEnd() const { return last_call_end_; }
  absl::Duration LastCallDuration() const { return last_call_duration_; }
  uint64_t RunningCount() const { return running_count_; }
  uint64_t CountAtLastUpdate() const { return count_at_last_update_; }

 private:
  const std::string name_;
  absl::Time start_ = absl::InfiniteFuture();
  absl::Duration total_ = absl::ZeroDuration();
  absl::Duration average_ = absl::ZeroDuration();
  uint64_t running_count_ = 0;
  uint64_t count_at_last_update_ = 0;
  absl::Time last_call_start_ = absl::InfiniteFuture();
  absl::Time last_call_end_ = absl::InfinitePast();
  absl::Duration last_call_duration_ = absl::ZeroDuration();
};

CallTimeAccumulator::CallTimeAccumulator(absl::string_view name)
    : name_(name) {}

void CallTimeAccumulator::Reset() {
  start_ = absl::InfiniteFuture();
  total_ = absl::ZeroDuration();
  last_call_start_ = absl::InfiniteFuture();
  last_call_end_ = absl::InfinitePast();
  last_call_duration_ = absl::ZeroDuration();
  running_count_ = 0;
}

void CallTimeAccumulator::StartCall() { start_ = absl::Now(); }

void CallTimeAccumulator::EndCall() {
  if (start_ == absl::InfiniteFuture()) {
    return;
  }
  last_call_start_ = start_;
  last_call_end_ = absl::Now();
  last_call_duration_ = last_call_end_ - last_call_start_;
  total_ += last_call_duration_;
  start_ = absl::InfiniteFuture();
  running_count_++;
}

void CallTimeAccumulator::UpdateAverage() {
  if (running_count_ == 0) {
    average_ = absl::ZeroDuration();
  } else {
    average_ = total_ / running_count_;
  }
  count_at_last_update_ = running_count_;
}

uint64_t TotalCountAtLastUpdate(
    absl::Span<const CallTimeAccumulator> accumulators,
    absl::Span<const uint32_t> indices) {
  uint64_t total_count_at_last_update = 0;
  for (uint32_t index : indices) {
    if (index < accumulators.size()) {
      total_count_at_last_update += accumulators[index].CountAtLastUpdate();
    }
  }
  return total_count_at_last_update;
}

absl::Duration AverageDuration(
    const std::vector<CallTimeAccumulator>& accumulators,
    const std::vector<uint32_t>& indices) {
  absl::Duration total_duration = absl::ZeroDuration();
  uint64_t total_count = 0;
  for (uint32_t index : indices) {
    if (index < accumulators.size()) {
      total_duration += accumulators[index].Average() *
                        accumulators[index].CountAtLastUpdate();
      total_count += accumulators[index].CountAtLastUpdate();
    }
  }
  if (total_count == 0) {
    return absl::ZeroDuration();
  } else {
    return total_duration / total_count;
  }
}

}  // namespace internal

namespace {
using ParentCallToChildCallsMap =
    std::vector<std::pair<uint32_t, std::vector<uint32_t>>>;

// ----- Names of profiler call names that will be tracked in the profiler -----
// If you add an entry to here, make sure to also update kParentProfileCallMap
// below and possibly ProfilerTest.HasExpectedProfileNames.
constexpr char kGazeboRunnerRunLoopProfileName[] = "GazeboRunner::RunLoopOnce";

constexpr char kSimulationRunnerStepProfileName[] =
    "SimulationRunner::Run - Iteration";

constexpr char kSimulationRunnerStepPreupdateProfileName[] = "PreUpdate";

constexpr char kSimulationRunnerStepUpdateProfileName[] = "Update";

constexpr char kSimulationRunnerStepPostupdateProfileName[] = "PostUpdate";

constexpr char kPhysicsUpdateProfileName[] = "Physics::Update";

constexpr char kForceTorqueUpdateProfileName[] = "ForceTorque::Update";

constexpr char kSceneBroadcasterPostUpdateProfileName[] =
    "SceneBroadcaster::PostUpdate";

constexpr char kSensorsPostUpdateProfileName[] = "Sensors::PostUpdate";

constexpr char kObjectInteractionModeratorPostUpdateProfileName[] =
    "ObjectInteractionModerator::PostUpdate";

constexpr char kWorldSyncSystemPostUpdateProfileName[] =
    "WorldSyncSystem::PostUpdate";

constexpr char kDeviceContainerPluginPostUpdateProfileName[] =
    "DeviceContainerPlugin::PostUpdate";

constexpr char kForceTorqueDevicePluginPostUpdateProfileName[] =
    "ForceTorqueDevicePlugin::PostUpdate";

constexpr char kActuatedGripperPluginPostUpdateProfileName[] =
    "ActuatedGripperPlugin::PostUpdate";

constexpr char kUserCommandsPreUpdateProfileName[] = "UserCommands::PreUpdate";

// -----------------------------------------------------------------------------

// Dummy call ID assigned to profile names which are not tracked in the
// profiler. A zero value cannot be used since it is the default value of the
// call hash at each site.
constexpr uint32_t kUntrackedCallId = 1;

ParentCallToChildCallsMap BuildChildCallIdMap(
    absl::Span<const std::pair<std::string_view, std::string_view>>
        parent_profile_names_map,
    const absl::flat_hash_map<std::string, uint32_t>& call_ids) {
  ParentCallToChildCallsMap child_map;
  for (auto& [child, parent] : parent_profile_names_map) {
    if (parent.empty()) {
      continue;
    }

    uint32_t parent_id = call_ids.at(parent);
    uint32_t child_id = call_ids.at(child);
    auto it = std::find_if(
        child_map.begin(), child_map.end(),
        [parent_id](const std::pair<uint32_t, std::vector<uint32_t>>& p) {
          return p.first == parent_id;
        });
    if (it == child_map.end()) {
      child_map.emplace_back(parent_id,
                             std::initializer_list<uint32_t>{child_id});
    } else {
      it->second.push_back(child_id);
    }
  }
  return child_map;
}

void PrintPerformanceMeasures(
    const intrinsic_proto::simulation::SimulatorPerformanceMeasures& measures,
    std::string* output) {
  google::protobuf::TextFormat::Printer printer;
  google::protobuf::io::StringOutputStream stream(output);
  if (!printer.Print(measures, &stream)) {
    absl::StrAppend(output, "Error: Failed to serialize performance measures");
  }
}

}  // namespace

absl::StatusOr<Profiler*> Profiler::CreateAndRegisterProfiler(
    absl::Duration simulation_step_size) {
  stats::RegisterHistogramView(kRealTimeFactorInstrumentName,
                               /*view_name=*/"",
                               // Hand-selected bucket sizes to get a reasonable
                               // distribution from 0 to 1.
                               stats::LinearBucketBoundaries(10, 0.0, 0.1));
  // Eagerly initialize histogram state to avoid hot-path latency.
  RealTimeFactorHistogram();

  INTR_ASSIGN_OR_RETURN(pthread_key_t thread_key, PthreadCreateKey());
  auto* profiler =
      new Profiler(absl::GetFlag(FLAGS_cycles_to_compute_sim_performance),
                   simulation_step_size, thread_key);
#if GZ_PROFILER_ENABLE
  if (!gz::common::Profiler::Instance()->SetImplementation(
          absl::WrapUnique(profiler))) {
    return absl::InternalError("Failed to register profiler implementation.");
  }
#else
  LOG(WARNING) << "GZ_PROFILER is disabled. Performance profiling will not be "
                  "available. To enable profiling, run with "
                  "--@gz-common//profiler:config=custom";
#endif
  return profiler;
}

Profiler::Profiler(uint update_interval_in_cycles,
                   absl::Duration simulation_step_size,
                   pthread_key_t thread_key)
    : update_interval_in_cycles_(update_interval_in_cycles),
      simulation_step_size_(simulation_step_size),
      thread_key_(thread_key) {
  // Map of parent profile calls for each call that we are tracking.
  // kGazeboRunnerRunLoopProfileName is the top level call, so there isn't
  // an entry for it.
  // The profile names that are tracked in the profiler are currently
  // hard-coded in kParentProfileCallMap. Profile begin/end calls for other
  // profile names are ignored.
  const std::vector<std::pair<std::string_view, std::string_view>>
      kParentProfileCallMap = {
          {kGazeboRunnerRunLoopProfileName, ""},
          // kGazeboRunnerRunLoopProfileName sub-calls
          {kSimulationRunnerStepProfileName, kGazeboRunnerRunLoopProfileName},
          // kSimulationRunnerStepProfileName sub-calls
          {kSimulationRunnerStepPreupdateProfileName,
           kSimulationRunnerStepProfileName},
          {kSimulationRunnerStepUpdateProfileName,
           kSimulationRunnerStepProfileName},
          {kSimulationRunnerStepPostupdateProfileName,
           kSimulationRunnerStepProfileName},
          // PreUpdate calls start here
          {kUserCommandsPreUpdateProfileName,
           kSimulationRunnerStepPreupdateProfileName},
          // Update calls start here
          {kPhysicsUpdateProfileName, kSimulationRunnerStepUpdateProfileName},
          {kForceTorqueUpdateProfileName,
           kSimulationRunnerStepUpdateProfileName},
          // PostUpdate calls start here
          {kSceneBroadcasterPostUpdateProfileName,
           kSimulationRunnerStepPostupdateProfileName},
          {kSensorsPostUpdateProfileName,
           kSimulationRunnerStepPostupdateProfileName},
          {kObjectInteractionModeratorPostUpdateProfileName,
           kSimulationRunnerStepPostupdateProfileName},
          {kWorldSyncSystemPostUpdateProfileName,
           kSimulationRunnerStepPostupdateProfileName},
          {kDeviceContainerPluginPostUpdateProfileName,
           kSimulationRunnerStepPostupdateProfileName},
          {kForceTorqueDevicePluginPostUpdateProfileName,
           kSimulationRunnerStepPostupdateProfileName},
          {kActuatedGripperPluginPostUpdateProfileName,
           kSimulationRunnerStepPostupdateProfileName}};

  call_time_accumulators_.reserve(100);

  // An incremental scheme is used to assign call IDs to profile names. Call IDs
  // up to kUntrackedCallId cannot be used.
  uint32_t call_id = kUntrackedCallId + 1;
  for (const auto& [child, parent] : kParentProfileCallMap) {
    call_name_to_id_[child] = call_id;
    call_id_to_accumulator_indices_.insert({call_id, {}});
    call_id++;
  }
  sim_runloop_call_id_ = call_name_to_id_[kGazeboRunnerRunLoopProfileName];

  child_profile_calls_ =
      BuildChildCallIdMap(kParentProfileCallMap, call_name_to_id_);

  // Initialize the call stack for the runloop thread.
  absl::MutexLock lock(call_stacks_mutex_);
  call_stacks_.reserve(100);
  sim_runloop_thread_stack_index_ = CreateCallStack();
}

Profiler::~Profiler() = default;

uint32_t Profiler::CreateCallStack() {
  // Reserve memory for stack vector to avoid run-time allocations.
  std::vector<CallStackEntry> stack_vector;
  stack_vector.reserve(100);
  call_stacks_.emplace_back(std::move(stack_vector));
  return call_stacks_.size() - 1;
}

void Profiler::SetThreadName(const char* /*name*/) {}

uint32_t Profiler::GetOrCreateAccumulatorIndex(uint32_t call_id,
                                               uint32_t thread_id,
                                               absl::string_view profile_name) {
  AccumulatorHandle handle(call_id, thread_id);
  if (auto it = accumulator_handle_to_id_.find(handle);
      it != accumulator_handle_to_id_.end()) {
    return it->second;
  }

  CHECK(!profile_name.empty());

  absl::MutexLock lock(call_time_accumulators_mutex_);
  call_time_accumulators_.emplace_back(profile_name);
  const uint32_t accumulator_index = call_time_accumulators_.size() - 1;
  accumulator_handle_to_id_[handle] = accumulator_index;
  call_id_to_accumulator_indices_[call_id].push_back(accumulator_index);

  return accumulator_index;
}

uint32_t Profiler::GetAccumulatorIndex(uint32_t call_id,
                                       uint32_t thread_id) const {
  return accumulator_handle_to_id_.at({call_id, thread_id});
}

void Profiler::BeginSample(const char* name, uint32_t* hash) {
  uint32_t call_id;
  if (hash == nullptr) {
    call_id = kUntrackedCallId;
  } else {
    // Set the hash value the first time this call is made to the call ID. It is
    // persisted as described here:
    // https://github.com/gazebosim/gz-common/blob/gz-common6/profiler/include/gz/common/Profiler.hh.
    if (*hash == 0) {
      if (auto it = call_name_to_id_.find(name); it != call_name_to_id_.end()) {
        *hash = it->second;
      } else {
        *hash = kUntrackedCallId;
      }
    }
    call_id = *hash;
  }

  void* thread_data = pthread_getspecific(thread_key_);
  uint32_t stack_id;
  if (thread_data != nullptr) {
    stack_id = *(uint32_t*)(thread_data);
  } else if (call_id == sim_runloop_call_id_) {
    stack_id = sim_runloop_thread_stack_index_;
    pthread_setspecific(thread_key_, &sim_runloop_thread_stack_index_);
  } else {
    return;
  }

  call_stacks_[stack_id].push({.call_id = call_id});

  if (call_id == kUntrackedCallId) {
    return;
  }

  uint32_t acc_index = GetOrCreateAccumulatorIndex(call_id, stack_id, name);
  call_stacks_[stack_id].top().accumulator_index = acc_index;
  call_time_accumulators_[acc_index].StartCall();
}

void Profiler::EndSample() {
  void* thread_data = pthread_getspecific(thread_key_);
  uint32_t stack_id;
  if (thread_data != nullptr) {
    stack_id = *(uint32_t*)(thread_data);
  } else {
    return;
  }

  CallStackEntry entry = call_stacks_[stack_id].top();
  call_stacks_[stack_id].pop();

  if (entry.call_id == kUntrackedCallId) {
    return;
  }

  call_time_accumulators_[entry.accumulator_index].EndCall();

  // EndSample for SimulationServerRunLoopProfile denotes the end of a full
  // simulation cycle.
  if (entry.call_id == sim_runloop_call_id_) {
    cycle_count_since_update_++;
    cycle_count_since_reset_++;

    // Log if the last cycle took very long.
    constexpr absl::Duration kVeryLongCycleDuration = absl::Seconds(1);
    if (call_time_accumulators_[entry.accumulator_index].LastCallDuration() >
        kVeryLongCycleDuration) {
      LOG(WARNING) << LastCycleDebugString();
    }

    if (cycle_count_since_update_ >= update_interval_in_cycles_) {
      for (auto& it : call_time_accumulators_) {
        it.UpdateAverage();
      }

      cycle_count_since_update_ = 0;

      absl::StatusOr<intrinsic_proto::simulation::SimulatorPerformanceMeasures>
          perf_measures = GetPerformanceMeasures();
      if (perf_measures.ok()) {
        std::string print_output = "Simulation performance snapshot: \n";
        PrintPerformanceMeasures(*perf_measures, &print_output);
        absl::StrAppend(&print_output, DebugString());
        VLOG_EVERY_N_SEC(1, 1) << print_output;
      }

      ResetAllCallTimeAccumulators();
    }
  }
}

void Profiler::ResetAllCallTimeAccumulators() {
  for (auto& it : call_time_accumulators_) {
    it.Reset();
  }
}

void Profiler::SetSimulationStepSize(absl::Duration simulation_step_size) {
  simulation_step_size_ = simulation_step_size;
}

void Profiler::Reset() {
  LOG(INFO) << "Resetting simulation profiler.";
  cycle_count_since_update_ = 0;
  cycle_count_since_reset_ = 0;
  call_time_accumulators_.clear();
  accumulator_handle_to_id_.clear();

  for (auto& it : call_id_to_accumulator_indices_) {
    it.second.clear();
  }
}

absl::StatusOr<intrinsic_proto::simulation::SimulatorPerformanceMeasures>
Profiler::GetPerformanceMeasures() const {
  // TODO(b/210407124) Compute and populate other sim performance metrics

  if (cycle_count_since_reset_ < update_interval_in_cycles_) {
    return absl::UnavailableError(
        "Insufficient cycles to accurately measure sim performance.");
  }

  intrinsic_proto::simulation::SimulatorPerformanceMeasures measures;
  absl::Duration average_sim_runloop_time_ =
      call_time_accumulators_[GetAccumulatorIndex(
                                  sim_runloop_call_id_,
                                  sim_runloop_thread_stack_index_)]
          .Average();
  measures.set_real_time_factor(
      absl::FDivDuration(simulation_step_size_, average_sim_runloop_time_));
  measures.set_cycles_since_last_reset(cycle_count_since_reset_);
  INTR_RETURN_IF_ERROR(FromAbslDuration(simulation_step_size_,
                                        measures.mutable_cycle_timestep()));
  measures.set_cycles_profiled(update_interval_in_cycles_);

  RealTimeFactorHistogram().Record(
      measures.real_time_factor(),
      opentelemetry::context::RuntimeContext::GetCurrent());

  return measures;
}

std::string Profiler::DebugString() const {
  // LINT.IfChange
  if (cycle_count_since_reset_ < update_interval_in_cycles_) {
    return "";
  }

  std::stringstream ss;
  ss << std::setprecision(4);
  auto get_average = [this](uint32_t id) -> absl::Duration {
    return AverageDuration(call_time_accumulators_,
                           call_id_to_accumulator_indices_.at(id));
  };
  auto get_count = [this](uint32_t id) -> uint64_t {
    return TotalCountAtLastUpdate(call_time_accumulators_,
                                  call_id_to_accumulator_indices_.at(id));
  };
  auto get_name = [this](uint32_t id) -> std::string {
    const internal::CallTimeAccumulator& acc =
        call_time_accumulators_[call_id_to_accumulator_indices_.at(id)[0]];
    return acc.Name();
  };

  ss << "Total cycle time: "
     << absl::FDivDuration(get_average(sim_runloop_call_id_),
                           absl::Milliseconds(1))
     << "ms \n";

  for (auto& [parent_hash, child_hashes] : child_profile_calls_) {
    absl::Duration children_total;
    ss << get_name(parent_hash) << " breakdown: \n";
    absl::Duration parent_average = get_average(parent_hash);
    uint64_t parent_count = get_count(parent_hash);

    for (auto& child_hash : child_hashes) {
      if (!call_id_to_accumulator_indices_.contains(child_hash) ||
          call_id_to_accumulator_indices_.at(child_hash).empty()) {
        continue;
      }

      absl::Duration child_average = get_average(child_hash);
      uint64_t child_count = get_count(child_hash);
      if (parent_count > 0) {
        double multiplier = static_cast<double>(child_count) / parent_count;
        child_average *= multiplier;
      }
      children_total += child_average;
      double ratio = absl::FDivDuration(child_average, parent_average);
      ss << "-- " << get_name(child_hash) << ": "
         << absl::FDivDuration(child_average, absl::Milliseconds(1)) << "ms ("
         << ratio * 100 << "%) (count: " << child_count << ")\n";
    }

    absl::Duration other = parent_average - children_total;
    double other_ratio = absl::FDivDuration(other, parent_average);
    ss << "-- others : " << absl::FDivDuration(other, absl::Milliseconds(1))
       << "ms (" << other_ratio * 100 << "%)\n";
  }
  return ss.str();
  // LINT.ThenChange(//intrinsic/simulation/tools/plot_sim_metrics.py)
}

std::string Profiler::LastCycleDebugString() const {
  if (sim_runloop_call_id_ == 0) {
    // The profiler has yet to run for a full sim cycle, return an empty string.
    return "";
  }

  const auto print_duration = [](std::ostream& os, absl::Duration d) {
    os << absl::IDivDuration(d, absl::Seconds(1), &d) << "s"
       << absl::IDivDuration(d, absl::Milliseconds(1), &d) << "ms.";
    // Ignore remaining duration below 1ms.
  };

  const internal::CallTimeAccumulator& runloop_acc =
      call_time_accumulators_[GetAccumulatorIndex(
          sim_runloop_call_id_, sim_runloop_thread_stack_index_)];
  const absl::Time cycle_start = runloop_acc.LastCallStart();
  std::stringstream ss;
  ss << "Last sim cycle (#" << runloop_acc.RunningCount() << ") took ";
  print_duration(ss, runloop_acc.LastCallDuration());
  ss << " Breakdown:";
  for (auto& accumulator : call_time_accumulators_) {
    // Ignore profile names which were not called in this cycle.
    if (accumulator.RunningCount() > 0 &&
        accumulator.LastCallStart() > cycle_start) {
      ss << " " << accumulator.Name() << ": ";
      print_duration(ss, accumulator.LastCallDuration());
    }
  }
  return ss.str();
}

}  // namespace simulation
}  // namespace intrinsic
