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

#include "intrinsic/motion_planning/path_planning/concurrent_proxy.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/stop_token.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/labels.h"

namespace intrinsic {

ConcurrentProxy::~ConcurrentProxy() {
  // We must close the queues to allow the worker threads to stop executing.
  task_queue_.Close();
  result_queue_.Close();
}

ConcurrentProxy::ConcurrentProxy(
    std::vector<std::unique_ptr<KinematicsSystemProxy>>&& proxies_in)
    : task_queue_(0), result_queue_(0), next_query_id_(0) {
  CHECK(!proxies_in.empty());
  proxies_.reserve(proxies_in.size());
  for (int ii = 0; ii < proxies_in.size(); ++ii) {
    proxies_.emplace_back(std::move(proxies_in[ii]));
  }
  workers_.reserve(proxies_.size());
  // In this loop, we create a worker thread for each proxy in proxies_. Each
  // worker thread runs in a loop until either (a) they receive a stop command
  // upon destruction of this class, or (b) the `task_queue_` or the
  // `result_queue_` is closed. Each worker thread repeatedly dequeues items
  // from the task queue, computes the validity of the task's configuration,
  // then pushes the validity result onto the result queue.
  //
  // This is modeled after //intrinsic/util/thread_pool.cc. The difference is
  // that each thread keeps a reference to its own GuardedProxy that it uses for
  // collision-checking.
  //
  // NOTE(torresl): You might wonder why we don't create each GuardedProxy
  // inside the worker thread. I tried this, but it made the code _much_ more
  // complicated.
  for (int ii = 0; ii < proxies_.size(); ++ii) {
    GuardedProxy& thread_proxy = proxies_[ii];
    workers_.emplace_back([this, &thread_proxy](StopToken stop_token) {
      while (!stop_token.stop_requested()) {
        absl::StatusOr<ValidityTask> task = task_queue_.Dequeue();
        if (!task.ok()) {
          // The task queue is closed. Good job, thread. You can go home now.
          CHECK_EQ(task.status().code(), absl::StatusCode::kUnavailable);
          return;
        }

        absl::StatusOr<JointConfigurationValidationResult> maybe_result;
        {
          absl::MutexLock lock(thread_proxy.mutex);
          maybe_result = thread_proxy.proxy->IsValid(task.value().q, nullptr);
        }
        const bool is_valid =
            maybe_result.ok() ? static_cast<bool>(maybe_result.value()) : false;
        // If validation fails, catch and propagate the error in return value.
        const absl::Status task_exit_status =
            maybe_result.ok() ? absl::OkStatus() : maybe_result.status();
        // We return the request's query_id as part of the result so that the
        // main thread knows which request this result corresponds to.
        const ValidityResult result{.query_id = task.value().query_id,
                                    .config_index = task.value().config_index,
                                    .is_valid = is_valid,
                                    .task_exit_status = task_exit_status};
        absl::Status status = result_queue_.Enqueue(result);
        if (!status.ok()) {
          // The result queue is closed. Job is done.
          CHECK_EQ(status.code(), absl::StatusCode::kUnavailable);
          return;
        }
      }
    });
  }
}

absl::StatusOr<std::vector<int>> ConcurrentProxy::FindInvalidConfigurationsImpl(
    absl::Span<const eigenmath::VectorXd> configurations,
    const std::optional<int> max_invalid_results) const {
  const uint64_t current_query_id = [this]() {
    absl::MutexLock lock(query_id_mutex_);
    return next_query_id_++;
  }();

  // Push all the input configurations onto the task queue.
  for (int ii = 0; ii < configurations.size(); ++ii) {
    const eigenmath::VectorXd& q = configurations.at(ii);
    ValidityTask task;
    task.config_index = ii;
    task.q = q;
    task.query_id = current_query_id;
    // This Enqueue() call is not expected to return an error because (1) the
    // queue cannot be closed here and (2) it cannot be full because it was
    // constructed to be unbounded. We prefer to CHECK here because it is
    // simpler to reason about the valid states of this class if we assume this
    // function always runs to completion.
    CHECK_OK(task_queue_.Enqueue(task));
  }

  const int max_invalid_results_value =
      max_invalid_results.value_or(std::numeric_limits<int>::max());

  // Now the threads are working on the tasks. We wait for results in the
  // `results_queue_` until either (a) we have seen `max_invalid_results`
  // invalid results, or (b) we have seen "valid" results for all input
  // configurations.

  // Ensure that the task queue is empty before we finish.
  // Use `absl::Cleanup` to ensure the queue is cleared even if we return
  // abruptly, e.g. due to `absl::InternalError`
  absl::Cleanup queue_cleaner = [this] { task_queue_.Clear(); };

  // NOTE: By the time the task queue is cleared, it's possible that there are
  // workers still working on their validity checks. This is fine; they can
  // finish asynchronously and push their results onto the `results_queue_`. The
  // next time we call AreAllValid, we will use the query_id to discard any of
  // these leftover results.

  int results_seen = 0;
  std::vector<int> invalid_indices;

  while (results_seen < configurations.size()) {
    if (invalid_indices.size() >= max_invalid_results_value) break;
    INTR_ASSIGN_OR_RETURN(
        const ValidityResult result,
        result_queue_.Dequeue(/*timeout=*/absl::Seconds(30)),
        // We do not expect this to ever happen; but if it does, return an error
        // instead of potentially halting execution indefinitely.
        _ << "Unexpected timeout waiting for validity results");

    if (result.query_id != current_query_id) {
      // This is a leftover result from a previous call. Ignore it.
      continue;
    }
    ++results_seen;

    // If a task failed to perform the validity check, immediately propagate to
    // the caller.
    if (!result.task_exit_status.ok()) {
      return result.task_exit_status;
    }

    if (!result.is_valid) {
      invalid_indices.push_back(result.config_index);
    }
  }

  std::sort(invalid_indices.begin(), invalid_indices.end());
  return invalid_indices;
}

absl::StatusOr<std::vector<int>> ConcurrentProxy::FindInvalidConfigurations(
    absl::Span<const eigenmath::VectorXd> configurations,
    const std::optional<int> max_invalid_results) const {
  return FindInvalidConfigurationsImpl(configurations, max_invalid_results);
}

absl::StatusOr<bool> ConcurrentProxy::AreAllValid(
    absl::Span<const eigenmath::VectorXd> configurations,
    int* invalid_index) const {
  INTR_ASSIGN_OR_RETURN(
      const std::vector<int> invalid_indices,
      FindInvalidConfigurationsImpl(configurations, /*max_invalid_results=*/1));
  if (!invalid_indices.empty()) {
    if (invalid_index != nullptr) {
      *invalid_index = invalid_indices.front();
    }
    return false;
  }
  return true;
}

absl::StatusOr<bool> ConcurrentProxy::AreConstraintsSatisfied(
    const eigenmath::VectorXd& configuration) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->AreConstraintsSatisfied(configuration);
}

absl::StatusOr<eigenmath::VectorXd>
ConcurrentProxy::ProjectOntoConstraintManifolds(
    const eigenmath::VectorXd& configuration, size_t max_projection_steps,
    double max_constraint_error_norm) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->ProjectOntoConstraintManifolds(
      configuration, max_projection_steps, max_constraint_error_norm);
}

absl::StatusOr<JointConfigurationValidationResult> ConcurrentProxy::IsValid(
    const eigenmath::VectorXd& configuration,
    CollisionCheckingDebug* collision_debug) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->IsValid(configuration, collision_debug);
}

absl::StatusOr<bool> ConcurrentProxy::IsWithinLimits(
    const eigenmath::VectorXd& configuration) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->IsWithinLimits(configuration);
}

std::vector<DofLabel> ConcurrentProxy::GetDofLabels() const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetDofLabels();
}

std::pair<PhysicalEntityId, PhysicalEntityId>
ConcurrentProxy::GetBaseAndTipIds() const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetBaseAndTipIds();
}

absl::StatusOr<Pose3d> ConcurrentProxy::GetFk(
    const eigenmath::VectorXd& configuration,
    const LabelId& named_location) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetFk(configuration, named_location);
}

absl::StatusOr<Pose3d> ConcurrentProxy::GetFkInWorld(
    const eigenmath::VectorXd& configuration,
    const LabelId& named_location) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetFkInWorld(configuration, named_location);
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> ConcurrentProxy::GetIk(
    const Pose3d& pose) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetIk(pose);
}

absl::StatusOr<std::vector<eigenmath::VectorXd>> ConcurrentProxy::GetIk(
    const Pose3d& pose, const eigenmath::VectorXd& seed) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetIk(pose, seed);
}

JointLimitsXd ConcurrentProxy::GetJointLimits() const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetJointLimits();
}

std::shared_ptr<CollisionChecker> ConcurrentProxy::GetCollisionChecker() const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetCollisionChecker();
}

void ConcurrentProxy::SetDistanceCheckStatistics(
    DistanceCheckStatistics* const statistics) {
  // DistanceCheckStatistics is not thread-safe. Multiple worker threads
  // mutating the same statistics object will result in data races and undefined
  // behavior. Therefore, we disallow collecting statistics on concurrent
  // proxies.
  if (statistics != nullptr) {
    LOG(WARNING) << "Ignoring non-null DistanceCheckStatistics pointer on "
                    "ConcurrentProxy because DistanceCheckStatistics is not "
                    "thread-safe.";
    return;
  }
  for (GuardedProxy& proxy : proxies_) {
    absl::MutexLock lock(proxy.mutex);
    proxy.proxy->SetDistanceCheckStatistics(nullptr);
  }
}

// This function is the main motivation for having each proxy be guarded by its
// own Mutex. It's theoretically possible that a worker thread is still
// computing a "leftover" validity check when this function gets called, so the
// mutex ensures that we don't modify a proxy while it's in use.
absl::Status ConcurrentProxy::SetJointLimits(
    const JointLimitsXd& joint_limits) {
  for (GuardedProxy& proxy : proxies_) {
    absl::MutexLock lock(proxy.mutex);
    INTR_RETURN_IF_ERROR(proxy.proxy->SetJointLimits(joint_limits));
  }
  return absl::OkStatus();
}

absl::StatusOr<eigenmath::VectorXd> ConcurrentProxy::GetRandomConfig(
    int* seed) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetRandomConfig(seed);
}

absl::StatusOr<eigenmath::VectorXd> ConcurrentProxy::GetRandomConfig(
    int* seed, const eigenmath::VectorXd& lower_limits,
    const eigenmath::VectorXd& upper_limits) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetRandomConfig(seed, lower_limits,
                                                 upper_limits);
}

absl::StatusOr<eigenmath::VectorXd> ConcurrentProxy::GetRandomConfigNear(
    const eigenmath::VectorXd& config, double distance, int* seed) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetRandomConfigNear(config, distance, seed);
}

absl::StatusOr<eigenmath::VectorXd> ConcurrentProxy::GetRandomConfigNear(
    const eigenmath::VectorXd& configuration,
    const eigenmath::VectorXd& distance, int* seed) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetRandomConfigNear(configuration, distance,
                                                     seed);
}

absl::StatusOr<std::string> ConcurrentProxy::PrintCollisionCheckingDebug(
    const CollisionCheckingDebug& collision_debug) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->PrintCollisionCheckingDebug(collision_debug);
}

absl::StatusOr<intrinsic_proto::motion_planning::v1::CollisionDebug>
ConcurrentProxy::GetCollisionDebugMessage(
    const CollisionCheckingDebug& collision_debug) const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetCollisionDebugMessage(collision_debug);
}

absl::StatusOr<const std::vector<std::unique_ptr<ConstraintInterface>>*>
ConcurrentProxy::GetConstraints() const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetConstraints();
}

absl::StatusOr<std::unique_ptr<icon::ManipulatorKinematics>>
ConcurrentProxy::GetManipulatorKinematics() const {
  absl::MutexLock lock(proxies_.front().mutex);
  return proxies_.front().proxy->GetManipulatorKinematics();
}

}  // namespace intrinsic
