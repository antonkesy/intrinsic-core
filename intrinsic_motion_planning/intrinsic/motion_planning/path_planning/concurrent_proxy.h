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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_CONCURRENT_PROXY_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_CONCURRENT_PROXY_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/world_kinematics_system_proxy.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_planning_error.pb.h"
#include "intrinsic/util/thread/concurrent_queue.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/configuration_validation.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/grouping.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// This class implements the typical KinematicsSystemProxy interface, but
// manages an internal thread pool to perform multithreaded computation of
// AreAllValid and FindInvalidConfigurations. All other methods are
// computed without multithreading.
//
// In order to enable multithreaded validity checking, we store a separate
// GuardedProxy for each thread to use.
class ConcurrentProxy : public KinematicsSystemProxy {
 public:
  // Instantiate a thread pool with thread_count == proxies_in.size(). Requires
  // proxies_in to be non-empty.
  explicit ConcurrentProxy(
      std::vector<std::unique_ptr<KinematicsSystemProxy>>&& proxies_in);
  ~ConcurrentProxy() override;

  absl::StatusOr<bool> AreAllValid(
      absl::Span<const eigenmath::VectorXd> configurations,
      int* invalid_index) const override;

  // Checks concurrently across the thread pool for invalid configurations.
  // Returns a vector containing the indices of the invalid configurations.
  // If `max_invalid_results` is provided, it terminates early after finding
  // that many invalid configurations.
  absl::StatusOr<std::vector<int>> FindInvalidConfigurations(
      absl::Span<const eigenmath::VectorXd> configurations,
      std::optional<int> max_invalid_results = std::nullopt) const override;

  absl::StatusOr<JointConfigurationValidationResult> IsValid(
      const eigenmath::VectorXd& configuration,
      CollisionCheckingDebug* collision_debug) const override;
  absl::StatusOr<bool> IsWithinLimits(
      const eigenmath::VectorXd& configuration) const override;
  std::vector<DofLabel> GetDofLabels() const override;
  std::pair<PhysicalEntityId, PhysicalEntityId> GetBaseAndTipIds()
      const override;
  absl::StatusOr<Pose3d> GetFk(const eigenmath::VectorXd& configuration,
                               const LabelId& named_location) const override;
  absl::StatusOr<Pose3d> GetFkInWorld(
      const eigenmath::VectorXd& configuration,
      const LabelId& named_location) const override;
  absl::StatusOr<std::vector<eigenmath::VectorXd>> GetIk(
      const Pose3d& pose) const override;
  absl::StatusOr<std::vector<eigenmath::VectorXd>> GetIk(
      const Pose3d& pose, const eigenmath::VectorXd& seed) const override;
  JointLimitsXd GetJointLimits() const override;
  absl::Status SetJointLimits(const JointLimitsXd& joint_limits) override;
  std::shared_ptr<CollisionChecker> GetCollisionChecker() const override;
  // Note: DistanceCheckStatistics is not thread-safe. ConcurrentProxy ignores
  // non-null statistics pointers to prevent data races across worker threads.
  void SetDistanceCheckStatistics(DistanceCheckStatistics* statistics) override;
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfigNear(
      const eigenmath::VectorXd& config, double distance,
      int* seed) const override;
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfigNear(
      const eigenmath::VectorXd& configuration,
      const eigenmath::VectorXd& distance, int* seed) const override;
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfig(int* seed) const override;
  absl::StatusOr<eigenmath::VectorXd> GetRandomConfig(
      int* seed, const eigenmath::VectorXd& lower_limits,
      const eigenmath::VectorXd& upper_limits) const override;
  absl::StatusOr<std::string> PrintCollisionCheckingDebug(
      const CollisionCheckingDebug& collision_debug) const override;
  absl::StatusOr<intrinsic_proto::motion_planning::v1::CollisionDebug>
  GetCollisionDebugMessage(
      const CollisionCheckingDebug& collision_debug) const override;
  absl::StatusOr<const std::vector<std::unique_ptr<ConstraintInterface>>*>
  GetConstraints() const override;
  absl::StatusOr<bool> AreConstraintsSatisfied(
      const eigenmath::VectorXd& configuration) const override;
  absl::StatusOr<eigenmath::VectorXd> ProjectOntoConstraintManifolds(
      const eigenmath::VectorXd& configuration, size_t max_projection_steps,
      double max_constraint_error_norm) const override;
  absl::StatusOr<std::unique_ptr<icon::ManipulatorKinematics>>
  GetManipulatorKinematics() const override;

  ConcurrentProxy(ConcurrentProxy&& other) = delete;
  ConcurrentProxy& operator=(ConcurrentProxy&& other) = delete;
  ConcurrentProxy() = delete;
  ConcurrentProxy(const ConcurrentProxy& other) = delete;
  ConcurrentProxy& operator=(const ConcurrentProxy& other) = delete;

 private:
  // Helper method that checks concurrently across the thread pool for invalid
  // configurations and returns their indices. If `max_invalid_results` is
  // provided, it terminates early after finding that many invalid
  // configurations.
  absl::StatusOr<std::vector<int>> FindInvalidConfigurationsImpl(
      absl::Span<const eigenmath::VectorXd> configurations,
      std::optional<int> max_invalid_results) const;

  // Represents a single configuration validation task to be processed by a
  // worker thread.
  struct ValidityTask {
    // The index of this configuration in the input path/span.
    int config_index = -1;

    // The joint configuration vector to validate.
    eigenmath::VectorXd q;

    // When AreAllValid is called, all ValidityTasks and ValidityResults for
    // that call will have the same unique query_id. When we pull results from
    // the result_queue_, there may be leftover results in the queue from a
    // previous call to AreAllValid; we use the query_id to filter out
    // leftover results from previous calls.
    uint64_t query_id;
  };

  // Holds the result of a configuration validation task computed by a worker
  // thread.
  struct ValidityResult {
    // The query ID associated with the task.
    uint64_t query_id;

    // The index of the validated configuration in the input path/span.
    int config_index = -1;

    // True if the joint configuration is valid.
    bool is_valid;

    // The exit status of the collision checking task.
    absl::Status task_exit_status;
  };
  // The main thread pushes ValidityTasks onto the task_queue_. Worker threads
  // pull tasks from task_queue_, compute results, and push those results onto
  // the result_queue_.
  mutable ConcurrentQueue<ValidityTask> task_queue_;
  mutable ConcurrentQueue<ValidityResult> result_queue_;
  // A monotonically increasing unique ID we use to associate
  // ValidityTasks/ValidityResponses with a single AreAllValid call.
  mutable uint64_t next_query_id_ ABSL_GUARDED_BY(query_id_mutex_);
  mutable absl::Mutex query_id_mutex_;

  // Wrapper around KinematicsSystemProxy that protects access to the proxy with
  // a mutex.
  struct GuardedProxy {
    std::unique_ptr<KinematicsSystemProxy> proxy ABSL_GUARDED_BY(mutex);
    mutable absl::Mutex mutex;

    explicit GuardedProxy(std::unique_ptr<KinematicsSystemProxy>&& proxy)
        : proxy(std::move(proxy)) {}
    GuardedProxy(GuardedProxy&& other) : proxy(std::move(other.proxy)) {}
  };
  // We store one GuardedProxy for each worker thread.
  std::vector<GuardedProxy> proxies_;

  // NOTE: the worker threads are declared LAST so that they get destroyed
  // FIRST. This is important because the threads access the above members; if
  // any of the above members gets destroyed before the worker threads, then the
  // workers will access deleted memory.
  std::vector<Thread> workers_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_CONCURRENT_PROXY_H_
