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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_KINEMATICS_SYSTEM_PROXY_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_KINEMATICS_SYSTEM_PROXY_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_checker.h"
#include "intrinsic/world/collision/collision_types.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/labels.h"

namespace intrinsic {
namespace kinematics_system_proxy_internal {

struct eigen_hash {
  size_t operator()(const eigenmath::VectorXd& value) const {
    return absl::HashOf(VectorXdToVector(value));
  }
};

struct eigen_equals {
  bool operator()(const eigenmath::VectorXd& lhs,
                  const eigenmath::VectorXd& rhs) const {
    return lhs == rhs;
  }
};

}  // namespace kinematics_system_proxy_internal

enum struct WithinLimitsStatus {
  kUnchecked = 0,
  kViolatedLimits = 1,
  kWithinLimits = 2,
};

inline std::ostream& operator<<(std::ostream& out,
                                const WithinLimitsStatus& status) {
  switch (status) {
    case WithinLimitsStatus::kUnchecked:
      out << "Unchecked";
      break;
    case WithinLimitsStatus::kViolatedLimits:
      out << "Violated Limits";
      break;
    case WithinLimitsStatus::kWithinLimits:
      out << "Within Limits";
      break;
  }
  return out;
}

enum struct ConstraintSatisfactionStatus {
  kUnchecked = 0,
  kViolatedConstraints = 1,
  kAllConstraintsSatisfied = 2,
};

inline std::ostream& operator<<(std::ostream& out,
                                const ConstraintSatisfactionStatus& status) {
  switch (status) {
    case ConstraintSatisfactionStatus::kUnchecked:
      out << "Unchecked";
      break;
    case ConstraintSatisfactionStatus::kViolatedConstraints:
      out << "Violated Constraints";
      break;
    case ConstraintSatisfactionStatus::kAllConstraintsSatisfied:
      out << "All Constraints are Satisfied";
      break;
  }
  return out;
}

struct JointConfigurationValidationResult {
  MarginPairConflictStatus collision_status =
      MarginPairConflictStatus::kUnchecked;
  WithinLimitsStatus within_limits_status = WithinLimitsStatus::kUnchecked;
  ConstraintSatisfactionStatus constraint_satisfaction_status =
      ConstraintSatisfactionStatus::kUnchecked;

  explicit operator bool() const {
    return (collision_status == MarginPairConflictStatus::kClear) &&
           (within_limits_status == WithinLimitsStatus::kWithinLimits) &&
           (constraint_satisfaction_status ==
            ConstraintSatisfactionStatus::kAllConstraintsSatisfied);
  }

  bool operator==(const JointConfigurationValidationResult& other) const {
    return ((other.collision_status == this->collision_status) &&
            (other.within_limits_status == this->within_limits_status) &&
            (other.constraint_satisfaction_status ==
             this->constraint_satisfaction_status));
  }
};

inline std::ostream& operator<<(
    std::ostream& out, const JointConfigurationValidationResult& status) {
  out << "collision_status: " << status.collision_status
      << ", within_limits_status: " << status.within_limits_status
      << ", constraint_satisfaction_status: "
      << status.constraint_satisfaction_status << ".";
  return out;
}

// Interface through which a PathPlanner can run forward and inverse kinematics
// (FK/IK) and query whether a particular configuration is within limits and/or
// valid (collision-free usually). It is intended to abstract the representation
// of the world from the information that a PathPlanner will care about.
class KinematicsSystemProxy {
 public:
  static constexpr size_t kDefaultMaxProjectionSteps = 1000;
  static constexpr double kDefaultMaxConstraintErrorNorm = 0.05;

  virtual ~KinematicsSystemProxy() = default;

  // Returns true if the given configuration fulfills all specified constraints
  // defined upon construction of the proxy. At a minimum this includes if the
  // configuration is within limits. Additional constraints are: if the
  // configuration is (i) collision free, i.e., collision rules were specified
  // and (ii) satisfying the geometric constraints. If non-null,
  // `collision_debug` is filled with collision checking debug info.
  virtual absl::StatusOr<JointConfigurationValidationResult> IsValid(
      const eigenmath::VectorXd& configuration,
      CollisionCheckingDebug* collision_debug) const = 0;

  // Returns true if all given configurations are found to be valid by this
  // proxy. If this returns false, the out-param `invalid_index` (if supplied)
  // will contain the index of an invalid configuration. This is not necessarily
  // the first invalid configuration in the path (for example, if an override
  // implements this method by multithreading, the invalid index returned may be
  // non-deterministic).
  virtual absl::StatusOr<bool> AreAllValid(
      absl::Span<const eigenmath::VectorXd> configurations,
      int* invalid_index = nullptr) const {
    for (int ii = 0; ii < configurations.size(); ++ii) {
      const eigenmath::VectorXd& q = configurations.at(ii);
      INTR_ASSIGN_OR_RETURN(JointConfigurationValidationResult result,
                            IsValid(q, /*collision_debug=*/nullptr));
      bool is_valid = static_cast<bool>(result);
      if (!is_valid) {
        if (invalid_index != nullptr) {
          *invalid_index = ii;
        }
        return false;
      }
    }
    return true;
  }

  // Returns a vector containing the indices of the invalid configurations in
  // the input `configurations` span. If `max_invalid_results` is provided, it
  // stops after finding `max_invalid_results` invalid configurations.
  // Otherwise, it finds all of them.
  //
  // Note: When using a concurrent proxy implementation with
  // `max_invalid_results`, the returned indices are not guaranteed to be the
  // strictly lowest indices due to non-deterministic thread completion order.
  virtual absl::StatusOr<std::vector<int>> FindInvalidConfigurations(
      absl::Span<const eigenmath::VectorXd> configurations,
      std::optional<int> max_invalid_results = std::nullopt) const;

  // Returns true if the given configuration is within limits.
  virtual absl::StatusOr<bool> IsWithinLimits(
      const eigenmath::VectorXd& configuration) const = 0;

  // Gets all the dof labels associated to proxy.
  virtual std::vector<DofLabel> GetDofLabels() const = 0;

  // Gets the robot id.
  virtual std::pair<PhysicalEntityId, PhysicalEntityId> GetBaseAndTipIds()
      const = 0;

  // Returns the pose of the point `named_location` with respect to the base
  // frame in the given `configuration`.
  virtual absl::StatusOr<Pose3d> GetFk(const eigenmath::VectorXd& configuration,
                                       const LabelId& named_location) const = 0;

  // Returns the pose of the origin of the frame labeled `named_location` in the
  // kinematic chain of the robot with respect to the world frame in the given
  // `configuartion`.
  virtual absl::StatusOr<Pose3d> GetFkInWorld(
      const eigenmath::VectorXd& configuration,
      const LabelId& named_location) const = 0;

  // Returns inverse kinematics solutions for the specified pose in the
  // reference frame of the base of the system. The returned solutions should
  // be within limits (as determined by IsWithinLimits). If there are no
  // solutions within limits the function should return an empty list.
  virtual absl::StatusOr<std::vector<eigenmath::VectorXd>> GetIk(
      const Pose3d& pose) const = 0;
  // Same as above but allows the user to pass a seed joint configuration. The
  // underlying implementation should try to return ik solutions close to this
  // seed, and sort solutions based on their closeness.
  virtual absl::StatusOr<std::vector<eigenmath::VectorXd>> GetIk(
      const Pose3d& pose, const eigenmath::VectorXd& seed) const {
    return absl::UnimplementedError("GetIk with seed is not supported.");
  }

  // Sets the joint position application limits of the robot.
  virtual absl::Status SetJointLimits(const JointLimitsXd& joint_limits) = 0;

  // Returns the joint limits of the robot, where the first set of joint values
  // are the minimal values a joint can have and the second set of joint values
  // are the maximal values a joint can have.
  //
  // Due to self collision, even if a configuration is within this limits, it
  // does not mean that the configuration is valid. A configuration within
  // limits therefore always need to be tested with isValid.
  virtual JointLimitsXd GetJointLimits() const = 0;

  // Returns the collision checker associated with the proxy, or nullptr if
  // collision checking is disabled.
  virtual std::shared_ptr<CollisionChecker> GetCollisionChecker() const = 0;

  // Sets distance check statistics aggregation on the underlying collision
  // checker, if present.
  //
  // NOTE: When collision checking is disabled on this proxy (or on a specific
  // motion segment's local proxy), `GetCollisionChecker()` returns nullptr.
  // This method safely guards against nullptr dereferences and no-ops when
  // collision checking is disabled.
  virtual void SetDistanceCheckStatistics(DistanceCheckStatistics* statistics);

  // Returns a random configuration for which IsWithinLimits will return true
  // (though IsValid may not). This call should be fast, i.e. not use collision
  // checks since it is expected that some implementation will call this
  // function many times in tight loops to generate candidate configurations.
  //
  // The implementation should mutate `seed` so that is is appropriate to call
  // this function again without the caller needing to do extra work to modify
  // it.
  //
  // No strong guarantees are made about the sampling distribution, but in
  // general, implementation should try to make this approximate uniform in the
  // space.
  virtual absl::StatusOr<eigenmath::VectorXd> GetRandomConfig(
      int* seed) const = 0;

  virtual absl::StatusOr<eigenmath::VectorXd> GetRandomConfig(
      int* seed, const eigenmath::VectorXd& lower_limits,
      const eigenmath::VectorXd& upper_limits) const = 0;

  // Returns a random configuration that is near by config within the +/- the
  // given distance and for which the IsWithinLimits will return true (though
  // IsValid may not).
  virtual absl::StatusOr<eigenmath::VectorXd> GetRandomConfigNear(
      const eigenmath::VectorXd& config, double distance, int* seed) const = 0;

  // Returns a random configuration that is near by config within a given
  // range where the distance to the configuration is defined for each degree of
  // freedom individually. The returned configuration IsWithinLimits though
  // IsValid might not return true.
  virtual absl::StatusOr<eigenmath::VectorXd> GetRandomConfigNear(
      const eigenmath::VectorXd& configuration,
      const eigenmath::VectorXd& distance, int* seed) const = 0;

  // Returns a string describing `collision_debug` which includes the detected
  // collisions.
  virtual absl::StatusOr<std::string> PrintCollisionCheckingDebug(
      const CollisionCheckingDebug& collision_debug) const = 0;

  // Returns the collision debug proto for a given collision
  virtual absl::StatusOr<intrinsic_proto::motion_planning::v1::CollisionDebug>
  GetCollisionDebugMessage(
      const CollisionCheckingDebug& collision_debug) const = 0;

  // Returns the constraints associated with the proxy.
  virtual absl::StatusOr<
      const std::vector<std::unique_ptr<ConstraintInterface>>*>
  GetConstraints() const = 0;

  virtual absl::StatusOr<bool> AreConstraintsSatisfied(
      const eigenmath::VectorXd& configuration) const = 0;

  // Perform an (iterative) projection of the configuration onto the constraint
  // manifolds, in order to obtain the closest configuration that satisfies all
  // constraints. The parameter `max_constraint_error_norm` governs how small
  // each projection step is performed, to take into account the locality
  // assumption of a Jacobian matrix, i.e. the aggregated constraint Jacobian.
  // Throws `kInternalError` if the projection does not converge within
  // `max_projection_steps` iterations. For the usage, please refer to
  // go/intrinsic-path-constrained-sbmp-improvement-design and
  // go/intrinsic-path-constrained-sbmp-improvement-presentation for details.
  virtual absl::StatusOr<eigenmath::VectorXd> ProjectOntoConstraintManifolds(
      const eigenmath::VectorXd& configuration, size_t max_projection_steps,
      double max_constraint_error_norm) const = 0;

  // ProjectOntoConstraintManifolds() with default parameters
  // max_projection_steps = 1000 and max_constraint_error_norm = 0.05.
  absl::StatusOr<eigenmath::VectorXd> ProjectOntoConstraintManifolds(
      const eigenmath::VectorXd& configuration) const {
    return ProjectOntoConstraintManifolds(
        configuration, /*max_projection_steps=*/kDefaultMaxProjectionSteps,
        /*max_constraint_error_norm=*/kDefaultMaxConstraintErrorNorm);
  }

  // TODO(aabyshk): b/282792254
  // Returns a ManipulatorKinematics object from the robot base to tip.
  virtual absl::StatusOr<std::unique_ptr<icon::ManipulatorKinematics>>
  GetManipulatorKinematics() const {
    return icon::UnimplementedError(
        "KinematicsSystemProxy implementation does not provide a "
        "ManipulatorKinematics object.");
  }

  struct JointConfigurationValidationData {
    JointConfigurationValidationResult result;
    std::vector<CollisionCheckingDebug::Collision> collisions;
  };

  // TODO(stoyang): Do we need a mutex here?
  mutable absl::flat_hash_map<eigenmath::VectorXd,
                              JointConfigurationValidationData,
                              kinematics_system_proxy_internal::eigen_hash,
                              kinematics_system_proxy_internal::eigen_equals>
      seen_points_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_KINEMATICS_SYSTEM_PROXY_H_
