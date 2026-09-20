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

#ifndef INTRINSIC_KINEMATICS_IK_INVERSE_KINEMATICS_INTERFACE_H_
#define INTRINSIC_KINEMATICS_IK_INVERSE_KINEMATICS_INTERFACE_H_

#include <cstddef>
#include <optional>
#include <string_view>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// An interface to request IK solutions for its underlying Chain.
// Note that while this class owns a Chain, such chain is only a view of the
// underlying Skeleton. Such Skeleton must be owned externally and outlive this
// class.
// No solvers must inherit from this class. Instead, solvers must inherit from
// the realtime and non-realtime subclasses defined below.
class InverseKinematicsInterface {
 public:
  struct Options {
    std::optional<double> frequency = std::nullopt;
  };

  explicit InverseKinematicsInterface(Chain chain) : chain_(std::move(chain)) {}
  virtual ~InverseKinematicsInterface() = default;

  // Returns the number of degrees of freedom that this interface can handle.
  virtual int GetNumDof() const = 0;

  // Return the name of the IK solver implementation behind this interface.
  virtual const std::string_view GetName() const = 0;

  // Whether the implementation provides a real-time safe method for IK
  // computation.
  virtual bool IsRealtimeSafe() const = 0;

  // Returns whether the implementation can seek solutions that are in the same
  // kinematic branch as the joint state hint.
  virtual bool ImplementsSameBranchIK() const = 0;

  // If implemented by a child class, returns the kinematic branch for the given
  // joint state.
  virtual absl::StatusOr<size_t> ComputeBranch(
      const JointStateP& joint_state) const {
    return icon::UnimplementedError(
        absl::StrCat(GetName(), " does not implement branch labeling."));
  }

  // Describes the results of the ComputeIK and RealtimeComputeIK functions.
  struct IKResult {
    enum Status {
      // Solution found and everything OK.
      OK,
      // No solution found. Some solvers may report the best effort solution.
      NO_SOLUTION,
    };
    Status status = NO_SOLUTION;
    // Number of returned solutions achieving the desired pose.
    int number_of_solutions = 0;
  };

  // Computes the joint-space positions so that the transform between the base
  // and the tip of the chain becomes 'desired_base_t_tip'.
  // This method is not guaranteed to be real-time safe. All solvers
  // need to implement this method.
  //
  // joint_state_hint:        The joint position used as a starting point for
  //                          many algorithms. Whenever possible, solvers will
  //                          attempt to provide the closest solution to this
  //                          value.
  // desired_base_t_tip:      Desired transform from tip frame to base frame.
  // limits:                  Minimum and maximum position limits for the
  //                          joints.
  // solutions:               This must be a pre-allocated Span of size n>=1,
  //                          where n the maximum number of solutions that the
  //                          solver will attempt to find. This method will fill
  //                          the first k (<=n) values of the Span with the all
  //                          the solutions found, where k is given as part of
  //                          the returned IKResult. The rest of the n-k
  //                          previously existing values in the Span will have
  //                          no meaning. Additionally, returned solutions will
  //                          be sorted from closest to furthest to
  //                          'joint_state_hint'.
  virtual absl::StatusOr<IKResult> ComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits,
      absl::Span<JointStateP> solutions) const INTRINSIC_NON_REALTIME_ONLY = 0;
  // Same as above, but only seeks one solution.
  absl::StatusOr<IKResult> ComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits,
      JointStateP* solution) const INTRINSIC_NON_REALTIME_ONLY {
    CHECK(solution != nullptr);
    return ComputeIK(hint_joint_state, desired_base_t_tip, limits,
                     absl::MakeSpan(solution, 1));
  }

  // Same as above, but seeks only one solution that is on the same
  // kinematic branch as the hint joint state.
  virtual absl::StatusOr<IKResult> ComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits, bool ensure_same_branch,
      JointStateP* solution) const INTRINSIC_NON_REALTIME_ONLY {
    if (ensure_same_branch) {
      return icon::UnimplementedError(absl::StrCat(
          GetName(), " does not implement same branch IK solution."));
    }
    return ComputeIK(hint_joint_state, desired_base_t_tip, limits, solution);
  }

  // Same as above, but guaranteed to be real-time safe. Only realtime-capable
  // solvers implement this method. Non realtime-capable solvers will raise an
  // UnimplementedError when this method is called.
  virtual icon::RealtimeStatusOr<IKResult> RealtimeComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits, absl::Span<JointStateP> solutions) const = 0;
  // Same as above, but only seeks one solution.
  icon::RealtimeStatusOr<IKResult> RealtimeComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits, JointStateP* solution) const {
    CHECK(solution != nullptr);
    return RealtimeComputeIK(hint_joint_state, desired_base_t_tip, limits,
                             absl::MakeSpan(solution, 1));
  }

  // Same as above, but seeks only one solution that is on the same
  // kinematic branch as the hint joint state.
  virtual icon::RealtimeStatusOr<IKResult> RealtimeComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits, bool ensure_same_branch,
      JointStateP* solution) const {
    if (ensure_same_branch) {
      return icon::UnimplementedError(icon::RealtimeStatus::StrCat(
          GetName(), " does not implement same branch IK solution."));
    }
    return RealtimeComputeIK(hint_joint_state, desired_base_t_tip, limits,
                             solution);
  }

  // Given a model and the id of a joint, this function finds the first ancestor
  // joint ('start_joint') to this one so that this interface can solve a Chain
  // from 'start_joint' to 'end_joint'. The function returns null_opt if no such
  // ancestor joint can be found (i.e. this solver can't solve for any chain
  // ending in 'end_joint'). Note that this function is only well defined for
  // fixed DOF solvers. Non-fixed DOF solvers always return null_opt.
  virtual std::optional<ElementId> RetrieveMatchingStartJoint(
      const ModelInterface& model, ElementId end_joint) const = 0;

  // Determines whether this interface can solve from the head to the tip of the
  // provided chain.
  virtual bool IsCompatibleWithChain(const Chain& chain) const = 0;

  // When the Inverse Kinematics (IK) solver is implementing this computation,
  // this function returns the (ground-truth) maximum arm length of the robot
  // manipulator, computed by the IK solver's extracted kinematic parameters
  // (e.g. those extracted from the robot manipulator's URDF file). The maximum
  // arm length is defined as the maximum distance between the base and the tip
  // of the robot manipulator. By default, this function is unimplemented.
  virtual absl::StatusOr<double> GetMaximumArmLength() const {
    return icon::UnimplementedError(
        absl::StrCat(GetName(), " does not implement GetMaximumArmLength()."));
  }

  const Chain& chain() const { return chain_; }

 private:
  // The chain that this interface is solving for. In particular, the operations
  // that this interface performs are specified from the base to the tip of this
  // chain. Note that a Chain is just a view of an underlying Skeleton. Such
  // Skeleton must be owned externally and outlive this class.
  Chain chain_;
};

// Real-time capable solvers must inherit from this class.
class InverseKinematicsInterfaceRT : public InverseKinematicsInterface {
 public:
  explicit InverseKinematicsInterfaceRT(Chain chain)
      : InverseKinematicsInterface(std::move(chain)) {}

  bool IsRealtimeSafe() const final { return true; }

  absl::StatusOr<IKResult> ComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits,
      absl::Span<JointStateP> solutions) const final {
    icon::RealtimeStatusOr<IKResult> result = RealtimeComputeIK(
        hint_joint_state, desired_base_t_tip, limits, solutions);
    if (result.ok()) {
      return std::move(result.value());
    } else {
      return result.status();
    }
  }

  absl::StatusOr<IKResult> ComputeIK(const JointStateP& hint_joint_state,
                                     const Pose3d& desired_base_t_tip,
                                     const JointLimits& limits,
                                     bool ensure_same_branch,
                                     JointStateP* solution) const final {
    icon::RealtimeStatusOr<IKResult> result =
        RealtimeComputeIK(hint_joint_state, desired_base_t_tip, limits,
                          ensure_same_branch, solution);
    if (result.ok()) {
      return std::move(result.value());
    } else {
      return result.status();
    }
  }
};

// Non real-time capable solvers must inherit from this class.
class InverseKinematicsInterfaceNonRT : public InverseKinematicsInterface {
 public:
  explicit InverseKinematicsInterfaceNonRT(Chain chain)
      : InverseKinematicsInterface(std::move(chain)) {}

  bool IsRealtimeSafe() const final { return false; }

  icon::RealtimeStatusOr<IKResult> RealtimeComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits,
      absl::Span<JointStateP> solutions) const final {
    return icon::UnimplementedError(
        icon::RealtimeStatus::StrCat(GetName(), " is not realtime safe."));
  }

  icon::RealtimeStatusOr<IKResult> RealtimeComputeIK(
      const JointStateP& hint_joint_state, const Pose3d& desired_base_t_tip,
      const JointLimits& limits, bool ensure_same_branch,
      JointStateP* solution) const final {
    return icon::UnimplementedError(
        icon::RealtimeStatus::StrCat(GetName(), " is not realtime safe."));
  }
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_IK_INVERSE_KINEMATICS_INTERFACE_H_
