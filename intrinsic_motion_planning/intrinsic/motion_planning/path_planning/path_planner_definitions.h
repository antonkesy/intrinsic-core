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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_PLANNER_DEFINITIONS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_PLANNER_DEFINITIONS_H_

#include <functional>
#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/kinematics/ik_solvers/kinematic_chain/kinematic_chain_newton_raphson_ik_solver.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {

constexpr double kSquaredConnectingDistance = 1e-2;

// A threshold used to assess whether two joint configurations coincide.
constexpr double kJointDeviationToleranceRad = 1e-3;

using ForwardKinematicsFunction =
    std::function<absl::StatusOr<Pose3d>(const eigenmath::VectorXd&)>;
using InverseKinematicsFunction =
    std::function<absl::StatusOr<eigenmath::VectorXd>(
        const Pose3d&, const eigenmath::VectorXd&)>;
using GeometricJacobianFunction =
    std::function<absl::StatusOr<eigenmath::Matrix6Nd>(
        const eigenmath::VectorXd&)>;

struct CartesianKinematicsComponents {
  // If `manipulator_kinematics` is not a nullptr, then the functions
  // `forward_kinematics_function`, `inverse_kinematics_function` and
  // `geometric_jacobian_function` will use it to compute the forward
  // kinematics, the inverse kinematics, and the geometric Jacobian
  // respectively.
  std::unique_ptr<icon::ManipulatorKinematics> manipulator_kinematics = nullptr;

  // If `singularity_robust_ik_solver` is not a nullptr, then it will be used
  // to compute inverse kinematics in the proximity of kinematic
  // singularities.
  std::unique_ptr<kinematics::KinematicChainRandomSeedIKSolver>
      singularity_robust_ik_solver = nullptr;

  // If `joint_limits` is not a nullptr, then it will be used to compute inverse
  // kinematics.
  std::unique_ptr<JointLimits> joint_limits = nullptr;

  ForwardKinematicsFunction forward_kinematics_function;
  InverseKinematicsFunction inverse_kinematics_function;
  GeometricJacobianFunction geometric_jacobian_function;
};

using PointPath = std::vector<intrinsic::eigenmath::VectorXd>;

// Definition of a validator that checks a single point. It is common for
// implementations to construct their own validators which wrap additional
// checks on top of the validity check provided by the proxy.
using PointValidator =
    std::function<absl::StatusOr<bool>(const eigenmath::VectorXd&)>;

// Definition of a validator that can check either a single point or an edge.
// Like the case of the `PointValidator`, implementations will typically use
// functions of this form since it abstracts the details of a particular edge
// check through variable binding.
using EdgeValidator = std::function<absl::StatusOr<bool>(
    const eigenmath::VectorXd&, const eigenmath::VectorXd&)>;

// Definition of a validator that can check a path.
// Like the case of the `EdgeValidator`, implementations will typically use
// functions of this form since it abstracts the details of a particular path
// check through variable binding.
using PathValidator = std::function<absl::StatusOr<bool>(const PointPath&)>;

// Definition of a projector function which projects a configuration into a
// configuration which satisfies all constraints, i.e. lie on constraint
// manifolds. For the usage, please refer to
// go/intrinsic-path-constrained-sbmp-improvement-design and
// go/intrinsic-path-constrained-sbmp-improvement-presentation for details.
using ConstraintManifoldProjector =
    std::function<absl::StatusOr<eigenmath::VectorXd>(
        const eigenmath::VectorXd&)>;

// Definition of a sampling function that is used as an abstraction to pass
// to the individual planning tools.
using GetConfigFn = std::function<absl::StatusOr<eigenmath::VectorXd>()>;

// Definition of a distance measure that defines the distance between two
// configurations. Depending on the nearest neighborhood data structure used,
// this measure might have to satisfy metric properties (i.e.,
// (1) non-negativity,
// (2) identity of indiscernibles: d(x,y)= 0 <=> x = y
// (3) symmetry
// (4) triangle inequality: d(x,z) <= d(x,y) + d(y,z).
using DistanceFn = std::function<double(const eigenmath::VectorXd& state1,
                                        const eigenmath::VectorXd& state2)>;

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PATH_PLANNER_DEFINITIONS_H_
