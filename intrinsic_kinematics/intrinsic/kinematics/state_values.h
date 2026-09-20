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

#ifndef INTRINSIC_KINEMATICS_STATE_VALUES_H_
#define INTRINSIC_KINEMATICS_STATE_VALUES_H_

#include "absl/log/check.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/to_fixed_string.h"

namespace intrinsic {
namespace kinematics {

struct StateValues {
  StateValues() = default;

  // Constructs a StateValues object from the provided input values. The
  // `in_dof_positions` are the input values of the DoFs, not the derived
  // values. The `joint_dependency_matrix` is the joint dependency matrix for
  // the model, which maps DoF input coordinates to DoF derived values. The
  // joint dependency matrix is square and its dimension is the number of
  // degrees of freedom. It is a lower-triangular matrix.
  StateValues(const JointStateP& in_dof_positions,
              const JointLimits& in_dof_limits,
              const eigenmath::MatrixNd& joint_dependency_matrix)
      : dof_positions(in_dof_positions),
        dof_limits(in_dof_limits),
        joint_dependency_matrix(joint_dependency_matrix) {
    int nb_dof = dof_positions.size();
    CHECK(dof_limits.IsValid()) << "Limits are invalid: "
                                << absl::string_view(ToFixedString(dof_limits));

    // Since in_dof_positions is not exceeding the size limit on JointState,
    // it's safe to only check.
    CHECK_EQ(dof_velocities.SetSize(nb_dof), icon::OkStatus());
    dof_velocities.velocity.setZero();
    CHECK_EQ(dof_accelerations.SetSize(nb_dof), icon::OkStatus());
    dof_accelerations.acceleration.setZero();
    CHECK_EQ(dof_efforts.SetSize(nb_dof), icon::OkStatus());
    dof_efforts.torque.setZero();
    // For a fully actuated system, the dependency matrix is square and its
    // dimension is the number of degrees of freedom.
    CHECK_EQ(joint_dependency_matrix.rows(), nb_dof);
    CHECK_EQ(joint_dependency_matrix.cols(), nb_dof);
  }

  // The following are DoF input values, not derived values.
  JointStateP dof_positions;
  JointStateV dof_velocities;
  JointStateA dof_accelerations;
  JointStateT dof_efforts;

  // State specific position, velocity, acceleration limits for the DoF. Often
  // referred as soft limits. These should be within the system limits of the
  // model. These limits apply to the input value of the DoF.
  JointLimits dof_limits;

  // For more efficient real-time computation of Jacobians, twists, etc, we
  // store a copy of the dependency matrix, which captures the linear dependency
  // `q_derived = A * q_input`.
  eigenmath::MatrixNd joint_dependency_matrix;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_STATE_VALUES_H_
