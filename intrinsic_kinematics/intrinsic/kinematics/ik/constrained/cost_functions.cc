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

#include "intrinsic/kinematics/ik/constrained/cost_functions.h"

#include <math.h>

#include <memory>

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "absl/log/check.h"
#include "absl/log/die_if_null.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik_solvers/ik_solver_utils.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/numopt/function_linearizer_numdiff.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kinematics {

namespace {

double ComputeManipulabilityWithRegularization(const eigenmath::Matrix6Nd& J) {
  // TODO(giftthaler): The diagonal regularizer "1e-8" is required to avoid
  // instabilities near singularities. We should consider damping the
  // problematic eigenvalues of J*J.transpose() specifically in the long run.
  constexpr double kSmallRegularizerEps = 1e-8;
  eigenmath::Matrix6d JJT =
      J * J.transpose() +
      kSmallRegularizerEps * eigenmath::Matrix6d::Identity();
  return sqrt(JJT.determinant());
}

}  // namespace

JointPositionCost::JointPositionCost(const eigenmath::MatrixXd& weight,
                                     const JointStateP& x_ref)
    : x_ref_(x_ref.position), weight_(weight) {}

/*static*/
absl::StatusOr<std::unique_ptr<JointPositionCost>> JointPositionCost::Create(
    const Chain* chain, const JointStateP& x_ref, double weight) {
  const int ndof = ABSL_DIE_IF_NULL(chain)->GetNumberDegreesOfFreedom();
  if (x_ref.size() != ndof) {
    return absl::FailedPreconditionError(
        "size of reference position inconsistent");
  }
  if (weight < 0) {
    return absl::FailedPreconditionError(
        "JointPositionCost must have non-negative "
        "weight.");
  }

  // Using 'new' to access private constructor.
  return absl::WrapUnique(new JointPositionCost(
      weight * eigenmath::MatrixXd::Identity(ndof, ndof), x_ref));
}

/*static*/
absl::StatusOr<std::unique_ptr<JointPositionCost>> JointPositionCost::Create(
    const Chain* chain, const JointStateP& x_ref,
    const eigenmath::MatrixXd& weight) {
  int ndof = ABSL_DIE_IF_NULL(chain)->GetNumberDegreesOfFreedom();
  if (x_ref.size() != ndof) {
    return absl::FailedPreconditionError(
        "size of reference position inconsistent");
  }
  if (weight.cols() != ndof || weight.rows() != ndof) {
    return absl::FailedPreconditionError(
        absl::StrCat("Inconsistent dimensions of weight matrix. Expected size ",
                     ndof, " but got size ", weight.rows()));
  }
  // Validate user-defined weight-matrix.
  if (Eigen::EigenSolver<eigenmath::MatrixXd>(weight)
          .eigenvalues()
          .real()
          .array()
          .minCoeff() < 0) {
    return absl::FailedPreconditionError(
        "JointPositionCost must be positive semi-definite");
  }

  // Using 'new' to access private constructor.
  return absl::WrapUnique(new JointPositionCost(weight, x_ref));
}

absl::StatusOr<double> JointPositionCost::Evaluate(
    const eigenmath::VectorXd& joint_angles) {
  eigenmath::VectorXd dx = joint_angles - x_ref_;
  return 0.5 * (dx.transpose() * weight_ * dx)(0);
}

absl::StatusOr<eigenmath::VectorXd> JointPositionCost::Gradient(
    const eigenmath::VectorXd& joint_angles) {
  return eigenmath::VectorXd(weight_ * (joint_angles - x_ref_));
}

ManipulabilityCost::ManipulabilityCost(const Chain* chain,
                                       ElementId robot_frame_id, double weight)
    : state_(ABSL_DIE_IF_NULL(chain)),
      limits_(chain->GetDofSystemLimits()),
      weight_(weight),
      robot_frame_id_(robot_frame_id) {}

/*static*/
absl::StatusOr<std::unique_ptr<ManipulabilityCost>> ManipulabilityCost::Create(
    const Chain* chain, ElementId robot_frame_id, double weight) {
  if (weight < 0) {
    return absl::FailedPreconditionError(
        "ManipulabilityCost weight must not be negative.");
  }

  if (!ABSL_DIE_IF_NULL(chain)->GetElement(robot_frame_id).ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("ElementId ", static_cast<int>(robot_frame_id),
                     " does not exist in chain '", chain->GetName(), "."));
  }

  // Using 'new' to access private constructor.
  return absl::WrapUnique(
      new ManipulabilityCost(chain, robot_frame_id, weight));
}

absl::StatusOr<double> ManipulabilityCost::Evaluate(
    const eigenmath::VectorXd& joint_angles) {
  intrinsic::JointStateP joint_positions;
  INTR_RETURN_IF_ERROR(joint_positions.SetSize(joint_angles.size()));
  joint_positions.position = joint_angles;
  INTR_RETURN_IF_ERROR(state_.SetDofPositions(joint_positions, false));
  INTRINSIC_RT_ASSIGN_OR_RETURN(eigenmath::MatrixNMd jac,
                                state_.ComputeJacobian(robot_frame_id_));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      double joint_limit_distance,
      ComputeJointLimitDistance(limits_, joint_angles,
                                /*is_using_softmin=*/true,
                                kDefaultSoftminAlpha));

  // use exp(-x) to smoothly invert x for minimization instead of maximization.
  return weight_ * exp(-joint_limit_distance *
                       ComputeManipulabilityWithRegularization(jac));
}

absl::StatusOr<eigenmath::VectorXd> ManipulabilityCost::Gradient(
    const eigenmath::VectorXd& joint_angles) {
  return eigenmath::VectorXd(
      ComputeDerivative(
          // This lambda is required because num-diff currently does not
          // handle StatusOr.
          [this](const eigenmath::VectorXd& joint_angles) {
            auto value_or = this->Evaluate(joint_angles);
            CHECK_OK(value_or.status());
            return eigenmath::VectorXd::Constant(1, *value_or);
          },
          joint_angles,
          /*double_sided_derivative=*/false)
          .transpose()  // this transpose is required because num-diff returns a
                        // row vector.
  );
}

}  // namespace intrinsic::kinematics
