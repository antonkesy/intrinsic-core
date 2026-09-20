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

#include "intrinsic/math/numopt/nlopt_nonlinear_program.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/numopt/costfunction_interface.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"
#include "magic_enum/magic_enum.hpp"
#include "nlopt.hpp"

namespace intrinsic {

namespace {

// Cost function wrapper for evaluating cost functions using NLOpts C-interface.
// Evaluates cost, and optionally evaluates the cost gradient. 'n' is the number
// of optimization variables, 'x' points to the optimization variable vector,
// 'grad' points to the first-order cost gradient, and 'problem_context' is a
// void pointer allowing us to hand over arbitrary data, in this case the
// 'CostContext'.
double NLOptCostWrapper(unsigned n, const double* x, double* grad,
                        void* problem_context) {
  auto context = static_cast<nlopt_internal::CostContext*>(problem_context);

  // Map variable to Eigen-type.
  Eigen::Map<const eigenmath::VectorXd> eigen_x(x, n);

  // Explicitly use `.array().isFinite()` to avoid template resolution errors
  // with `.allFinite()` on `Eigen::Map`.
  if (!eigen_x.array().isFinite().all()) {
    // Force stop if the search diverged and produced non-finite variables.
    nlopt_force_stop(context->Opt());
    return -1.0;
  }

  if (grad) {
    // Map gradient vectors to Eigen-types.
    Eigen::Map<eigenmath::VectorXd> cost_derivative(grad, n);
    if (const auto gradient_or = context->Cost()->EvaluateCostGradient(eigen_x);
        gradient_or.ok() && gradient_or.value().array().isFinite().all()) {
      cost_derivative = gradient_or.value();
    } else {
      // Force stop NLOpt in case of evaluation error or non-finite gradient.
      nlopt_force_stop(context->Opt());
      return -1.0;
    }
  }

  if (const auto cost_or = context->Cost()->EvaluateCostFunction(eigen_x);
      !cost_or.ok() || !std::isfinite(*cost_or)) {
    // Force stop NLOpt in case of evaluation error or non-finite cost.
    nlopt_force_stop(context->Opt());
    // Returning some value is still required in case of failure.
    return -1.0;
  } else {
    return *cost_or;
  }
}

// Constraint wrapper for evaluating constraint residuals and gradients using
// NLOpts C-interface. 'n' is the number of optimization variables, 'm' is the
// dimension of the constraint residual, 'x' points to the optimization variable
// vector, 'grad' points to the first-order constraint gradient, and
// 'problem_context' is a void pointer allowing us to hand over arbitrary data,
// in this case the 'ConstraintContext'.
void NLOptConstraintWrapper(unsigned m, double* residual, unsigned n,
                            const double* x, double* grad,
                            void* problem_context) {
  auto* context =
      static_cast<nlopt_internal::ConstraintContext*>(problem_context);

  // Map variable to Eigen-types.
  Eigen::Map<const eigenmath::VectorXd> eigen_x(x, n);

  // Explicitly use `.array().isFinite()` to avoid template resolution errors
  // with `.allFinite()` on `Eigen::Map`.
  if (!eigen_x.array().isFinite().all()) {
    // Force stop if the search diverged and produced non-finite variables.
    nlopt_force_stop(context->Opt());
    return;
  }

  if (grad) {
    // The n dimension of grad is stored contiguously, so that \partci/\partxj
    // is stored in grad[i*n + j] Here you see take dCi/dx0...dxn and store it
    // one by one, then repeat. grad is just an one dimensional array

    // Compute the constraint derivative, at the same time flip to row-major
    // representation before transcribing, to comply with NLOpts definition of
    // "stacked" gradient matrices.
    if (const auto gradient_or = context->Constraint()->Gradient(eigen_x);
        gradient_or.ok() && gradient_or.value().array().isFinite().all()) {
      Eigen::Map<eigenmath::VectorXd> eigen_grad(grad, n * m);
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
          constrained_derivative_mat_row_major = *std::move(gradient_or);

      Eigen::Map<Eigen::RowVectorXd> constraint_derivative_vectorized(
          constrained_derivative_mat_row_major.data(),
          constrained_derivative_mat_row_major.size());

      eigen_grad = constraint_derivative_vectorized;
    } else {
      // Force stop NLOpt in case of evaluation error or non-finite gradient.
      nlopt_force_stop(context->Opt());
      return;
    }
  }

  if (const auto value_or = context->Constraint()->Evaluate(eigen_x);
      value_or.ok() && value_or.value().array().isFinite().all()) {
    Eigen::Map<eigenmath::VectorXd> eigen_result(residual, m);
    eigen_result = value_or.value();
  } else {
    // Force stop in case of an evaluation error or non-finite constraint value.
    nlopt_force_stop(context->Opt());
  }
}

}  // namespace

namespace nlopt_internal {

void CostContainer::AddCostTerm(
    std::unique_ptr<CostFunctionInterface> cost_term) {
  cost_terms_.push_back(std::move(cost_term));
}

absl::StatusOr<double> CostContainer::EvaluateCostFunction(
    const eigenmath::VectorXd& x) const {
  double cost = 0;
  for (const auto& cost_term : cost_terms_) {
    INTR_ASSIGN_OR_RETURN(double cost_increment, cost_term->Evaluate(x));
    cost += cost_increment;
  }
  return cost;
}

absl::StatusOr<eigenmath::VectorXd> CostContainer::EvaluateCostGradient(
    const eigenmath::VectorXd& x) const {
  eigenmath::VectorXd grad = eigenmath::VectorXd::Zero(x.size());
  for (const auto& cost_term : cost_terms_) {
    INTR_ASSIGN_OR_RETURN(eigenmath::VectorXd grad_increment,
                          cost_term->Gradient(x));
    grad += grad_increment;
  }
  return grad;
}

}  // namespace nlopt_internal

void NLOptProblem::AddCost(std::unique_ptr<CostFunctionInterface> cost_term) {
  costs_.AddCostTerm(std::move(cost_term));
}

void NLOptProblem::AddConstraint(
    std::unique_ptr<ConstraintInterface> constraint) {
  constraints_.push_back(std::move(constraint));
}

absl::Status NLOptProblem::SetBoxConstraintLowerBound(
    const eigenmath::VectorXd& x_lower) {
  if (x_lower.size() != num_opt_var_) {
    return absl::FailedPreconditionError(
        "Size of lower bound must match number of optimization variables.");
  }
  x_lower_bound_ = x_lower;
  return absl::OkStatus();
}

absl::Status NLOptProblem::SetBoxConstraintUpperBound(
    const eigenmath::VectorXd& x_upper) {
  if (x_upper.size() != num_opt_var_) {
    return absl::FailedPreconditionError(
        "Size of upper bound must match number of optimization variables.");
  }
  x_upper_bound_ = x_upper;
  return absl::OkStatus();
}

/*static*/
absl::StatusOr<std::unique_ptr<NLOptSolver>> NLOptSolver::Create(
    std::unique_ptr<NLOptProblem> problem, nlopt_algorithm algorithm) {
  if (problem->constraints_.empty() && problem->cost().empty()) {
    return absl::FailedPreconditionError(
        "No costs or constraints specified, aborting because problem is "
        "empty.");
  }

  NLOptUniquePtr opt(nlopt_create(algorithm, problem->num_opt_var_));

  auto cost_context = std::make_unique<nlopt_internal::CostContext>(
      &problem->costs_, opt.get());
  INTR_RET_CHECK_EQ(
      nlopt_set_min_objective(opt.get(), NLOptCostWrapper, cost_context.get()),
      NLOPT_SUCCESS)
      << "nlopt_set_min_objective() failed";

  if (problem->x_lower_bound_.has_value()) {
    INTR_RET_CHECK_EQ(
        nlopt_set_lower_bounds(opt.get(), problem->x_lower_bound_->data()),
        NLOPT_SUCCESS)
        << "nlopt_set_lower_bounds() failed.";
  }
  if (problem->x_upper_bound_.has_value()) {
    INTR_RET_CHECK_EQ(
        nlopt_set_upper_bounds(opt.get(), problem->x_upper_bound_->data()),
        NLOPT_SUCCESS)
        << "nlopt_set_upper_bounds() failed.";
  }

  std::vector<std::unique_ptr<nlopt_internal::ConstraintContext>>
      constraint_context;
  for (const auto& constraint : problem->constraints_) {
    constraint_context.push_back(
        std::make_unique<nlopt_internal::ConstraintContext>(constraint.get(),
                                                            opt.get()));

    if (constraint->Type() == ConstraintInterface::GENERAL_EQUALITY) {
      INTR_RET_CHECK_EQ(
          nlopt_add_equality_mconstraint(
              opt.get(), constraint->ConstraintDimension(),
              NLOptConstraintWrapper, constraint_context.back().get(),
              constraint->Tolerance().data()),
          NLOPT_SUCCESS)
          << "nlopt_add_equality_mconstraint() failed for GENERAL_EQUALITY "
             "constraint.";
    } else {
      INTR_RET_CHECK_EQ(
          nlopt_add_inequality_mconstraint(
              opt.get(), constraint->ConstraintDimension(),
              NLOptConstraintWrapper, constraint_context.back().get(),
              constraint->Tolerance().data()),
          NLOPT_SUCCESS)
          << "nlopt_add_inequality_mconstraint() failed for GENERAL_INEQUALITY "
             "constraint.";
    }
  }

  // Using `new` to access a non-public constructor.
  return absl::WrapUnique(
      new NLOptSolver(std::move(problem), std::move(cost_context),
                      std::move(constraint_context), std::move(opt)));
}

NLOptSolver::NLOptSolver(
    std::unique_ptr<NLOptProblem> problem,
    std::unique_ptr<nlopt_internal::CostContext> cost_context,
    std::vector<std::unique_ptr<nlopt_internal::ConstraintContext>>
        constraint_context,
    NLOptUniquePtr opt)
    : problem_(std::move(problem)),
      cost_context_(std::move(cost_context)),
      constraint_context_(std::move(constraint_context)),
      opt_(std::move(opt)) {}

absl::StatusOr<NLOptSolver::Result> NLOptSolver::Solve(
    const eigenmath::VectorXd& x_init, const NLOptOptions& options) {
  INTR_RET_CHECK_EQ(nlopt_set_xtol_rel(opt_.get(), options.xtol_relative),
                    NLOPT_SUCCESS)
      << "nlopt_set_xtol_rel() failed.";
  INTR_RET_CHECK_EQ(nlopt_set_xtol_abs1(opt_.get(), options.xtol_absolute),
                    NLOPT_SUCCESS)
      << "nlopt_set_xtol_abs1() failed.";
  INTR_RET_CHECK_EQ(nlopt_set_maxeval(opt_.get(), options.max_evaluations),
                    NLOPT_SUCCESS)
      << "nlopt_set_maxeval() failed.";

  eigenmath::VectorXd x_solution_candidate = x_init;
  double fmin = 0;
  Result result;
  result.status = Result::SOLVED;
  absl::Time tstart = absl::Now();

  nlopt_result exit_code =
      nlopt_optimize(opt_.get(), x_solution_candidate.data(), &fmin);

  result.measured_processing_time = absl::Now() - tstart;

  // Interpret exit codes. Negative exit codes are general failures, exit codes
  // greater than 4 point to timeout and max number of iterations exceeded.
  if (exit_code < 0 || exit_code > 4) {
    // Set NOT_SOLVED due to timeout or due to exceeding iteration limit and
    // return early.
    result.status = Result::SolveStatus::NOT_SOLVED;
    return result;
  } else {
    // Have a solution candidate, need to evaluate constraint residuals for
    // accepting or rejecting solution candidate.
    result.named_constraint_residuals.reserve(problem_->constraints_.size());
    for (auto& constraint : problem_->constraints_) {
      INTR_ASSIGN_OR_RETURN(eigenmath::VectorXd residual,
                            constraint->Evaluate(x_solution_candidate));
      // Constraint residual must lie within a relaxed tolerance to be accepted
      // as SOLVED. Note that this is not a strict test criterion. NLOpt's
      // solvers sometimes terminate "in the vicinity" of constraint
      // statisfaction, which means we can expect NLOpt to terminate well within
      // the most 'loose' tolerance employed in the problem. We do need to check
      // this manually, as the exit codes alone do not guarantee constraint
      // satisfaction.
      eigenmath::VectorXd relaxed_tolerance =
          constraint->Tolerance().array().max(
              std::max(options.xtol_absolute, options.xtol_relative));

      if (constraint->Type() == ConstraintInterface::GENERAL_EQUALITY) {
        // Check if equality constraints are sufficiently satisfied.
        if ((residual.cwiseAbs() - relaxed_tolerance).maxCoeff() > 0) {
          result.status = Result::SolveStatus::NOT_SOLVED;
        }
      } else if (constraint->Type() ==
                 ConstraintInterface::GENERAL_INEQUALITY) {
        // Check if inequality constraints are sufficiently satisfied.
        if ((residual - relaxed_tolerance).maxCoeff() > 0.0) {
          result.status = Result::SolveStatus::NOT_SOLVED;
        }
        // We only report back the positive part of the residual, because
        // negative residuals are okay for inequality constraints:
        residual = residual.array().max(0.0);
      } else {
        return absl::NotFoundError(absl::StrCat(
            "Requested constraint ", magic_enum::enum_name(constraint->Type()),
            " not found."));
      }
      result.named_constraint_residuals.emplace_back(constraint->Name(),
                                                     residual);
    }

    if (result.status == Result::NOT_SOLVED) return result;

    // All constraints are satisfied, return SOLVED.
    result.status = Result::SolveStatus::SOLVED;
    result.x_solution = x_solution_candidate;
    result.cost = fmin;
  }

  return result;
}

}  // namespace intrinsic
