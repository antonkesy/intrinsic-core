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

#ifndef INTRINSIC_MATH_NUMOPT_NLOPT_NONLINEAR_PROGRAM_H_
#define INTRINSIC_MATH_NUMOPT_NLOPT_NONLINEAR_PROGRAM_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/numopt/constraint_interface.h"
#include "intrinsic/math/numopt/costfunction_interface.h"
#include "intrinsic/util/status/status_macros.h"
#include "nlopt.hpp"

namespace intrinsic {

namespace nlopt_internal {

// Since NLOpt does not natively support adding several cost functions
// separately from each other to an optimization problem, we need a custom
// 'CostContainer' to bundle individual cost terms. This 'CostContainer'
// maintains a std::vector<> of cost functions, and loops over all of them for
// cost and gradient evaluation.
class CostContainer {
 public:
  // Adds a cost term to the cost container.
  void AddCostTerm(std::unique_ptr<CostFunctionInterface> cost_term);

  // Returns the sum of all cost terms J(x) evaluated at 'x'.
  absl::StatusOr<double> EvaluateCostFunction(
      const eigenmath::VectorXd& x) const;

  // Returns the sum of all cost gradients dJ(x)/dx,  evaluated at 'x'.
  absl::StatusOr<eigenmath::VectorXd> EvaluateCostGradient(
      const eigenmath::VectorXd& x) const;

  const std::vector<std::unique_ptr<CostFunctionInterface>>& cost_terms() {
    return cost_terms_;
  }

 private:
  std::vector<std::unique_ptr<CostFunctionInterface>> cost_terms_;
};

// The ConstraintContext is a per-constraint helper class for passing both a
// pointer to the constraint and a pointer to NLOpts internal optimization
// struct to the constraint evaluation functions. The pointer to the constraint
// is necessary for constraint evaluation, the pointer to the nlopt optimization
// struct is necessary for triggering custom termination conditions.
class ConstraintContext {
 public:
  ConstraintContext() = delete;

  // Construct a constraint context from a pointer to a constraint
  // implementation deriving from 'ConstraintInterface', and a pointer to the
  // nlopt_opt problem 'opt'.
  ConstraintContext(ConstraintInterface* const constraint, const nlopt_opt opt)
      : constraint_(constraint), nlopt_opt_(opt) {}

  // Retrieve pointer to constraint.
  ConstraintInterface* Constraint() const { return constraint_; }

  // Retrieve pointer to nlopt_opt struct.
  nlopt_opt Opt() { return nlopt_opt_; }

 private:
  ConstraintInterface* const constraint_ = nullptr;
  const nlopt_opt nlopt_opt_;
};

// The CostContext is a helper class for passing both a pointer to the
// CostContainer and a pointer to NLOpts internal optimization struct to the
// cost evaluation functions. The pointer to the cost is necessary
// for cost evaluation, the pointer to the nlopt optimization struct is
// necessary for triggering custom termination conditions.
class CostContext {
 public:
  CostContext() = delete;

  // Construct a cost context from a pointer to a cost container, and a pointer
  // to the nlopt_opt problem 'opt'.
  CostContext(CostContainer* const cost, const nlopt_opt nlopt_opt_ptr)
      : cost_(cost), nlopt_opt_ptr_(nlopt_opt_ptr) {}

  // Retrieve pointer to CostContainer.
  CostContainer* Cost() const { return cost_; }

  // Retrieve pointer to nlopt_opt struct.
  nlopt_opt Opt() { return nlopt_opt_ptr_; }

 private:
  CostContainer* const cost_ = nullptr;
  const nlopt_opt nlopt_opt_ptr_ = nullptr;
};

}  // namespace nlopt_internal

// The 'NLOptProblem' class serves as a container for cost functions,
// constraints, and upper and lower bounds on optimization variables. Allows the
// friend class 'NLOptSolver' direct access to members.
class NLOptProblem {
  friend class NLOptSolver;

 public:
  // Construct NLOptProblem with a known number of optimization variables
  // 'num_opt_var'.
  explicit NLOptProblem(int num_opt_var) : num_opt_var_(num_opt_var) {}

  // Adds a cost term to the cost function.
  void AddCost(std::unique_ptr<CostFunctionInterface> cost_term);

  void AddConstraint(std::unique_ptr<ConstraintInterface> constraint);

  // If possible formulate constraints on the optimization vector itself as box
  // constraint. Sets lower bound on the optimization vector, returns
  // kFailedPrecondition in case of dimension mismatch.
  absl::Status SetBoxConstraintLowerBound(const eigenmath::VectorXd& x_lower);

  // If possible formulate constraints on the optimization vector itself as box
  // constraint. Sets upper bound on the optimization vector, returns
  // kFailedPrecondition in case of dimension mismatch.
  absl::Status SetBoxConstraintUpperBound(const eigenmath::VectorXd& x_upper);

  const std::vector<std::unique_ptr<ConstraintInterface>>& constraints() {
    return constraints_;
  }

  const std::vector<std::unique_ptr<CostFunctionInterface>>& cost() {
    return costs_.cost_terms();
  }

 private:
  int num_opt_var_;
  nlopt_internal::CostContainer costs_;
  std::vector<std::unique_ptr<ConstraintInterface>> constraints_;
  std::optional<eigenmath::VectorXd> x_lower_bound_ = std::nullopt;
  std::optional<eigenmath::VectorXd> x_upper_bound_ = std::nullopt;
};

struct NLOptOptions {
  // Sets relative tolerance on optimization parameters: stop when an
  // optimization step (or an estimate of the optimum) changes every parameter
  // by less than xtol_relative multiplied by the absolute value of the
  // parameter. Criterion is disabled in NLOpt if xtol_relative is
  // non-positive. If there's a chance that the optimal solution is close to
  // zero, 'xtol_absolute' must be set non-zero to ensure termination.
  double xtol_relative = 1e-4;
  // Sets absolute tolerance on optimization parameters.
  double xtol_absolute = 1e-4;
  // Maximum number of iterations allowed before we treat the problem as
  // infeasible.
  int max_evaluations = 250;
};

// NLOptSolver class wraps the NLOpts C-interface for solving nonlinear
// optimization problems. The C-interface has been preferred over the C++
// interface, since it shows well-defined termination behaviour and returns
// error codes, while the C++ interface throws exceptions.
class NLOptSolver {
 public:
  static constexpr nlopt_algorithm kDefaultAlgorithm = NLOPT_LD_SLSQP;

  struct Result {
    enum SolveStatus {
      SOLVED = 0,
      NOT_SOLVED,
    };
    SolveStatus status;
    absl::Duration measured_processing_time;
    // Contains the name and constraint residual for every constraint in the
    // problem.
    std::vector<std::pair<std::string, eigenmath::VectorXd>>
        named_constraint_residuals;
    // Solution vector, to be filled only if solution candidate exists.
    std::optional<eigenmath::VectorXd> x_solution;
    // Min cost value, to be filled only if solution candidate exists.
    std::optional<double> cost;
  };

  NLOptSolver() = delete;

  // NLOptSolver factory, creates an NLOptSolver from an NLOptProblem 'problem'
  // and a desired optimization algorithm 'nlopt_algorithm' used internally for
  // solving the optimization problem. You need to select an algorithm capable
  // of handling your constraint configuration. See
  // https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/ for detailed
  // reference.
  static absl::StatusOr<std::unique_ptr<NLOptSolver>> Create(
      std::unique_ptr<NLOptProblem> problem,
      nlopt_algorithm algorithm = kDefaultAlgorithm);

  // Solve given problem using initial guess 'x_init' and solver 'options'.
  absl::StatusOr<Result> Solve(const eigenmath::VectorXd& x_init,
                               const NLOptOptions& options = NLOptOptions());

  // Non-const accessor required for custom verification steps.
  NLOptProblem* problem() const { return problem_.get(); }

 private:
  // For using std::unique_ptr, need to define a custom deleter functor which
  // can delete the managed nlopt instance.
  struct NLOptDeleter {
    void operator()(nlopt_opt s) const { nlopt_destroy(s); }
  };

  using NLOptUniquePtr = std::unique_ptr<nlopt_opt_s, NLOptDeleter>;

  // Constructs an NLOptSolver from a 'problem', cost and constraint context,
  // and a pointer to the wrapped nlopt C-solver interface. Takes ownership of
  // the NLOpt C-interface pointer.
  NLOptSolver(std::unique_ptr<NLOptProblem> problem,
              std::unique_ptr<nlopt_internal::CostContext> cost_context,
              std::vector<std::unique_ptr<nlopt_internal::ConstraintContext>>
                  constraint_context,
              NLOptUniquePtr opt);

  std::unique_ptr<NLOptProblem> problem_;
  // We need to keep the contexts as class members in order to allow NLOpt to
  // access them implicitly through its wrapper functions.
  std::unique_ptr<nlopt_internal::CostContext> cost_context_;
  std::vector<std::unique_ptr<nlopt_internal::ConstraintContext>>
      constraint_context_;
  NLOptUniquePtr opt_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_NUMOPT_NLOPT_NONLINEAR_PROGRAM_H_
