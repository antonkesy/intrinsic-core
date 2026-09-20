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

#include "intrinsic/math/numopt/simplex.h"

#include <cstdlib>
#include <limits>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/numopt/ruiz_equilibration.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

// Numerical tolerance for values close to zero. Numerical values or
// coefficients below this threshold are considered zero.
constexpr double kNumericalZeroTolerance = 1.0e-13;

absl::Status ValidateInput(Eigen::Ref<const eigenmath::VectorXd> c,
                           Eigen::Ref<const eigenmath::MatrixXd> A_ineq,
                           Eigen::Ref<const eigenmath::VectorXd> b_ineq,
                           Eigen::Ref<const eigenmath::MatrixXd> A_eq,
                           Eigen::Ref<const eigenmath::VectorXd> b_eq,
                           const SimplexOptions& options) {
  if (A_ineq.rows() != b_ineq.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The rows of `A_ineq` should match the size of `b_ineq`. Got ",
        A_ineq.rows(), " != ", b_ineq.size(), "."));
  }
  if (A_ineq.rows() > 0 && A_ineq.cols() != c.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The columns of `A_ineq` should match the size of `c`. Got ",
        A_ineq.cols(), " != ", c.size(), "."));
  }
  if (A_eq.rows() != b_eq.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The rows of `A_eq` should match the size of `b_eq`. Got ",
                     A_eq.rows(), " != ", b_eq.size(), "."));
  }
  if (A_eq.rows() > 0 && A_eq.cols() != c.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The columns of `A_eq` should match the size of `c`. Got ",
                     A_eq.cols(), " != ", c.size(), "."));
  }
  if (options.max_iterations <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The maximum number of iterations should be positive. Got ",
        options.max_iterations, "."));
  }
  const int kNumVars = c.size();
  for (const int free_var_id : options.free_primal_variables) {
    if (free_var_id < 0 || free_var_id >= kNumVars) {
      return absl::InvalidArgumentError(
          absl::StrCat("The free variable index ", free_var_id,
                       " should be in the range [0, ", kNumVars, ")."));
    }
  }
  if (options.num_ruiz_equilibration_iterations.has_value() &&
      *options.num_ruiz_equilibration_iterations <= 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The number of Ruiz equilibration iterations mas be positive. Got ",
        *options.num_ruiz_equilibration_iterations, "."));
  }
  return absl::OkStatus();
}

// Finds the entering variable in the `tableau`. The entering variable is the
// variable with the most negative coefficient in the objective row (bottom).
// Returns -1 if no entering variable is found, which indicates optimality.
int FindEnteringVariable(const eigenmath::RowMajorMatrixXd& tableau) {
  int entering_var = -1;
  // Start the threshold slightly below zero to ignore floating-point precision
  // errors created by the equilibration routine or row operations.
  double min_coeff = -kNumericalZeroTolerance;
  const int kObjectiveRowId = tableau.rows() - 1;
  for (int var_id = 0; var_id < tableau.cols() - 1; ++var_id) {
    if (tableau(kObjectiveRowId, var_id) < min_coeff) {
      min_coeff = tableau(kObjectiveRowId, var_id);
      entering_var = var_id;
    }
  }
  return entering_var;
}

// Finds the leaving variable in the `tableau`. The leaving variable is the
// variable with the minimum ratio of the last column to the `entering_var`.
// Returns -1 if no positive ratio is found, which indicates unboundedness.
int FindLeavingVariable(const eigenmath::RowMajorMatrixXd& tableau,
                        const int entering_var) {
  int leaving_var = -1;
  double min_ratio = std::numeric_limits<double>::max();
  const int kLastColumnId = tableau.cols() - 1;
  for (int constraint_id = 0; constraint_id < tableau.rows() - 1;
       ++constraint_id) {
    // Avoid division by zero or negative.
    if (tableau(constraint_id, entering_var) > kNumericalZeroTolerance) {
      const double ratio = tableau(constraint_id, kLastColumnId) /
                           tableau(constraint_id, entering_var);
      if (ratio < min_ratio) {
        min_ratio = ratio;
        leaving_var = constraint_id;
      }
    }
  }
  return leaving_var;
}

// Runs the pivot on tableau that normalizes the `leaving_var`'s row using the
// (`leaving_var`, `entering_var`) coefficient known as the pivot. It then
// subtracts it from the rows of the other variables, so that the value of the
// coefficient of all rows at the `entering_var` is zero.
void RunPivotOperation(const int leaving_var, const int entering_var,
                       eigenmath::RowMajorMatrixXd& tableau) {
  const double pivot_value = tableau(leaving_var, entering_var);
  tableau.row(leaving_var) /= pivot_value;

  for (int row_id = 0; row_id < tableau.rows(); ++row_id) {
    if (row_id != leaving_var) {
      const double factor = tableau(row_id, entering_var);
      tableau.row(row_id) -= factor * tableau.row(leaving_var);
    }
  }
}

// The tableau is a matrix that stores the entire problem description,
// including:
// - the objective `c` in the last row.
// - the inequality constraints `A_ineq * x <= b_ineq`.
// - the equality constraints `A_eq * x == b_eq`.
//
// A standard tableau for a maximization problem looks like this:
//   [ A_ineq | I_ineq | artificial variables | b_ineq ]
//   [  A_eq  |  0.0   | artificial variables |  b_eq  ]
//   [  -c^T  |  0.0   |         0.0          |   0.0  ]
//
// Each inequality constraint is augmented by an additional non-negative slack
// variable, represented by the identity matrix `I_ineq`. This makes it easy to
// construct an initial basic feasible solution by setting all original
// variables to zero, allowing the slack variables to equal the right-hand side.
//
// Equality constraints strictly enforce `Ax == b`, so they do not receive slack
// variables. Because they lack slacks, they always require an artificial
// variable to serve as the initial basic variable for Phase 1 of the simplex.
//
// Artificial variables are also required for inequalities when the right-hand
// side is negative. If we simply used the slack variable, it would be negative
// (matching the right-hand side), which violates the non-negativity constraint.
// To fix this, we multiply the entire constraint row by -1 (making the
// right-hand side positive), and introduce an artificial variable to act as the
// basic variable instead.
//
// When the `options` contain indices of free variables (variables not bounded
// to be >= 0), they are replaced by a difference of two non-negative variables
// (`x = x^+ - x^-`). This requires adding mirroring columns for the free
// variables to `A` and `c`, resulting in the final tableau format:
//   [ A_ineq | -A_ineq(free) | I_ineq | artificial variables | b_ineq ]
//   [  A_eq  |  -A_eq(free)  |  0.0   | artificial variables |  b_eq  ]
//   [  -c^T  |    c(free)    |  0.0   |         0.0          |   0.0  ]
struct LinearProgramTableau {
  int num_vars = 0;
  int num_cons = 0;
  int num_free_vars = 0;
  int num_slack_vars = 0;
  int num_artificial_vars = 0;
  eigenmath::RowMajorMatrixXd tableau;
  std::vector<int> basic_variables = {};
};

absl::StatusOr<LinearProgramTableau> ConstructLinearProgramTableau(
    Eigen::Ref<const eigenmath::VectorXd> c,
    Eigen::Ref<const eigenmath::MatrixXd> A_ineq,
    Eigen::Ref<const eigenmath::VectorXd> b_ineq,
    Eigen::Ref<const eigenmath::MatrixXd> A_eq,
    Eigen::Ref<const eigenmath::VectorXd> b_eq, const SimplexOptions& options) {
  // Instantiate the tableau and initialize its main dimensions.
  LinearProgramTableau lp;
  lp.num_vars = c.size();
  lp.num_free_vars = options.free_primal_variables.size();

  // Check unbounded constraints (those with infinite right hand side).
  std::vector<int> bounded_ineq;
  bounded_ineq.reserve(A_ineq.rows());
  for (int constraint_id = 0; constraint_id < A_ineq.rows(); ++constraint_id) {
    if (b_ineq(constraint_id) < kSimplexInfinity)
      bounded_ineq.push_back(constraint_id);
  }
  const int num_bounded_ineq = bounded_ineq.size();
  const int num_eq = A_eq.rows();
  lp.num_cons = num_bounded_ineq + num_eq;
  lp.num_slack_vars = num_bounded_ineq;  // Equalities don't use slack variables

  // Loop over the vector of right hand side values and assign the corresponding
  // basic variable for each constraint. Keep track of the constraints that
  // require a sign flip.
  lp.basic_variables.reserve(lp.num_cons);
  std::vector<int> artificial_rows;
  artificial_rows.reserve(lp.num_cons);
  std::vector<bool> flip_row(lp.num_cons, false);

  // Identify inequalities that need artificial variables (b < 0).
  for (int i = 0; i < num_bounded_ineq; ++i) {
    if (b_ineq(bounded_ineq[i]) <= -kNumericalZeroTolerance) {
      artificial_rows.push_back(i);
      flip_row[i] = true;
    }
  }
  // All equalities need artificial variables.
  for (int i = 0; i < num_eq; ++i) {
    artificial_rows.push_back(num_bounded_ineq + i);
    if (b_eq(i) <= -kNumericalZeroTolerance) {
      flip_row[num_bounded_ineq + i] = true;
    }
  }
  lp.num_artificial_vars = artificial_rows.size();

  const int slack_start = lp.num_vars + lp.num_free_vars;
  const int art_start = slack_start + lp.num_slack_vars;

  int art_count = 0;
  for (int i = 0; i < num_bounded_ineq; ++i) {
    if (flip_row[i]) {
      lp.basic_variables.push_back(art_start + art_count++);
    } else {
      lp.basic_variables.push_back(slack_start + i);
    }
  }
  for (int i = 0; i < num_eq; ++i) {
    lp.basic_variables.push_back(art_start + art_count++);
  }

  lp.tableau = eigenmath::RowMajorMatrixXd::Zero(
      lp.num_cons + 1, lp.num_vars + lp.num_free_vars + lp.num_slack_vars +
                           lp.num_artificial_vars + 1);

  const int rhs_col = lp.tableau.cols() - 1;

  // Fill blocks for inequalities.
  for (int i = 0; i < num_bounded_ineq; ++i) {
    const int row = bounded_ineq[i];
    const double sign = flip_row[i] ? -1.0 : 1.0;

    lp.tableau.row(i).head(lp.num_vars) = sign * A_ineq.row(row);
    for (int free_id = 0; free_id < lp.num_free_vars; ++free_id) {
      lp.tableau(i, lp.num_vars + free_id) =
          -sign * A_ineq(row, options.free_primal_variables[free_id]);
    }
    lp.tableau(i, slack_start + i) = sign;
    lp.tableau(i, rhs_col) = sign * b_ineq(row);
  }

  // Fill blocks for equalities (No slack variable assigned).
  for (int i = 0; i < num_eq; ++i) {
    const int row = num_bounded_ineq + i;
    const double sign = flip_row[row] ? -1.0 : 1.0;

    lp.tableau.row(row).head(lp.num_vars) = sign * A_eq.row(i);
    for (int free_id = 0; free_id < lp.num_free_vars; ++free_id) {
      lp.tableau(row, lp.num_vars + free_id) =
          -sign * A_eq(i, options.free_primal_variables[free_id]);
    }
    lp.tableau(row, rhs_col) = sign * b_eq(i);
  }

  // Set up artificial variables.
  for (int i = 0; i < lp.num_artificial_vars; ++i) {
    lp.tableau(artificial_rows[i], art_start + i) = 1.0;
  }

  return lp;
}

// Fills the cost vector for Phase 1 of the simplex method. This cost vector
// is used to determine the feasibility of the solution. Only modifies the last
// row of the tableau.
void FillCostForPhaseI(LinearProgramTableau& lp) {
  lp.tableau.row(lp.num_cons).setZero();

  const int art_vars_start = lp.num_vars + lp.num_free_vars + lp.num_slack_vars;
  for (int art_id = 0; art_id < lp.num_artificial_vars; ++art_id) {
    lp.tableau(lp.num_cons, art_vars_start + art_id) = 1.0;
  }
}

// Fills the cost vector for Phase 2 of the simplex method. This cost vector
// is used to determine the optimal solution. Only modifies the last row of the
// tableau.
void FillCostForPhaseII(Eigen::Ref<const eigenmath::VectorXd> c,
                        const SimplexOptions& options,
                        LinearProgramTableau& lp) {
  lp.tableau.row(lp.num_cons).setZero();
  lp.tableau.bottomLeftCorner(1, lp.num_vars) = -c.transpose();

  // Fill the mirroring columns due to free variables.
  for (int free_id = 0; free_id < lp.num_free_vars; ++free_id) {
    const int free_var_id = options.free_primal_variables[free_id];
    lp.tableau(lp.tableau.rows() - 1, lp.num_vars + free_id) = c(free_var_id);
  }
}

// Expresses the objective row of the `lp.tableau` (that corresponds to the last
// row) in terms of non-basic variables. Thus, it runs Gaussian elimination on
// the objective row to remove the basic variables.
absl::Status GaussianEliminationOfBasicVariablesInCost(
    LinearProgramTableau& lp) {
  const int kObjectiveRowId = lp.tableau.rows() - 1;
  for (int constraint_id = 0; constraint_id < lp.basic_variables.size();
       ++constraint_id) {
    // Because we are using basic variables, the coefficient of the term
    // `lp.tableau(id, basic_var)` should be one. Thus, the division is safe.
    // however, we add a safety check that should never be reached.
    const int basic_var = lp.basic_variables[constraint_id];
    if (std::abs(lp.tableau(constraint_id, basic_var)) <
        kNumericalZeroTolerance) {
      return absl::InternalError(absl::StrCat(
          "During Gaussian Elimination the normalization coefficient ",
          lp.tableau(constraint_id, basic_var),
          " became too close to zero, while it should be close to one."));
    }
    double factor = -lp.tableau(kObjectiveRowId, basic_var) /
                    lp.tableau(constraint_id, basic_var);
    lp.tableau.row(kObjectiveRowId) += factor * lp.tableau.row(constraint_id);
    lp.tableau(kObjectiveRowId, basic_var) = 0.0;
  }

  return absl::OkStatus();
}

// Removes the columns corresponding to artificial variables from the tableau.
void RemoveArtificialVariables(LinearProgramTableau& lp) {
  const int art_vars_start = lp.num_vars + lp.num_free_vars + lp.num_slack_vars;
  for (int art_id = 0; art_id < lp.num_artificial_vars; ++art_id) {
    lp.tableau.col(art_vars_start + art_id).setZero();
  }
}

// Removes the columns corresponding to free variables from the solution vector.
void RemoveFreeVariables(const SimplexOptions& options, const int num_variables,
                         SimplexResult& result) {
  if (!options.free_primal_variables.empty()) {
    eigenmath::VectorXd solution = result.solution.head(num_variables);
    for (int id = 0; id < options.free_primal_variables.size(); ++id) {
      const int free_var_id = options.free_primal_variables[id];
      solution[free_var_id] -= result.solution[num_variables + id];
    }
    result.solution = solution;
  }
}

// Based on the `lp.tableau`, the simplex method solves the linear programming
// problem by identifying a pair of entering and leaving variables and
// performing a pivot operation. The process is repeated until the entering
// variable is not found, which indicates optimality.
SimplexResult SolveLPFromTableau(const SimplexOptions& options,
                                 LinearProgramTableau& lp) {
  int num_iterations = 0;
  while (num_iterations < options.max_iterations) {
    // Search the tableau for the next entering variable. The entering variable
    // is the variable with the most negative coefficient in the objective row
    // (bottom). If no entering variable is found, the problem is solved.
    const int entering_var = FindEnteringVariable(lp.tableau);
    if (entering_var == -1) break;

    // Find the leaving variable (based on minimum ratio test).
    // If no positive ratio, the problem is unbounded.
    const int leaving_var = FindLeavingVariable(lp.tableau, entering_var);
    if (leaving_var == -1) {
      return SimplexResult{
          .status = SimplexStatus::kUnbounded,
          .objective_value = std::numeric_limits<double>::infinity(),
          .solution =
              eigenmath::VectorXd::Zero(lp.num_vars + lp.num_free_vars)};
    }

    // Run the pivot operation that normalizes the leaving variable's row and
    // subtracts it  from the rows of the other variables.
    RunPivotOperation(leaving_var, entering_var, lp.tableau);

    // Update the basic variable
    lp.basic_variables[leaving_var] = entering_var;

    num_iterations++;
  }

  // Construct the simplex result.
  SimplexResult result;
  result.status = (num_iterations == options.max_iterations)
                      ? SimplexStatus::kMaxIterationsReached
                      : SimplexStatus::kOptimal;
  result.objective_value =
      lp.tableau(lp.tableau.rows() - 1, lp.tableau.cols() - 1);
  result.solution = eigenmath::VectorXd::Zero(lp.num_vars + lp.num_free_vars);
  for (int row_id = 0; row_id < lp.tableau.rows() - 1; ++row_id) {
    if (lp.basic_variables[row_id] < lp.num_vars + lp.num_free_vars) {
      result.solution[lp.basic_variables[row_id]] =
          lp.tableau(row_id, lp.tableau.cols() - 1);
    }
  }
  return result;
}

// Solves the primal linear programming problem using the simplex method.
//   maximize:     c^T x
//   subject to:   A x <= b
//                 x >= 0
// The problem is solved by converting it to a tableau and then solving the
// tableau. The `options` contain indices of free variables, which are
// variables that are not bounded and can take any value. The free variables
// are replaced by a difference of non-negative variables, which implies we
// add additional columns to the tableau for `A` and `c`.
absl::StatusOr<SimplexResult> SolvePrimalLP(
    Eigen::Ref<const eigenmath::VectorXd> c,
    Eigen::Ref<const eigenmath::MatrixXd> A_ineq,
    Eigen::Ref<const eigenmath::VectorXd> b_ineq,
    Eigen::Ref<const eigenmath::MatrixXd> A_eq,
    Eigen::Ref<const eigenmath::VectorXd> b_eq, const SimplexOptions& options) {
  // Build the initial tableau and check whether it has artificial variables.
  INTR_ASSIGN_OR_RETURN(
      LinearProgramTableau lp,
      ConstructLinearProgramTableau(c, A_ineq, b_ineq, A_eq, b_eq, options));
  if (lp.num_artificial_vars > 0) {
    // Run Phase 1 of the simplex algorithm, by setting up in the tableau as
    // objective to minimize the value of the sum of artificial variables.
    FillCostForPhaseI(lp);
    INTR_RETURN_IF_ERROR(GaussianEliminationOfBasicVariablesInCost(lp));

    // Solve the problem and check for feasibility. If the cost cannot be driven
    // to zero, it means that the problem is infeasible. If the problem is
    // feasible, we continue with Phase 2.
    SimplexResult result = SolveLPFromTableau(options, lp);
    if (std::abs(result.objective_value) >=
        options.phase_one_feasibility_tolerance) {
      return SimplexResult{.status = SimplexStatus::kInfeasible,
                           .objective_value = 0.0,
                           .solution = eigenmath::VectorXd::Zero(lp.num_vars)};
    }

    // Drives degenerate artificial variables out of the basis before Phase 2.
    //
    // If an equality constraint has a right-hand side of zero, Phase 1 can
    // terminate successfully while an artificial variable is still part of the
    // active basis (with a value of 0.0).
    //
    // We must forcefully pivot these degenerate artificial variables out of the
    // basis using any eligible real variable (primal, free, or slack) with a
    // non-zero coefficient. If we skip this, the subsequent step to clear
    // artificial columns (`RemoveArtificialVariables`) will wipe out the basic
    // identity column for that row, completely destroying the equality
    // constraint and allowing Phase 2 to return physically invalid solutions.
    const int art_vars_start =
        lp.num_vars + lp.num_free_vars + lp.num_slack_vars;
    for (int i = 0; i < lp.num_cons; ++i) {
      if (lp.basic_variables[i] >= art_vars_start) {
        // The artificial variable is still basic. Pivot it out using any
        // valid primal or slack variable with a non-zero coefficient.
        for (int j = 0; j < art_vars_start; ++j) {
          if (std::abs(lp.tableau(i, j)) > kNumericalZeroTolerance) {
            RunPivotOperation(i, j, lp.tableau);
            lp.basic_variables[i] = j;
            break;
          }
        }
        // If no non-zero coefficient was found, the entire row is `0 = 0` (a
        // redundant constraint). The artificial variable safely remains in the
        // basis but will be completely ignored during Phase 2 pivoting.
      }
    }
  }

  // We setup the objective row for Phase 2, which is used to determine the
  // optimal solution. If the problem used Phase 1, we run Gaussian elimination
  // to express the objective row in terms of non-basic variables.
  FillCostForPhaseII(c, options, lp);
  if (lp.num_artificial_vars > 0) {
    INTR_RETURN_IF_ERROR(GaussianEliminationOfBasicVariablesInCost(lp));
    RemoveArtificialVariables(lp);
  }

  // Solve the problem.
  SimplexResult result = SolveLPFromTableau(options, lp);
  RemoveFreeVariables(options, lp.num_vars, result);
  return result;
}

}  // namespace

// Primary solver routine.
absl::StatusOr<SimplexResult> SimplexSolve(
    Eigen::Ref<const eigenmath::VectorXd> c,
    Eigen::Ref<const eigenmath::MatrixXd> A_ineq,
    Eigen::Ref<const eigenmath::VectorXd> b_ineq,
    Eigen::Ref<const eigenmath::MatrixXd> A_eq,
    Eigen::Ref<const eigenmath::VectorXd> b_eq, const SimplexOptions& options) {
  INTR_RETURN_IF_ERROR(ValidateInput(c, A_ineq, b_ineq, A_eq, b_eq, options));
  if (options.num_ruiz_equilibration_iterations.has_value()) {
    INTR_ASSIGN_OR_RETURN(
        const RuizEquilibrationResult scaled_problem,
        ApplyRuizEquilibration(c, A_ineq, b_ineq, A_eq, b_eq,
                               *options.num_ruiz_equilibration_iterations));
    INTR_ASSIGN_OR_RETURN(
        SimplexResult simplex_result,
        SolvePrimalLP(scaled_problem.c, scaled_problem.A_ineq,
                      scaled_problem.b_ineq, scaled_problem.A_eq,
                      scaled_problem.b_eq, options));

    // Scaled back the solution based on `scaled_problem.columns_scaling`.
    if (simplex_result.solution.size() !=
        scaled_problem.columns_scaling.size()) {
      // This branch should never happen.
      return absl::InternalError(
          absl::StrCat("The size of the simplex result and equilibration "
                       "scaling do not match. Got ",
                       simplex_result.solution.size(), " vs. ",
                       scaled_problem.columns_scaling.size(), "."));
    }
    simplex_result.solution =
        simplex_result.solution.cwiseProduct(scaled_problem.columns_scaling);
    return simplex_result;
  }
  return SolvePrimalLP(c, A_ineq, b_ineq, A_eq, b_eq, options);
}

// Backward-compatible overload.
absl::StatusOr<SimplexResult> SimplexSolve(
    Eigen::Ref<const eigenmath::VectorXd> c,
    Eigen::Ref<const eigenmath::MatrixXd> A,
    Eigen::Ref<const eigenmath::VectorXd> b, const SimplexOptions& options) {
  eigenmath::MatrixXd A_eq(0, c.size());
  eigenmath::VectorXd b_eq(0);
  return SimplexSolve(c, A, b, A_eq, b_eq, options);
}

}  // namespace intrinsic
