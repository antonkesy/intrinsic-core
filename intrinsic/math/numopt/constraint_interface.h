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

#ifndef INTRINSIC_MATH_NUMOPT_CONSTRAINT_INTERFACE_H_
#define INTRINSIC_MATH_NUMOPT_CONSTRAINT_INTERFACE_H_

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Interface class for a vector-valued constraint equation in the form
// c(x) == 0 or c(x) <= 0 for a residual function 'c' that maps an input vector
// with dimension 'n' to an output vector with dimension 'm'.
//
// This class cannot mix equality constraints with inequality constraints;
// separate instances must be created.
//
// The cost function must be continuously differentiable, and should be designed
// to work well with optimizers; see README.md in this directory for details.
class ConstraintInterface {
 public:
  enum ConstraintType {
    GENERAL_INEQUALITY = 0,
    GENERAL_EQUALITY,
  };

  virtual ~ConstraintInterface() = default;

  virtual ConstraintType Type() const = 0;

  // Returns the number of constraint dimensions 'm'.
  virtual int ConstraintDimension() const = 0;

  // Returns the dimension 'n' of input vectors to `Evaluate`, `Gradient`, and
  // `IsSatisfied`.
  virtual int DomainDimension() const = 0;

  // Returns the 'm' dimensional tolerance vector. The constraint is
  // satisfied whenever |c(x)| <= tolerance for equality constraints and
  // whenever c(x) <= tolerance for inequality constraints.
  virtual const eigenmath::VectorXd& Tolerance() const = 0;

  // Evaluates the residual function `c(x)` and returns its output vector.
  virtual absl::StatusOr<eigenmath::VectorXd> Evaluate(
      const eigenmath::VectorXd& x) = 0;

  // Evaluates the Jacobian of the residual function `c(x)` at the given input
  // vector 'x' with dimension 'n'. Returns an 'm' by 'n' matrix, where 'm' is
  // the dimension of the output of `c(x)`.
  virtual absl::StatusOr<eigenmath::MatrixXd> Gradient(
      const eigenmath::VectorXd& x) = 0;

  // Returns whether `|c(x)| <= Tolerance()` when the constraint type is
  // `GENERAL_EQUALITY` or whether `c(x) <= Tolerance()` when the constraint
  // type is `GENERAL_INEQUALITY`.
  virtual absl::StatusOr<bool> IsSatisfied(const eigenmath::VectorXd& x) = 0;

  virtual absl::string_view Name() const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_NUMOPT_CONSTRAINT_INTERFACE_H_
