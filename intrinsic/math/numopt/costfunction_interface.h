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

#ifndef INTRINSIC_MATH_NUMOPT_COSTFUNCTION_INTERFACE_H_
#define INTRINSIC_MATH_NUMOPT_COSTFUNCTION_INTERFACE_H_

#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Interface class for a scalar-valued cost function term J(x) with
// vector-valued input 'x' of dimension 'n'.
class CostFunctionInterface {
 public:
  virtual ~CostFunctionInterface() = default;

  // Evaluates cost function J(x).
  virtual absl::StatusOr<double> Evaluate(const eigenmath::VectorXd& x) = 0;

  // Evaluates the cost functions gradient w.r.t 'x', dJ(x)/x, returns an n x 1
  // vector.
  virtual absl::StatusOr<eigenmath::VectorXd> Gradient(
      const eigenmath::VectorXd& x) = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_NUMOPT_COSTFUNCTION_INTERFACE_H_
