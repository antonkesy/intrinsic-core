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

#ifndef INTRINSIC_MATH_SIGNALS_NESTED_QUADRATURE_H_
#define INTRINSIC_MATH_SIGNALS_NESTED_QUADRATURE_H_

// A library defining an interface for nested quadrature rules, i.e., for that
// for the same set of function evaluation points, apply two quadrature rules:
// one higher order and one lower order (the latter called an embedded rule).
// The difference between these two approximations is used to get an integration
// error estimate.

#include "absl/functional/function_ref.h"
#include "absl/status/statusor.h"

namespace intrinsic {

// A struct to hold the result of a nested quadrature rule. The `integral`
// represents the integral of the function over the integration interval
// [start, end]. The `error_estimate` represents the difference between the
// result of the higher-order quadrature rule (i.e., `integral`) and the one
// provided by a lower-order embedded rule.
struct NestedQuadratureResult {
  double integral;
  double error_estimate;
};

// An interface for nested quadrature rules.
class NestedQuadratureRule {
 public:
  virtual ~NestedQuadratureRule() = default;

  // Computes the integral of the given `function` over the interval [`start`,
  // `end`].
  virtual absl::StatusOr<NestedQuadratureResult> Integrate(
      absl::FunctionRef<absl::StatusOr<double>(double)> function, double start,
      double end) const = 0;

  // Returns the number of nodes used by the embedded rule of the nested
  // quadrature, i.e., the rule of lower order.
  virtual int NumNodesEmbeddedRule() const = 0;

  // Returns the number of nodes used by the extended rule of the nested
  // quadrature, i.e., the rule of higher order.
  virtual int NumNodesExtendedRule() const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SIGNALS_NESTED_QUADRATURE_H_
