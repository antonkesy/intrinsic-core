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

#ifndef INTRINSIC_MATH_NUMOPT_FUNCTION_LINEARIZER_NUMDIFF_H_
#define INTRINSIC_MATH_NUMOPT_FUNCTION_LINEARIZER_NUMDIFF_H_

#include "absl/functional/function_ref.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Approximates the gradient df/dx of a vector-valued function `func` with
// the signature "eigenmath::VectorXd func(const eigenmath::VectorXd& x)"
// evaluated at `x` via numerical differentation by finite differences. Returns
// the derivative matrix as eigenmath::MatrixXd of dimensions n x m, where `n`
// is the output-dimension of `func` and `m` is the dimension of 'x' and the
// input-dimension of `func`. By default, approximates the gradient via
// dfdx = (f(x+h)-f(x)) / h. If `double_sided_derivative` is set to true, the
// finite differencing rule is applied to both sides of the setpoint, i.e.
// dfdx = (f(x+h)-f(x-h)) / 2h.
// Note: the implementation of this function does not implement the naive finite
// differencing rule, but a numerically more stable version.
eigenmath::MatrixXd ComputeDerivative(
    absl::FunctionRef<eigenmath::VectorXd(const eigenmath::VectorXd&)> func,
    const eigenmath::VectorXd& x, bool double_sided_derivative = true);

// Approximates the gradient df/dx of a vector-valued function `func` with
// the signature "eigenmath::VectorNd func(const eigenmath::VectorNd& x)"
// evaluated at `x` via numerical differentation by finite differences. Returns
// the derivative matrix as eigenmath::MatrixNMd of dimensions n x m, where `n`
// is the output-dimension of `func` and `m` is the dimension of `x` and the
// input-dimension of `func`. By default, approximates the gradient via
// dfdx = (f(x+h)-f(x)) / h. If `double_sided_derivative` is set to true, the
// finite differencing rule is applied to both sides of the setpoint, i.e.
// dfdx = (f(x+h)-f(x-h)) / 2h.
// Note: the implementation of this function does not implement the naive finite
// differencing rule, but a numerically more stable version.
eigenmath::MatrixNMd ComputeDerivative(
    absl::FunctionRef<eigenmath::VectorNd(const eigenmath::VectorNd&)> func,
    const eigenmath::VectorNd& x, bool double_sided_derivative = true);

// Approximates the derivative df/dx of a single-variable, vector-valued
// function `func` with the signature "eigenmath::VectorXd func(double x)"
// evaluated at `x` via numerical differentation by finite differences. Returns
// the derivative vector as eigenmath::VectorXd of dimensions `n`, where `n`
// is the output-dimension of `func`. By default, approximates the gradient via
// dfdx = (f(x+h)-f(x)) / h. If `double_sided_derivative` is set to true, the
// finite differencing rule is applied to both sides of the setpoint, i.e. dfdx
// = (f(x+h)-f(x-h)) / 2h. Note: the implementation of this function does not
// implement the naive finite differencing rule, but a numerically more stable
// version.
eigenmath::VectorXd ComputeDerivative(
    absl::FunctionRef<eigenmath::VectorXd(double x)> func, double x,
    bool double_sided_derivative = true);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_NUMOPT_FUNCTION_LINEARIZER_NUMDIFF_H_
