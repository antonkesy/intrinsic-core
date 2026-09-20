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

#include "intrinsic/math/numopt/function_linearizer_numdiff.h"

#include <cmath>

#include "Eigen/Core"
#include "absl/functional/any_invocable.h"
#include "absl/functional/function_ref.h"
#include "absl/log/check.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

namespace {

// Templated implementation allows to write the numerical differentiation
// routine independently of input and output matrix types, thus generalizing to
// both dynamic and fixed-size Eigen-types.
template <typename VectorType, typename MatrixType>
MatrixType ComputeDerivativeImpl(
    absl::FunctionRef<VectorType(const VectorType&)> func, const VectorType& x,
    bool double_sided_derivative) {
  CHECK_GT(x.size(), 0);

  // Default perturbation for numerical differentiation.
  double eps = sqrt(Eigen::NumTraits<double>::epsilon());
  MatrixType dfdx;  // Derivative matrix, to be computed.

  // Reference evaluation of the function to be linearized.
  VectorType f_ref;
  // If only interested in a one-sided derivative, we store f(x) as reference
  // value.
  if (!double_sided_derivative) {
    f_ref = func(x);
  }
  VectorType x_perturbed = x;

  for (int i = 0; i < x.size(); ++i) {
    // Inspired from
    // http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
    double h = eps * std::max(std::abs(x(i)), 1.0);
    volatile double x_ph = x(i) + h;
    double dxp = x_ph - x(i);

    x_perturbed(i) = x_ph;

    // Evaluate function at perturbed state
    VectorType res_plus = func(x_perturbed);

    // After the first call to 'func' we know the output dimension of the
    // function can resize the gradient matrix accordingly.
    if (i == 0) {
      dfdx.resize(res_plus.rows(), x.rows());
    }

    if (double_sided_derivative) {
      volatile double x_mh = x(i) - h;
      double dxm = x(i) - x_mh;

      x_perturbed(i) = x_mh;

      VectorType res_minus = func(x_perturbed);
      dfdx.col(i) = (res_plus - res_minus) / (dxp + dxm);
    } else {
      dfdx.col(i) = (res_plus - f_ref) / dxp;
    }
    // Restore the original x value.
    x_perturbed(i) = x(i);
  }

  return dfdx;
}

}  // namespace

eigenmath::MatrixXd ComputeDerivative(
    absl::FunctionRef<eigenmath::VectorXd(const eigenmath::VectorXd&)> func,
    const eigenmath::VectorXd& x, bool double_sided_derivative) {
  return ComputeDerivativeImpl<eigenmath::VectorXd, eigenmath::MatrixXd>(
      func, x, double_sided_derivative);
}

eigenmath::MatrixNMd ComputeDerivative(
    absl::FunctionRef<eigenmath::VectorNd(const eigenmath::VectorNd&)> func,
    const eigenmath::VectorNd& x, bool double_sided_derivative) {
  return ComputeDerivativeImpl<eigenmath::VectorNd, eigenmath::MatrixNMd>(
      func, x, double_sided_derivative);
}

eigenmath::VectorXd ComputeDerivative(
    absl::FunctionRef<eigenmath::VectorXd(double x)> func, double x,
    bool double_sided_derivative) {
  absl::AnyInvocable<eigenmath::VectorXd(const eigenmath::VectorXd&) const>
      func_as_multi_variable_function =
          [&func](const eigenmath::VectorXd& x) { return func(x(0)); };
  eigenmath::VectorXd x_as_vector = eigenmath::VectorXd::Constant(1, x);

  return ComputeDerivativeImpl<eigenmath::VectorXd, eigenmath::VectorXd>(
      func_as_multi_variable_function, x_as_vector, double_sided_derivative);
}

}  // namespace intrinsic
