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

#ifndef INTRINSIC_MATH_SYMMETRIC_POSITIVE_DEFINITE_MATRIX_SQUAREROOT_H_
#define INTRINSIC_MATH_SYMMETRIC_POSITIVE_DEFINITE_MATRIX_SQUAREROOT_H_

#include <utility>

#include "absl/log/log.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic {

// Computes the square-root of a symmetric positive-definite matrix. Returns
// false in case of an invalid input matrix or computation error. Returns the
// matrix square root as output parameter.
icon::RealtimeStatusOr<eigenmath::MatrixNd>
SymmetricPositiveDefiniteMatrixSquareRoot(const eigenmath::MatrixNd& mat);

// Computes the entropy-regulated Wasserstein distance interpolation between
// two symmetric positive-definite matrices. Implements Equation (85) from
// https://link.springer.com/article/10.1007/s41884-021-00052-8.
//
// Important note: singular matrices (such as orthogonal projection matrices)
// are not supported and need to be interpolated differently.
class EntropyRegulatedWassersteinDistanceInterpolation {
 public:
  // Creates an interpolation between the two matrices. The main (expensive)
  // computations are done in this factory.
  static absl::StatusOr<EntropyRegulatedWassersteinDistanceInterpolation>
  Create(const eigenmath::MatrixNd& mat_1, const eigenmath::MatrixNd& mat_2);

  // Interpolate between the two matrices using the given `alpha`, where alpha
  // is in the range [0, 1]. Has almost no computational cost, all involved
  // operations are trivial.
  icon::RealtimeStatusOr<eigenmath::MatrixNd> Interpolate(double alpha) const;

 private:
  EntropyRegulatedWassersteinDistanceInterpolation(
      eigenmath::MatrixNd mat_1, eigenmath::MatrixNd mat_2,
      eigenmath::MatrixNd mat_1_times_mat_2_sqrt,
      eigenmath::MatrixNd mat_2_times_mat_1_sqrt)
      : mat_1_(std::move(mat_1)),
        mat_2_(std::move(mat_2)),
        mat_1_times_mat_2_sqrt_(std::move(mat_1_times_mat_2_sqrt)),
        mat_2_times_mat_1_sqrt_(std::move(mat_2_times_mat_1_sqrt)) {}

  eigenmath::MatrixNd mat_1_;
  eigenmath::MatrixNd mat_2_;
  eigenmath::MatrixNd mat_1_times_mat_2_sqrt_;
  eigenmath::MatrixNd mat_2_times_mat_1_sqrt_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SYMMETRIC_POSITIVE_DEFINITE_MATRIX_SQUAREROOT_H_
