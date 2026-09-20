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

#include "intrinsic/eigenmath/invert_lower_triangular_matrix.h"

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic {

namespace {

constexpr double kEpsilon = 1.0e-10;

}  // namespace

icon::RealtimeStatusOr<eigenmath::MatrixNd> InvertLowerTriangularMatrix(
    const eigenmath::MatrixNd& lower_triangular_matrix) {
  if (!lower_triangular_matrix.isLowerTriangular(kEpsilon)) {
    return icon::InvalidArgumentError("Input matrix is not lower triangular.");
  }

  // Get the size of the matrix.
  const int n = lower_triangular_matrix.rows();

  // Use Eigen's highly optimized triangular solver.
  // This tells Eigen to view the matrix as lower triangular and solve L*X = I.
  return eigenmath::MatrixNd(
      lower_triangular_matrix.triangularView<Eigen::Lower>().solve(
          eigenmath::MatrixNd::Identity(n, n)));
}

}  // namespace intrinsic
