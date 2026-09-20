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

#ifndef INTRINSIC_EIGENMATH_INVERT_LOWER_TRIANGULAR_MATRIX_H_
#define INTRINSIC_EIGENMATH_INVERT_LOWER_TRIANGULAR_MATRIX_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic {

// Inverts a dense lower triangular matrix efficiently.
//
// This function uses Eigen's built-in optimized triangular solver to find the
// inverse. It solves the matrix equation L * X = I, where L is the input
// matrix, I is the Identity matrix, and X is the resulting inverse. This is
// significantly faster and numerically more stable than calling a generic
// .inverse() method.
icon::RealtimeStatusOr<eigenmath::MatrixNd> InvertLowerTriangularMatrix(
    const eigenmath::MatrixNd& lower_triangular_matrix);

}  // namespace intrinsic

#endif  // INTRINSIC_EIGENMATH_INVERT_LOWER_TRIANGULAR_MATRIX_H_
