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

#ifndef INTRINSIC_MATH_PROJECTION_MATRIX_UTILS_H_
#define INTRINSIC_MATH_PROJECTION_MATRIX_UTILS_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

// Returns an NxN orthogonal projection matrix 'P' which projects an Nx1 vector
// 'b' into the column space of the NxM matrix 'C' by multiplication 'P*b'. The
// columns of 'C' need to be filled with linearly independent vectors, an
// icon::FailedPreconditionError is returned otherwise. Linear independence of
// the columns of 'C' is checked by singular value decomposition, if there are
// singular values smaller in magnitude than
// 'singular_value_magnitude_threshold', C will be considered rank-deficient.
icon::RealtimeStatusOr<eigenmath::MatrixNd>
ComputeOrthogonalColumnSpaceProjector(
    const eigenmath::MatrixNMd& C,
    double singular_value_magnitude_threshold = 1e-12);

// Returns an NxN orthogonal projection matrix 'P' which projects an Nx1 vector
// 'b' into the nullspace of the NxM matrix 'C' by multiplication 'P*b', such
// that 'C.transpose() * P * b = 0' for every 'b'. The columns of 'C' need to be
// filled with linearly independent vectors, an icon::FailedPreconditionError is
// returned otherwise. Linear independence of the columns of 'C' is checked by
// singular value decomposition, if there are singular values smaller in
// magnitude than 'singular_value_magnitude_threshold', C will be considered
// rank-deficient.
icon::RealtimeStatusOr<eigenmath::MatrixNd> ComputeOrthogonalNullSpaceProjector(
    const eigenmath::MatrixNMd& C,
    double singular_value_magnitude_threshold = 1e-12);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_PROJECTION_MATRIX_UTILS_H_
