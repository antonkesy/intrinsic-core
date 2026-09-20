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

#include "intrinsic/math/projection_matrix_utils.h"

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic {

icon::RealtimeStatusOr<eigenmath::MatrixNd>
ComputeOrthogonalColumnSpaceProjector(
    const eigenmath::MatrixNMd& C, double singular_value_magnitude_threshold) {
  if (C.cols() > C.rows()) {
    return icon::FailedPreconditionError(
        "Cannot compute ColumnSpaceProjector for matrix C, since C has more "
        "columns than rows.");
  }

  if (singular_value_magnitude_threshold < 0.0) {
    return icon::FailedPreconditionError(
        "singular_value_magnitude_threshold must be greater than zero.");
  }

  // Check singular values to make sure C does not contain collinear vectors.
  Eigen::JacobiSVD<eigenmath::MatrixNMd> svd(C, Eigen::ComputeThinU);

  // Get singular values, they are already sorted in descending order. Compare
  // the smallest singular value against the threshold for determining if matrix
  // C is rank-deficient.
  const auto& singular_values = svd.singularValues();
  if (singular_values.tail<1>()(0) < singular_value_magnitude_threshold) {
    return icon::FailedPreconditionError(
        "Cannot compute ColumnSpaceProjector for matrix C, since C's columns "
        "are not linearly independent.");
  }

  // Efficiently compute the column space projector P =  C* (C.transpose() *
  // C).inverse() * C.transpose() using above's SVD. Given the SVD C =
  // U*S*V.transpose(), the column space projector becomes P = U *
  // U.transpose().
  return eigenmath::MatrixNd(svd.matrixU() * svd.matrixU().transpose());
}

icon::RealtimeStatusOr<eigenmath::MatrixNd> ComputeOrthogonalNullSpaceProjector(
    const eigenmath::MatrixNMd& C, double singular_value_magnitude_threshold) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(eigenmath::MatrixNd P_column_space,
                                ComputeOrthogonalColumnSpaceProjector(
                                    C, singular_value_magnitude_threshold));
  // P_nullspace = I - P_column_space.
  return eigenmath::MatrixNd(eigenmath::MatrixNd::Identity(C.rows(), C.rows()) -
                             P_column_space);
}

}  // namespace intrinsic
