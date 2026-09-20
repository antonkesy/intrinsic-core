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

#ifndef INTRINSIC_PERCEPTION_CORE_EIGEN_TYPES_H_
#define INTRINSIC_PERCEPTION_CORE_EIGEN_TYPES_H_

#include <cstdint>

#include "Eigen/Core"         // IWYU pragma: export
#include "Eigen/Eigenvalues"  // IWYU pragma: export
#include "Eigen/Geometry"     // IWYU pragma: export

namespace intrinsic {
namespace perception {
// We define the custom vector/matrix types here to not
// use the aligned version of Eigen (which causes several issues
// especially if used as pixel traits where we can not used aligned
// pixel traits). This might result in slower performance but allows us
// to use Eigen implicitly in our image class without the need of copying data.
// We will use only these structs/classes in the perception framework.

/** @brief N-vector using Scalar */
template <class Scalar, int N, int Options = Eigen::DontAlign>
using Vector = Eigen::Matrix<Scalar, N, 1, Options>;

/** @brief 2-vector using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Vector2 = Vector<Scalar, 2, Options>;

using Vector2f = Vector2<float, Eigen::DontAlign>;
using Vector2d = Vector2<double, Eigen::DontAlign>;
using Vector2i = Vector2<int32_t, Eigen::DontAlign>;
using Vector2u = Vector2<uint32_t, Eigen::DontAlign>;
using Vector2b = Vector2<uint8_t, Eigen::DontAlign>;

/** @brief 3-vector using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Vector3 = Vector<Scalar, 3, Options>;

using Vector3f = Vector3<float, Eigen::DontAlign>;
using Vector3d = Vector3<double, Eigen::DontAlign>;
using Vector3i = Vector3<int32_t, Eigen::DontAlign>;
using Vector3u = Vector3<uint32_t, Eigen::DontAlign>;
using Vector3b = Vector3<uint8_t, Eigen::DontAlign>;

/** @brief 4-vector using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Vector4 = Vector<Scalar, 4, Options>;

using Vector4f = Vector4<float, Eigen::DontAlign>;
using Vector4d = Vector4<double, Eigen::DontAlign>;
using Vector4i = Vector4<int32_t, Eigen::DontAlign>;
using Vector4u = Vector4<uint32_t, Eigen::DontAlign>;
using Vector4b = Vector4<uint8_t, Eigen::DontAlign>;

/** @brief 5-vector using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Vector5 = Vector<Scalar, 5, Options>;

using Vector5f = Vector5<float, Eigen::DontAlign>;
using Vector5d = Vector5<double, Eigen::DontAlign>;
using Vector5i = Vector5<int32_t, Eigen::DontAlign>;
using Vector5u = Vector5<uint32_t, Eigen::DontAlign>;
using Vector5b = Vector5<uint8_t, Eigen::DontAlign>;

/** @brief 6-vector using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Vector6 = Vector<Scalar, 6, Options>;

using Vector6f = Vector6<float, Eigen::DontAlign>;
using Vector6d = Vector6<double, Eigen::DontAlign>;
using Vector6i = Vector6<int32_t, Eigen::DontAlign>;
using Vector6u = Vector6<uint32_t, Eigen::DontAlign>;
using Vector6b = Vector6<uint8_t, Eigen::DontAlign>;

/** @brief N-vector (runtime) using Scalar */
template <typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using VectorXd = VectorX<double>;
using VectorXf = VectorX<float>;
using VectorXi = VectorX<int32_t>;
using VectorXu = VectorX<uint32_t>;

// using Eigen::Matrix;
template <class Scalar, int Rows, int Cols,
          int Options = Eigen::DontAlign |
                        ((Rows == 1 && Cols != 1) ? Eigen::RowMajor
                         : (Cols == 1 && Rows != 1)
                             ? Eigen::ColMajor
                             : EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION)>
using Matrix = Eigen::Matrix<Scalar, Rows, Cols, Options>;

/** @brief (2x2) matrix using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Matrix2 = Matrix<Scalar, 2, 2, Options>;

using Matrix2f = Matrix2<float, Eigen::DontAlign>;
using Matrix2d = Matrix2<double, Eigen::DontAlign>;
using Matrix2i = Matrix2<int32_t, Eigen::DontAlign>;
using Matrix2u = Matrix2<uint32_t, Eigen::DontAlign>;

/** @brief (3x3) matrix using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Matrix3 = Matrix<Scalar, 3, 3, Options>;

using Matrix3f = Matrix3<float, Eigen::DontAlign>;
using Matrix3d = Matrix3<double, Eigen::DontAlign>;
using Matrix3i = Matrix3<int32_t, Eigen::DontAlign>;
using Matrix3u = Matrix3<uint32_t, Eigen::DontAlign>;

/** @brief (4x4) matrix using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Matrix4 = Matrix<Scalar, 4, 4, Options>;

using Matrix4f = Matrix4<float, Eigen::DontAlign>;
using Matrix4d = Matrix4<double, Eigen::DontAlign>;
using Matrix4i = Matrix4<int32_t, Eigen::DontAlign>;
using Matrix4u = Matrix4<uint32_t, Eigen::DontAlign>;

/** @brief (5x5) matrix using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Matrix5 = Matrix<Scalar, 5, 5, Options>;

using Matrix5f = Matrix5<float, Eigen::DontAlign>;
using Matrix5d = Matrix5<double, Eigen::DontAlign>;
using Matrix5i = Matrix5<int32_t, Eigen::DontAlign>;
using Matrix5u = Matrix5<uint32_t, Eigen::DontAlign>;

/** @brief 6 x 6 matrix using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Matrix6 = Matrix<Scalar, 6, 6, Options>;

using Matrix6f = Matrix6<float, Eigen::DontAlign>;
using Matrix6d = Matrix6<double, Eigen::DontAlign>;
using Matrix6i = Matrix6<int32_t, Eigen::DontAlign>;
using Matrix6u = Matrix6<uint32_t, Eigen::DontAlign>;

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::MatrixXi;

/** @brief Quaternion */
template <class Scalar, int Options = Eigen::DontAlign>
using Quaternion = Eigen::Quaternion<Scalar, Options>;

using Quaternionf = Quaternion<float, Eigen::DontAlign>;
using Quaterniond = Quaternion<double, Eigen::DontAlign>;

/** @brief 3d plane using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Plane3 = Eigen::Hyperplane<Scalar, 3, Options>;

using Plane3d = Plane3<double, Eigen::DontAlign>;
using Plane3f = Plane3<float, Eigen::DontAlign>;

/** @brief 2d isometry using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Isometry2 = Eigen::Transform<Scalar, 2, Eigen::Isometry, Options>;

using Isometry2f = Isometry2<float, Eigen::DontAlign>;

/** @brief 3d affine transformation using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine, Options>;

using Affine3f = Affine3<float, Eigen::DontAlign>;
using Affine3d = Affine3<double, Eigen::DontAlign>;

/** @brief 3d isometry using Scalar */
template <class Scalar, int Options = Eigen::DontAlign>
using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry, Options>;

using Isometry3f = Isometry3<float, Eigen::DontAlign>;
using Isometry3d = Isometry3<double, Eigen::DontAlign>;

/** @brief Angle axis */
using AngleAxisd = Eigen::AngleAxisd;
using AngleAxisf = Eigen::AngleAxisf;

/** @brief 2D Rotation */
template <class Scalar>
using Rotation2D = Eigen::Rotation2D<Scalar>;

using Rotation2Df = Rotation2D<float>;
using Rotation2Dd = Rotation2D<double>;

/** @brief Self Adjoint Eigen Solver */
template <class T>
using SelfAdjointEigenSolver = Eigen::SelfAdjointEigenSolver<T>;

/** @brief Jacobi SVD */
template <class T>
using JacobiSVD = Eigen::JacobiSVD<T>;

/** @brief Flag for Jacobi SVD */
constexpr Eigen::DecompositionOptions EigenComputeFullU = Eigen::ComputeFullU;
constexpr Eigen::DecompositionOptions EigenComputeFullV = Eigen::ComputeFullV;

/** @brief Flag for Eigen */
constexpr Eigen::UpLoType EigenLower = Eigen::Lower;

/** @brief Flag for Eigen */
constexpr Eigen::StorageOptions EigenDontAlign = Eigen::DontAlign;

// Convenience function to create an isometry from rotation and translation.
template <class Scalar, int Dimensions, int Options = Eigen::DontAlign>
Eigen::Transform<Scalar, Dimensions, Eigen::Isometry, Options> CreateIsometry(
    const Matrix<Scalar, Dimensions, Dimensions, Options>& rotation,
    const Vector<Scalar, Dimensions, Options>& translation) {
  Eigen::Transform<Scalar, Dimensions, Eigen::Isometry, Options> isometry =
      Eigen::Transform<Scalar, Dimensions, Eigen::Isometry,
                       Options>::Identity();
  isometry.translation() = translation;
  isometry.linear() = rotation;
  return isometry;
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_EIGEN_TYPES_H_
