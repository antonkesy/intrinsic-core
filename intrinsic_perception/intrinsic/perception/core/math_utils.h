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

#ifndef INTRINSIC_PERCEPTION_CORE_MATH_UTILS_H_
#define INTRINSIC_PERCEPTION_CORE_MATH_UTILS_H_

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/core/argument_checks.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/util/status/ret_check.h"

namespace intrinsic {
namespace perception {

struct AngleTraits {
  struct NormalizeTag {};
  struct DontNormalizeTag {};

  static inline constexpr NormalizeTag kNormalize;
  static inline constexpr DontNormalizeTag kDontNormalize;
};

namespace internal {

template <typename NormalizationTag, typename VectorTypeA, typename VectorTypeB>
constexpr auto NormalizedDot(const VectorTypeA& a, const VectorTypeB& b) {
  if constexpr (std::is_same_v<NormalizationTag, AngleTraits::NormalizeTag>) {
    return a.normalized().dot(b.normalized());
  } else if constexpr (std::is_same_v<NormalizationTag,
                                      AngleTraits::DontNormalizeTag>) {
    DCHECK_OK(HasUnitLength(a));
    DCHECK_OK(HasUnitLength(b));
    return a.dot(b);
  }
}

}  // namespace internal

// Returns pi.
template <typename T>
constexpr T Pi() {
  return static_cast<T>(M_PI);
}

// Returns pi/2.
template <typename T>
constexpr T Pi2() {
  return static_cast<T>(M_PI_2);
}

// Returns true, if the passed value is a power of two.
constexpr bool IsPowerOfTwo(int32_t x) {
  return ((x & (x - 1)) == 0) && (x > 0);
}

constexpr int RoundUpToPowerOfTwo(int32_t i) {
  int j = 1;
  while (i > j) {
    j *= 2;
  }
  return j;
}

// Computes and returns the square of 'val'.
template <class T>
constexpr T Sqr(T val) {
  return val * val;
}

// Gives back the signum of floating point numbers.
template <class T>
constexpr T Sgn(T val) {
  return val >= T(0) ? T(1) : T(-1);
}

template <typename T>
bool IsApprox(T a, T b, T eps) {
  return std::abs(a - b) <= eps;
}

// Converts degrees to radians.
//
// The computed result is \f$degree * \frac{\pi}{180}\f$.
template <typename T>
constexpr T Deg2Rad(T degree) {
  static_assert(std::is_floating_point<T>::value,
                "Deg2Rad should use floating point type");
  return degree * (static_cast<T>(M_PI) / static_cast<T>(180));
}

// Converts radians to degrees.
//
// The computed result is \f$radian * \frac{180}{\pi}\f$.
template <typename T>
constexpr T Rad2Deg(T radian) {
  static_assert(std::is_floating_point<T>::value,
                "Deg2Rad should use floating point type");
  return radian * (static_cast<T>(180) / static_cast<T>(M_PI));
}

// Returns true, if the cosine of the angle between 'a' and 'b' is smaller than
// the specified cosine angle.
//
// If if AngleTraits::kDontNormalize is specified as the normalization tag,
// the input vectors 'a' and 'b' must have length |a| = 1.
template <typename T, int OptionsA, int OptionsB,
          typename NormalizationTag = AngleTraits::NormalizeTag>
bool IsCosAngleSmaller(const Vector2<T, OptionsA>& a,
                       const Vector2<T, OptionsB>& b, T cos_max_angle_rad,
                       NormalizationTag unused = AngleTraits::kNormalize) {
  // The dot product ranges between [-1, 1] and 1 is the largest value for
  // perfectly aligned vectors. The direction vectors have both unit length!
  return internal::NormalizedDot<NormalizationTag>(a, b) > cos_max_angle_rad;
}

// Returns true, if the cosine of the minimal (or smallest) angle between 'a'
// and 'b' is smaller than the specified cosine angle.
// This function can be interpreted as computing the smallest angle beweeen
// the undirected line-segments defined by 'a' and 'b'.
//
// If if AngleTraits::kDontNormalize is specified as the normalization tag,
// the input vectors 'a' and 'b' must have length |a| = 1.
// The cos_max_angle_rad must be in [cos(pi/2), cos(0)[ = [0, 1[
//
template <typename T, int OptionsA, int OptionsB,
          typename NormalizationTag = AngleTraits::NormalizeTag>
bool IsMinCosAngleSmaller(const Vector2<T, OptionsA>& a,
                          const Vector2<T, OptionsB>& b, T cos_max_angle_rad,
                          NormalizationTag unused = AngleTraits::kNormalize) {
  // We need the epsilon here since std::cos(pi/2) can become just negative.
  DCHECK_GE(cos_max_angle_rad + std::numeric_limits<T>::epsilon(), 0)
      << "The cosine of the angle must be >= 0.";
  DCHECK_LT(cos_max_angle_rad, 1) << "The cosine of the angle must be < 1.";
  // The absolute dot product ranges between [0, 1] and 1 is the largest value
  // for perfectly aligned vectors. The direction vectors have both unit length!
  return std::abs(internal::NormalizedDot<NormalizationTag>(a, b)) >
         cos_max_angle_rad;
}

// Returns true, if the angle between the two normalized vectors is smaller than
// the user specified angle.
//
// The inpute angle must be in ]0, pi], i.e. ]0, 180] degrees. Consequenctly,
// two opposing vectors are considered to span an angle of 180 degrees and not
// 0. In case the caller does not care about the normal direction but only the
// smallest angle, consider using IsAngleSmaller().
//
// If if AngleTraits::kDontNormalize is specified as the normalization tag,
// the input vectors 'a' and 'b' must have length |a| = 1.
// The input angle is specified in radians in ]0, pi].
template <typename T, int OptionsA, int OptionsB,
          typename NormalizationTag = AngleTraits::NormalizeTag>
bool IsAngleSmaller(
    const Vector2<T, OptionsA>& a, const Vector2<T, OptionsB>& b,
    T max_angle_rad,
    NormalizationTag normalization_tag = AngleTraits::kNormalize) {
  DCHECK_GT(max_angle_rad, 0) << "Angle must be > 0.";
  DCHECK_LE(max_angle_rad, Pi<T>()) << "Angle must be <= pi.";
  const T cos_max_angle_rad = std::cos(max_angle_rad);
  return IsCosAngleSmaller(a, b, cos_max_angle_rad, normalization_tag);
}

// Returns true, if the minimum (or smallest) angle between the vectors 'a' and
// 'b' is smaller thatn the use specified angle.
template <typename T, int OptionsA, int OptionsB,
          typename NormalizationTag = AngleTraits::NormalizeTag>
bool IsMinAngleSmaller(
    const Vector2<T, OptionsA>& a, const Vector2<T, OptionsB>& b,
    T max_angle_rad,
    NormalizationTag normalization_tag = AngleTraits::kNormalize) {
  DCHECK_GT(max_angle_rad, 0) << "Angle must be > 0.";
  DCHECK_LE(max_angle_rad, Pi2<T>()) << "Angle must be <= pi/2.";
  const T cos_max_angle_rad = std::cos(max_angle_rad);
  return IsMinCosAngleSmaller(a, b, cos_max_angle_rad, normalization_tag);
}

// Returns the signed angle between vector a -> b in the range (-pi, +pi[.
//
// The input vectors do not need to be normalized.
// The function returns an angle such that:
//   R(angle) * a = b
template <typename T, int OptionsA, int OptionsB>
T SignedAngle(const Vector2<T, OptionsA>& a, const Vector2<T, OptionsB>& b) {
  // This is a special case of the 3D version where z-coordinates are all 0.
  // The following equations hold:
  //   |a x b| = |a| |b| sin(theta)   where |x| the norm of x
  //    a . b  = |a| |b| cos(theta)   where . the dot-product
  // We also know that
  //   tan(theta) = sin(theta) / cos(theta)
  // and thus we get
  //   tan(theta) = |a x b| / a . b
  // and
  //   theta = atan2(a . b, |a x b|)
  const T dot = a.dot(b);
  const T det = a.x() * b.y() - a.y() * b.x();
  return std::atan2(det, dot);
}

// Returns the signed angle between vector a -> b in the range (-pi, +pi[.
//
// The function returns an angle such that:
//   Eigen::AngleAxis(angle, axis.normalized()) * a = b
//
// The input vectors a, b and axis do not need to be normalized for the returned
// angle to be correct.
template <typename T, int OptionsA, int OptionsB>
T SignedAngle(const Vector<T, 3, OptionsA>& a, const Vector<T, 3, OptionsB>& b,
              const Vector<T, 3, OptionsB>& axis) {
  // The following equations hold:
  //    a x b = |a| |b| sin(theta) n   where |x| the norm of x,
  //                                   n is a unit vector perpendicular to
  //                                   a and b
  //    a . b = |a| |b| cos(theta)     where . the dot-product
  //
  // We also know that
  //   n . axis.normalized() = 1     when theta < 180
  //   n . axis.normalized() = -1    when theta > 180
  // and thus we get
  //   (a x b) . axis = |a| |b| sin(theta)
  // and
  //   tan(theta) = sin(theta) / cos(theta)
  //              = (a x b) . axis / a . b
  // which follows
  //   theta = atan2(a . b, |a x b|)
  return std::atan2(a.cross(b).dot(axis), a.dot(b));
}

// Returns the angle between two vectors in the range [0, pi].
//
// The function assumes that non of the input vectors is normalized and performs
// the normalization internally.
template <typename T, int N, int OptionsA, int OptionsB,
          typename NormalizationTag = AngleTraits::NormalizeTag>
T Angle(const Vector<T, N, OptionsA>& a, const Vector<T, N, OptionsB>& b,
        NormalizationTag unused = AngleTraits::kNormalize) {
  const T dir_dot_product = internal::NormalizedDot<NormalizationTag>(a, b);
  // The clamping is required since we may have values of 1 + eps which
  // results purely from numerical instabilities.
  return std::acos(
      std::clamp(dir_dot_product, static_cast<T>(-1), static_cast<T>(1)));
}

// Returns the smallest angle between two directionless vectors in the range [0,
// pi/2].
//
// The function assumes that none of the input vectors is normalized and
// performs the normalization internally.
template <typename T, int N, int OptionsA, int OptionsB,
          typename NormalizationTag = AngleTraits::NormalizeTag>
T MinAngle(const Vector<T, N, OptionsA>& a, const Vector<T, N, OptionsB>& b,
           NormalizationTag unused = AngleTraits::kNormalize) {
  const T dir_dot_product =
      std::abs(internal::NormalizedDot<NormalizationTag>(a, b));
  // The clamping is required since we may have values of 1 + eps which
  // results purely from numerical instabilities.
  return std::acos(
      std::clamp(dir_dot_product, static_cast<T>(-1), static_cast<T>(1)));
}

// Returns the angle [0, pi] between two rotation matrices.
// The function assumes that both input values are correct rotation matrices.
template <typename T, int OptionsA, int OptionsB>
T MatrixAngle(const Matrix3<T, OptionsA>& a, const Matrix3<T, OptionsB>& b) {
  return Eigen::AngleAxis<T>(a * b.transpose()).angle();
}

// Concatenates matrices or vectors by row. All inputs must have the same number
// of columns but the number of rows can differ.
//
// Example usage:
// std::vector<MatrixXf> matrices = ...;
// MatrixXf merged_matrix = ConcatenateRows(matrices);
// std::vector<VectorXd> vectors = ...;
// VectorXd merged_vector = ConcatenateRows(vectors);
template <typename Scalar, int rows, int cols>
absl::StatusOr<Eigen::Matrix<Scalar, rows, cols>> ConcatenateRows(
    const std::vector<Eigen::Matrix<Scalar, rows, cols>>& matrices) {
  using MatrixType = Eigen::Matrix<Scalar, rows, cols>;
  INTR_RET_CHECK(!matrices.empty()).SetCode(absl::StatusCode::kInvalidArgument);
  const int num_cols = matrices.front().cols();
  int num_rows = 0;
  for (const MatrixType& matrix : matrices) {
    INTR_RET_CHECK(matrix.cols() == num_cols)
        .SetCode(absl::StatusCode::kInvalidArgument);
    num_rows += matrix.rows();
  }
  MatrixType merged_matrix(num_rows, num_cols);
  int current_row = 0;
  for (const MatrixType& matrix : matrices) {
    merged_matrix.middleRows(current_row, matrix.rows()) = matrix;
    current_row += matrix.rows();
  }
  return merged_matrix;
}

// Compute rotation axis (direction+origin) of a rigid transformation.
// Return true iff the result can be computed (i.e., if 't' contains a non-zero
// rotation, so that the axis is well defined). The returned origin can be any
// point on the rotation axis.
template <typename Scalar>
bool ComputeRotationAxis(const Isometry3<Scalar>& t,
                         Vector3<Scalar>& rotation_axis,
                         Vector3<Scalar>& axis_origin) {
  const Scalar kMinAbsRotationAngle = 1e-10;
  const Eigen::AngleAxis<Scalar> angle_axis(t.rotation());
  if (std::abs(angle_axis.angle()) < kMinAbsRotationAngle) {
    rotation_axis = Vector3<Scalar>(1, 0, 0);
    axis_origin = Vector3<Scalar>::Zero();
    return false;
  }
  rotation_axis = angle_axis.axis();

  // Compute a rotation axis origin by solving a small linear system.
  // Derivation: Given the input transform as Rx+t we want to find an 'o' which
  // acts as the rotation origin:
  // Ro+t = o
  // t = o-Ro = (I-R)o
  Eigen::Matrix<Scalar, 4, 3> A;
  Vector4<Scalar> b;
  A.block(0, 0, 3, 3) = Matrix3<Scalar>::Identity() - t.rotation().matrix();
  b.template head<3>() = t.translation();
  // To make the linear system full rank we constrain the origin to the plane
  // rotation_axis * x =  0.
  A.row(3) = rotation_axis.transpose();
  b(3) = 0;
  axis_origin =
      A.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);
  return true;
}

template <class Scalar, int Options = Eigen::DontAlign>
Vector3<Scalar, Options> ComputeOrthogonalVector(
    const Vector3<Scalar, Options>& v) {
  Vector3<Scalar, Options> s2 = v.normalized();
  Vector3<Scalar, Options> s0 = Vector3<Scalar, Options>::UnitX();
  if (std::abs(s2.dot(s0)) >
      std::abs(s2.dot(Vector3<Scalar, Options>::UnitY()))) {
    s0 = Vector3<Scalar, Options>::UnitY();
  }
  if (std::abs(s2.dot(s0)) >
      std::abs(s2.dot(Vector3<Scalar, Options>::UnitZ()))) {
    s0 = Vector3<Scalar, Options>::UnitZ();
  }
  Vector3<Scalar, Options> s1 = s2.cross(s0).normalized();
  s0 = s1.cross(s2).normalized();
  return s0;
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_MATH_UTILS_H_
