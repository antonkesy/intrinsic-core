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

#ifndef INTRINSIC_PERCEPTION_CORE_POSE_UTILS_H_
#define INTRINSIC_PERCEPTION_CORE_POSE_UTILS_H_

#include <type_traits>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::perception {

// Returns the translational error in meter.
// The function returns the euclidean distance between the endpoints of the two
// translation vectors.
double ComputePoseTranslationError(const Pose3d& lhs_estimated_pose,
                                   const Pose3d& rhs_estimated_pose);

double ComputePoseTranslationError(const Eigen::Isometry3d& lhs_estimated_pose,
                                   const Eigen::Isometry3d& rhs_estimated_pose);

// Returns the element-wise translational error along each axis.
// The function computes the absolute error along all of the translation axis.
double ComputeElementWiseMaximumTranslationError(
    const Pose3d& lhs_estimated_pose, const Pose3d& rhs_estimated_pose);

double ComputeElementWiseMaximumTranslationError(
    const Eigen::Isometry3d& lhs_estimated_pose,
    const Eigen::Isometry3d& rhs_estimated_pose);

// Returns the rotation error in radians.
// The error is computed as the angle of the quaternion which results from
// multiplying the inverse of the first input pose with the second input pose.
double ComputePoseRotationError(const Pose3d& lhs_estimated_pose,
                                const Pose3d& rhs_estimated_pose);

double ComputePoseRotationError(const Eigen::Isometry3d& lhs_estimated_pose,
                                const Eigen::Isometry3d& rhs_estimated_pose);

// Computes an average between all quaternions.
// The methods implements:
// https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
absl::StatusOr<Eigen::Quaternion<double>> AverageRotations(
    const std::vector<Eigen::Quaternion<double>>& rotations);

// Returns the average of all input translations.
absl::StatusOr<Eigen::Vector3<double>> AverageTranslations(
    const std::vector<Eigen::Vector3<double>>& translations);

// Computes an average between all input poses.
// Computes the average of all poses assuming the input does not contain any
// outliers.
absl::StatusOr<Pose3d> AveragePoses(const std::vector<Pose3d>& poses);

// Returns the average pose of the largest cluster computed by the meanshift
// algorithm.
// The rotation radius (radians) and translation radius (meters) are
// used to determine if pose samples are close to each other. The 'closeness' is
// corresponding to the bandwidth parameter in the mean-shift algorithm. It
// strongly affects the cluster size from which the pose average is computed.
// The larger the radii, the larger the cluster over which the final pose is
// averaged.
// TODO(b/141840549): Implement automatic bandwidth estimation.
absl::StatusOr<Pose3d> AveragePosesWithMeanShift(
    const std::vector<Pose3d>& poses, double rotation_radius,
    double translation_radius);

// Generates random uniform pose jitter around identity with a random rotation
// axis and a rotation angle of rotation_perturbation_in_radian and a random
// translation axis with length translation_perturbation_in_meters.
template <typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Isometry>
GenerateUniformPoseJitterAroundIdentity(
    Scalar rotation_perturbation_in_radian,
    Scalar translation_perturbation_in_meters) {
  static_assert(
      std::is_floating_point<Scalar>::value,
      "GeneratePoseJitterAroundIdentity should use floating point type");

  Eigen::Transform<Scalar, 3, Eigen::Isometry> pose_jitter_around_identity =
      Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
  pose_jitter_around_identity.linear() =
      Eigen::AngleAxis<Scalar>(
          rotation_perturbation_in_radian,
          Eigen::Matrix<Scalar, 3, 1>::Random().normalized())
          .toRotationMatrix();
  pose_jitter_around_identity.translation() =
      Eigen::Matrix<Scalar, 3, 1>::Random().normalized() *
      translation_perturbation_in_meters;

  return pose_jitter_around_identity;
}

// Generates random pose around identity with a random rotation axis and
// standard_rotation_perturbation_in_radian for standard deviation of the
// rotation angle and a random tranlation axis with
// standard_translation_perturbation_in_meters as the standard deviation for the
// length of the translation.
Pose3d GenerateRandomGaussianPoseAroundIdentity(
    double standard_rotation_perturbation_in_radian,
    double standard_translation_perturbation_in_meters);

// Generates random pose around identity with a random rotation axis and
// a uniformly distributed rotation angle in [-rotation_perturbation_in_radian,
// rotation_perturbation_in_radian] range and a random tranlation axis with a
// length uniformly distributed in the [-translation_perturbation_in_meters,
// translation_perturbation_in_meters] range.
Pose3d GenerateRandomUniformPoseAroundIdentity(
    double rotation_perturbation_in_radian,
    double translation_perturbation_in_meters);

// Compute yaw, pitch, roll angles from rotation matrix.
// We cannot use Eigen's EulerAngles directly, as they return the Euler Angles
// in following ranges: [0:pi]x[-pi:pi]x[-pi:pi]. To make the angles more human
// interpretable for this use case, we want to follow the Tait-Bryan convention
// for yaw, pitch, roll angles: [-pi:pi]x[-pi/2:pi/2]x[-pi:pi]. This way, all
// angles are centered around 0, so the tilting angle should cover +-45 degrees.
//
// This implementation follows the threejs implementation for the 'XYZ' rotation
// order: https://github.com/mrdoob/three.js/blob/master/src/math/Euler.js
Eigen::Vector3d GetYawPitchRollFromRotationMatrix(
    const Eigen::Matrix3d& rotation);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CORE_POSE_UTILS_H_
