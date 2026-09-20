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

#include "intrinsic/perception/core/pose_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/random/distributions.h"
#include "absl/random/random.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/math_utils.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {
namespace {

// Returns all poses which belong to the provided cluster center pose.
std::vector<Pose3d> AssignToClusterCenter(const Pose3d& cluster_center_pose,
                                          const std::vector<Pose3d>& poses,
                                          double rotation_threshold,
                                          double translation_threshold) {
  const size_t num_poses = poses.size();
  std::vector<Pose3d> pose_cluster;
  pose_cluster.reserve(num_poses);
  for (size_t i = 0; i < num_poses; ++i) {
    const Pose3d& current_pose = poses[i];
    if (ComputePoseRotationError(current_pose, cluster_center_pose) <=
            rotation_threshold &&
        ComputePoseTranslationError(current_pose, cluster_center_pose) <=
            translation_threshold) {
      pose_cluster.push_back(current_pose);
    }
  }
  return pose_cluster;
}

// This function averages quaternions. It works even for quaternions which are
// not close to each other. The function implements:
// https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
template <typename Scalar>
absl::StatusOr<Eigen::Quaternion<Scalar>> GenericAverageRotations(
    const std::vector<Eigen::Quaternion<Scalar>>& rotations) {
  using Matrix4x4 = Eigen::Matrix<Scalar, 4, 4>;

  if (rotations.empty()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "No input rotations specified.";
  }

  Matrix4x4 A = Matrix4x4::Zero();
  for (const auto& q : rotations) {
    A += q.coeffs() * q.coeffs().transpose();
  }
  A /= static_cast<Scalar>(rotations.size());

  // A = sum_i q_i * q_i^T and thus it is self-adjoint.
  Eigen::SelfAdjointEigenSolver<Matrix4x4> eigensolver(A);
  if (eigensolver.info() != Eigen::Success) {
    return intrinsic::InternalErrorBuilder()
           << "Solving eigensystem during pose averaging failed.";
  }

  // The SelfAdjointEigenSolver returns eigenvectors in increasing order w.r.t.
  // to the eigenvalues. The eigenvectors contain the scalars [x, y, z, w]. See
  // the computation of A for a reference. There, Quaternion::coeffs() returns
  // [x, y, z, w].
  const Eigen::Matrix<Scalar, 4, 1> xyzw = eigensolver.eigenvectors().col(3);

  // The ctor of Quaternion requires [w, x, y, z].
  return Eigen::Quaternion<Scalar>{xyzw[3], xyzw[0], xyzw[1], xyzw[2]};
}

template <typename Scalar>
absl::StatusOr<Eigen::Vector3<Scalar>> GenericAverageTranslations(
    const std::vector<Eigen::Vector3<Scalar>>& translations) {
  using Vector3 = Eigen::Vector3<Scalar>;

  if (translations.empty()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "No input translations specified.";
  }

  Vector3 average_translation = Vector3::Zero();
  for (const Vector3& translation : translations) {
    average_translation += translation;
  }
  average_translation /= static_cast<Scalar>(translations.size());
  return average_translation;
}

template <typename Scalar, int Options>
double GenericComputePoseTranslationError(
    const Eigen::Matrix<Scalar, 3, 1, Options>& lhs_t,
    const Eigen::Matrix<Scalar, 3, 1, Options>& rhs_t) {
  return (lhs_t - rhs_t).norm();
}

template <typename Scalar, int Options>
double GenericComputePoseRotationError(
    const Eigen::Quaternion<Scalar, Options>& lhs_quat,
    const Eigen::Quaternion<Scalar, Options>& rhs_quat) {
  return Eigen::AngleAxisd(lhs_quat.inverse() * rhs_quat).angle();
}

class PoseComparator {
 public:
  PoseComparator(double rotation_threshold_rad,
                 double translation_threshold_meters)
      : rotation_threshold_rad_{rotation_threshold_rad},
        translation_threshold_meters_{translation_threshold_meters} {}
  bool operator()(const Pose3d& lhs, const Pose3d& rhs) const {
    return ComputePoseTranslationError(lhs, rhs) <=
               translation_threshold_meters_ &&
           ComputePoseRotationError(lhs, rhs) <= rotation_threshold_rad_;
  }

 private:
  double rotation_threshold_rad_;
  double translation_threshold_meters_;
};

struct Cluster {
  Pose3d center_pose;
  std::vector<Pose3d> meanshift_poses;
  std::vector<Pose3d> original_poses;
};

absl::StatusOr<Pose3d> MeanShiftPose(const Pose3d& pose,
                                     const std::vector<Pose3d>& poses,
                                     double rotation_radius,
                                     double translation_radius) {
  constexpr int kMaxIterations = 1000;
  constexpr double kRotationDeltaInDegrees = 0.001;
  constexpr double kTranslationDeltaInMeters = 0.00000001;

  Pose3d average_pose = pose;
  for (int i = 0; i < kMaxIterations; ++i) {
    const std::vector<Pose3d> pose_cluster = AssignToClusterCenter(
        average_pose, poses, rotation_radius, translation_radius);
    INTR_ASSIGN_OR_RETURN(const Pose3d current_average_pose,
                          AveragePoses(pose_cluster));

    // Compute the pose change compared to the current estimate of the
    // global average.
    const double relative_rotation_change_rad =
        ComputePoseRotationError(average_pose, current_average_pose);
    const double relative_translation_change_meter =
        ComputePoseTranslationError(average_pose, current_average_pose);

    average_pose = current_average_pose;
    // Perform an early termination in case the estimated average does not
    // differ from the currently computed average.
    if (Rad2Deg(relative_rotation_change_rad) < kRotationDeltaInDegrees &&
        relative_translation_change_meter < kTranslationDeltaInMeters) {
      break;
    }
  }
  return average_pose;
}

}  // namespace

double ComputePoseTranslationError(const Pose3d& lhs_estimated_pose,
                                   const Pose3d& rhs_estimated_pose) {
  return GenericComputePoseTranslationError(lhs_estimated_pose.translation(),
                                            rhs_estimated_pose.translation());
}

double ComputePoseTranslationError(
    const Eigen::Isometry3d& lhs_estimated_pose,
    const Eigen::Isometry3d& rhs_estimated_pose) {
  return GenericComputePoseTranslationError(
      lhs_estimated_pose.translation().eval(),
      rhs_estimated_pose.translation().eval());
}

double ComputeElementWiseMaximumTranslationError(
    const Pose3d& lhs_estimated_pose, const Pose3d& rhs_estimated_pose) {
  return (lhs_estimated_pose.translation() - rhs_estimated_pose.translation())
      .cwiseAbs()
      .maxCoeff();
}

double ComputeElementWiseMaximumTranslationError(
    const Eigen::Isometry3d& lhs_estimated_pose,
    const Eigen::Isometry3d& rhs_estimated_pose) {
  return (lhs_estimated_pose.translation() - rhs_estimated_pose.translation())
      .cwiseAbs()
      .maxCoeff();
}

double ComputePoseRotationError(const Pose3d& lhs_estimated_pose,
                                const Pose3d& rhs_estimated_pose) {
  return GenericComputePoseRotationError(lhs_estimated_pose.quaternion(),
                                         rhs_estimated_pose.quaternion());
}

double ComputePoseRotationError(const Eigen::Isometry3d& lhs_estimated_pose,
                                const Eigen::Isometry3d& rhs_estimated_pose) {
  return GenericComputePoseRotationError(
      Eigen::Quaterniond(lhs_estimated_pose.rotation()),
      Eigen::Quaterniond(rhs_estimated_pose.rotation()));
}

absl::StatusOr<Eigen::Quaternion<double>> AverageRotations(
    const std::vector<Eigen::Quaternion<double>>& rotations) {
  return GenericAverageRotations(rotations);
}

absl::StatusOr<Eigen::Vector3<double>> AverageTranslations(
    const std::vector<Eigen::Vector3<double>>& translations) {
  return GenericAverageTranslations(translations);
}

absl::StatusOr<Pose3d> AveragePoses(const std::vector<Pose3d>& poses) {
  if (poses.empty()) {
    // There is no good default value and return we issue an error.
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "No input poses specified.";
  }
  std::vector<Eigen::Quaternion<double>> rotations;
  std::vector<Eigen::Vector3<double>> translations;
  const size_t num_poses = poses.size();
  rotations.reserve(num_poses);
  translations.reserve(num_poses);
  for (const Pose3d& pose : poses) {
    rotations.emplace_back(pose.so3().quaternion());
    translations.push_back(pose.translation());
  }
  INTR_ASSIGN_OR_RETURN(const Eigen::Quaternion<double> average_rotation,
                        AverageRotations(rotations));
  INTR_ASSIGN_OR_RETURN(const Eigen::Vector3<double> average_translation,
                        AverageTranslations(translations));
  return Pose3d{average_rotation, average_translation};
}

absl::StatusOr<Pose3d> AveragePosesWithMeanShift(
    const std::vector<Pose3d>& poses, double rotation_radius,
    double translation_radius) {
  std::vector<Pose3d> meanshift_poses = poses;
  for (auto& meanshift_pose : meanshift_poses) {
    INTR_ASSIGN_OR_RETURN(meanshift_pose,
                          MeanShiftPose(meanshift_pose, poses, rotation_radius,
                                        translation_radius));
  }

  const PoseComparator is_similar_pose(rotation_radius, translation_radius);

  // This container stores for each cluster center, all poses which were
  // assigned to the cluster center.
  std::vector<Cluster> clusters;

  // In this loop, we need to cluster mean-shift poses. Conceptually, the poses
  // in this vector are either close to identical, or they constitute a new
  // cluster center.
  for (size_t i = 0; i < meanshift_poses.size(); ++i) {
    const Pose3d& meanshift_pose = meanshift_poses[i];
    // The rotation and translation radii specify a ball around each cluster
    // center which we use to find a cluster center which contains the current
    // mean-shift pose.
    const auto cluster_iter =
        absl::c_find_if(clusters, [&](const auto& cluster) {
          return is_similar_pose(meanshift_pose, cluster.center_pose);
        });
    const bool found_new_cluster_center = cluster_iter == clusters.end();
    if (found_new_cluster_center) {
      // If we have not found a match for the current mean-shift pose, we create
      // a new cluster-center from the current pose.
      clusters.push_back({meanshift_pose, {meanshift_pose}, {poses[i]}});
    } else {
      // In case we did find a match, we assign the current pose to the set of
      // poses which were matched to this particular cluster center.
      cluster_iter->meanshift_poses.push_back(meanshift_pose);
      cluster_iter->original_poses.push_back(poses[i]);
    }
  }

  const auto largest_cluster_iter =
      absl::c_max_element(clusters, [](const Cluster& lhs, const Cluster& rhs) {
        return lhs.meanshift_poses.size() < lhs.meanshift_poses.size();
      });

  if (largest_cluster_iter == clusters.end()) {
    // This should never happen.
    return intrinsic::InternalErrorBuilder() << "No maximum cluster found.";
  }

  // Return the average of all poses matched to the largest cluster center.
  INTR_ASSIGN_OR_RETURN(const Pose3d average_pose,
                        AveragePoses(largest_cluster_iter->meanshift_poses));

  double max_translation_error = 0.0;
  double avg_translation_error = 0.0;
  double max_rotation_error = 0.0;
  double avg_rotation_error = 0.0;
  for (const Pose3d& pose : largest_cluster_iter->original_poses) {
    const double translation_error_in_meters =
        ComputePoseTranslationError(average_pose, pose);
    const double rotation_error_in_degress =
        ComputePoseRotationError(average_pose, pose);
    max_translation_error =
        std::max(max_translation_error, translation_error_in_meters);
    avg_translation_error += translation_error_in_meters;
    max_rotation_error =
        std::max(max_rotation_error, rotation_error_in_degress);
    avg_rotation_error += rotation_error_in_degress;
  }

  const size_t num_detections = meanshift_poses.size();
  const size_t num_poses = largest_cluster_iter->meanshift_poses.size();

  avg_translation_error /= static_cast<double>(num_poses);
  avg_rotation_error /= static_cast<double>(num_poses);

  LOG(INFO) << "Pose Averaging Statistics:\n"
            << absl::StrFormat(
                   "\tUsed %d / %d poses..................: %f %s\n", num_poses,
                   num_detections,
                   100.0 * num_poses / static_cast<double>(num_detections), "%")
            << absl::StrFormat("\tMax translation error in meters.....: %f\n",
                               max_translation_error)
            << absl::StrFormat("\tMax rotation error in degrees.......: %f\n",
                               Rad2Deg(max_rotation_error))
            << absl::StrFormat("\tAverage translation error in meters.: %f\n",
                               avg_translation_error)
            << absl::StrFormat("\tAverage rotation error in degrees...: %f",
                               Rad2Deg(avg_rotation_error));

  return average_pose;
}

Pose3d GenerateRandomGaussianPoseAroundIdentity(
    double standard_rotation_perturbation_in_radian,
    double standard_translation_perturbation_in_meters) {
  absl::BitGen bitgen;
  const Vector3d translation =
      Eigen::Matrix<double, 3, 1>::Random().normalized() *
      absl::Gaussian(bitgen, 0.0, standard_translation_perturbation_in_meters);
  const AngleAxisd rotation = AngleAxisd(
      absl::Gaussian(bitgen, 0.0, standard_rotation_perturbation_in_radian),
      Eigen::Matrix<double, 3, 1>::Random().normalized());
  return Pose3d(rotation.toRotationMatrix(), translation);
}

Pose3d GenerateRandomUniformPoseAroundIdentity(
    double standard_rotation_perturbation_in_radian,
    double standard_translation_perturbation_in_meters) {
  absl::BitGen bitgen;
  const Vector3d translation =
      Eigen::Matrix<double, 3, 1>::Random().normalized() *
      absl::Uniform(bitgen, -standard_translation_perturbation_in_meters,
                    standard_translation_perturbation_in_meters);
  const AngleAxisd rotation = AngleAxisd(
      absl::Uniform(bitgen, -standard_rotation_perturbation_in_radian,
                    standard_rotation_perturbation_in_radian),
      Eigen::Matrix<double, 3, 1>::Random().normalized());
  return Pose3d(rotation.toRotationMatrix(), translation);
}

Eigen::Vector3d GetYawPitchRollFromRotationMatrix(
    const Eigen::Matrix3d& rotation) {
  Eigen::Vector3d yaw_pitch_roll;
  yaw_pitch_roll(1) = std::asin(std::clamp(rotation(0, 2), -1.0, 1.0));
  if (std::abs(rotation(0, 2)) < 1) {
    yaw_pitch_roll(0) = std::atan2(-rotation(1, 2), rotation(2, 2));
    yaw_pitch_roll(2) = std::atan2(-rotation(0, 1), rotation(0, 0));
  } else {
    yaw_pitch_roll(0) = std::atan2(-rotation(2, 1), rotation(1, 1));
    yaw_pitch_roll(2) = 0.0;
  }
  return yaw_pitch_roll;
}

}  // namespace perception
}  // namespace intrinsic
