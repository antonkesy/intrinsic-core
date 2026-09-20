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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_POSE_SAMPLING_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_POSE_SAMPLING_H_

#include "absl/random/random.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::perception {

template <typename URBG = absl::BitGen>
eigenmath::Vector3d GenerateRandomVector3d(double min = 0.0, double max = 1.0,
                                           URBG&& bitgen = URBG()) {
  return eigenmath::Vector3d(absl::Uniform(bitgen, min, max),
                             absl::Uniform(bitgen, min, max),
                             absl::Uniform(bitgen, min, max));
}

struct SampleRandomPoseParams {
  eigenmath::Vector3d sample_box_halfsize;
  double rotation_randomization_angle_radians = 0;
  double rotation_randomization_roll_angle_radians = 0;
};

// Samples random poses for the flange, returns ie. base_t_flange_sampled.
// Parameters:
// sample_box_halfsize is interpreted as aligned with the base
// coordinate frame, and it relates to translation of the attached object.
// Rotation angles are interpreted in the coordinate frame of the attached
// object, after the attached object (camera, calibration pattern) has been
// translated to the sampled location and its viewing direction (Z) has been
// pointed toward the center of the stationary object.
template <typename URBG = absl::BitGen>
Pose3d SampleRandomPose(const Pose3d& base_t_flange,
                        const Pose3d& flange_t_attached_object,
                        const Pose3d& base_t_stationary_object,
                        const SampleRandomPoseParams& params,
                        URBG&& bitgen = URBG()) {
  // Get all transforms to base reference frame for simplicity.
  const Pose3d base_t_attached_object =
      base_t_flange * flange_t_attached_object;

  const eigenmath::Vector3d offset_uniform =
      GenerateRandomVector3d(0, 1, bitgen);
  const eigenmath::Vector3d offset =
      -params.sample_box_halfsize +
      2 * offset_uniform.cwiseProduct(params.sample_box_halfsize);

  // Offset is multiplied from the left, since it is expected to be expressed
  // in relation to the base frame.
  const Pose3d base_t_attached_object_offset =
      Pose3d(offset) * base_t_attached_object;

  // Determine initial new rotation for the attached object.
  // Point the attached object's (either camera or calibration object) z
  // direction to the center of the stationary object, so that the camera faces
  // the calibration object.
  eigenmath::Matrix3d base_t_attached_object_rotation;
  base_t_attached_object_rotation.col(2) =
      (base_t_stationary_object.translation() -
       base_t_attached_object_offset.translation())
          .normalized();
  // By convention the z-axis points in viewing direction or represents 'up' in
  // case of a marker. However, some objects face into z-negative direction
  // which is handled here by flipping z in that case.
  if (base_t_attached_object_rotation.col(2).dot(
          base_t_attached_object_offset.rotationMatrix().col(2)) < 0.0f) {
    base_t_attached_object_rotation.col(2) *= -1;
  }
  base_t_attached_object_rotation.col(0) =
      base_t_attached_object_offset.rotationMatrix()
          .col(1)
          .cross(base_t_attached_object_rotation.col(2))
          .normalized();
  base_t_attached_object_rotation.col(1) =
      base_t_attached_object_rotation.col(2)
          .cross(base_t_attached_object_rotation.col(0))
          .normalized();

  // Apply random tilting around the two axes orthogonal to the viewing
  // direction, and a random rotation around the viewing axis.
  const double rotation_randomization =
      params.rotation_randomization_angle_radians;
  const double tilt_angle_x =
      absl::Uniform(bitgen, -rotation_randomization, rotation_randomization);
  const double tilt_angle_y =
      absl::Uniform(bitgen, -rotation_randomization, rotation_randomization);

  const double rotation_z_max_rad =
      params.rotation_randomization_roll_angle_radians;
  const double tilt_angle_z =
      absl::Uniform(bitgen, -rotation_z_max_rad, rotation_z_max_rad);

  // Multipliciation from the right, since the angle tilts are supposed to act
  // along the axes of the attached object frame.
  // We first apply the sampling rotation around Z axis, then Y and X.
  base_t_attached_object_rotation =
      base_t_attached_object_rotation *
      Eigen::AngleAxisd(tilt_angle_x, Eigen::Vector3d(1, 0, 0)) *
      Eigen::AngleAxisd(tilt_angle_y, Eigen::Vector3d(0, 1, 0)) *
      Eigen::AngleAxisd(tilt_angle_z, Eigen::Vector3d(0, 0, 1));

  // Combine sampled translation and rotation into a single transformation for
  // the attached object.
  const Pose3d base_t_attached_object_sampled =
      Pose3d(base_t_attached_object_rotation,
             base_t_attached_object_offset.translation());

  // Return the transformation w.r.t. the flange reference frame.
  return base_t_attached_object_sampled * flange_t_attached_object.inverse();
}

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CALIBRATION_POSE_SAMPLING_H_
