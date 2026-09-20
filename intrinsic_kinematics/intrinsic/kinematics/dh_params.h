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

#ifndef INTRINSIC_KINEMATICS_DH_PARAMS_H_
#define INTRINSIC_KINEMATICS_DH_PARAMS_H_

#include <optional>
#include <ostream>

#include "absl/types/optional.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

struct DhParams {
  double theta{0.0};  // Rotation about z-axis.
  double d{0.0};      // Translation along z-axis.
  double a{0.0};      // Translation along x-axis.
  double alpha{0.0};  // Rotation about x-axis.

  DhParams() = default;
  DhParams(double in_theta, double in_d, double in_a, double in_alpha)
      : theta(in_theta), d(in_d), a(in_a), alpha(in_alpha) {}

  friend std::ostream& operator<<(std::ostream& os, const DhParams& dh_params) {
    return os << "theta=" << dh_params.theta << ", d=" << dh_params.d
              << ", a=" << dh_params.a << ", alpha=" << dh_params.alpha;
  }
};

// Computes Denavit-Hartenberg parameters for a joint based upon the previous
// joint's pose and the current joint's axis, specified by "from" and "to"
// points. All arguments are in the robot base coordinate frame.
//
// Takes an optional epsilon argument. Vectors of this length or less are
// considered to be zero, which is used when determining if lines intersect or
// are coincident.
DhParams DhParamsFromBaseSpaceParentPoseAndAxis(
    Pose3d base_t_prev_joint, eigenmath::Vector3d cur_joint_axis_from,
    eigenmath::Vector3d cur_joint_axis_to, double epsilon = 1e-4);

// Converts a DhParams struct into a Pose3d, representing the current joint's
// pose in the previous joint's coordinate frame.
//
// This function is the inverse of DhParamsFromPose().
Pose3d DhParamsToPose(const DhParams& dh);

// Tries to extract DhParams from a Pose3d representing the current joint's pose
// in the previous joint's coordinate frame. Returns an empty value if
// parent_t_joint cannot be decomposed into DH parameters. Takes and optional
// epsilon argument, which is used to compare scalar values to 0.
//
// This function is the inverse of DhParamsToPose().
std::optional<DhParams> DhParamsFromPose(const Pose3d& parent_t_joint,
                                         double epsilon = 1e-4);

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_DH_PARAMS_H_
