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

#include "intrinsic/kinematics/dh_params.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>

#include "intrinsic/eigenmath/common_normal_between_lines.h"
#include "intrinsic/eigenmath/so3.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

using ::intrinsic::Pose3d;
using ::intrinsic::eigenmath::Vector3d;

DhParams DhParamsFromBaseSpaceParentPoseAndAxis(
    const Pose3d base_t_prev_joint,
    const eigenmath::Vector3d cur_joint_axis_from,
    const eigenmath::Vector3d cur_joint_axis_to, double epsilon) {
  const Vector3d& prev_origin = base_t_prev_joint.translation();
  Vector3d prev_x = base_t_prev_joint.xAxis();
  Vector3d prev_z = base_t_prev_joint.zAxis();
  Vector3d cur_z = cur_joint_axis_to - cur_joint_axis_from;
  cur_z.normalize();
  // Calling CommonNormalBetweenLines in this way will set the second point
  // (which we will use as the current origin) to:
  // - Closest point to prev_z on cur_z, if the two lines do not intersect.
  // - Intersection of prev_z and cur_z if they intersect.
  // - prev_origin if prev_z and cur_z are coincident.
  auto common_norm_points = ::intrinsic::eigenmath::CommonNormalBetweenLines(
      prev_origin, prev_z, cur_joint_axis_from, cur_z);
  // cnp_on_prev_z = common normal point on previous z
  const Vector3d& cnp_on_prev_z = common_norm_points.first;
  const Vector3d& cur_origin = common_norm_points.second;

  // d is the translation along prev_z from prev_origin to cnp_on_prev_z.
  DhParams dh_params;
  dh_params.d = prev_z.dot(cnp_on_prev_z - prev_origin);

  // Determine cur_x and a:
  // - If prev_z and cur_z do not intersect (i.e. distance between closest
  //   approach points > 0), vector from cnp_on_prev_z to cur_origin.
  // - If prev_z and cur_z intersect, prev_z cross cur_z.
  // - If prev_z and cur_z are coincident, pick prev_x arbitarily.
  Vector3d cur_x = cur_origin - cnp_on_prev_z;
  double norm = cur_x.norm();
  if (norm >= epsilon) {
    dh_params.a = norm;
    cur_x /= norm;
  } else {
    // prev_z and cur_z either intersect or are coincident, so dh_params.a is 0.
    dh_params.a = 0.0;
    cur_x = prev_z.cross(cur_z);
    norm = cur_x.norm();
    if (norm < epsilon) {
      // prev_z and cur_z are coincident, so use prev_x arbitrarily.
      cur_x = prev_x;
    } else {
      cur_x /= norm;
    }
  }

  // alpha_ is the rotation from prev_z to cur_z about cur_x.
  dh_params.alpha = ::std::acos(::std::clamp(prev_z.dot(cur_z), -1.0, 1.0));
  if (prev_z.cross(cur_z).dot(cur_x) < 0.0) {
    dh_params.alpha *= -1.0;
  }

  // theta_ is the rotation from prev_x to cur_x about prev_z.
  dh_params.theta = ::std::acos(::std::clamp(prev_x.dot(cur_x), -1.0, 1.0));
  if (prev_x.cross(cur_x).dot(prev_z) < 0.0) {
    dh_params.theta *= -1.0;
  }
  return dh_params;
}

Pose3d DhParamsToPose(const DhParams& dh) {
  return CreateAngleAxisPose(dh.theta, Vector3d(0, 0, 1),
                             Vector3d(0, 0, dh.d)) *
         CreateAngleAxisPose(dh.alpha, Vector3d(1, 0, 0), Vector3d(dh.a, 0, 0));
}

std::optional<DhParams> DhParamsFromPose(const Pose3d& parent_t_joint,
                                         double epsilon) {
  // Extract Euler angles and check there's no pitch component.
  DhParams ret;
  double p;
  eigenmath::SO3ToRPY(parent_t_joint.so3(), &ret.alpha, &p, &ret.theta);
  if (std::abs(p) >= epsilon) {
    return std::nullopt;
  }

  // Compute ret.a based upon the x and y translation components, which should
  // be (a * cos(theta)) and (a * sin(theta)) respectively.
  const auto& trans = parent_t_joint.translation();
  double cos_theta = std::cos(ret.theta);
  double sin_theta = std::sin(ret.theta);
  if (std::abs(sin_theta) >= epsilon) {
    ret.a = trans[1] / sin_theta;
    if (std::abs(ret.a * cos_theta - trans[0]) >= epsilon) {
      return std::nullopt;
    }
  } else {
    ret.a = trans[0] / cos_theta;
    if (std::abs(ret.a * sin_theta - trans[1]) >= epsilon) {
      return std::nullopt;
    }
  }

  // ret.d is just the z translation component.
  ret.d = trans[2];
  return ret;
}

}  // namespace kinematics
}  // namespace intrinsic
