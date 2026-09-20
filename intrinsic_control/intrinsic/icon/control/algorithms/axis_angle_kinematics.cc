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

#include "intrinsic/icon/control/algorithms/axis_angle_kinematics.h"

#include <cmath>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {
namespace control {

constexpr double kEpsilon = 1e-5;

// utility function to compute "spin matrix" coresponding to cross product.
eigenmath::Matrix3d tilde(const eigenmath::Vector3d& v) {
  eigenmath::Matrix3d result;
  result(0, 0) = 0.0;
  result(0, 1) = -v[2];
  result(0, 2) = v[1];
  result(1, 0) = v[2];
  result(1, 1) = 0.0;
  result(1, 2) = -v[0];
  result(2, 0) = -v[1];
  result(2, 1) = v[0];
  result(2, 2) = 0.0;
  return result;
}

bool AxisAngleKinematics::CloseToUnhandledSingularity(
    const eigenmath::Vector3d& axis_angle) {
  const double angle = axis_angle.norm();
  if ((angle > kEpsilon) && (std::fabs(std::fmod(angle, M_PI)) < kEpsilon)) {
    return true;
  }
  return false;
}

void AxisAngleKinematics::UpdateVelocity() {
  sina_ = std::sin(angle_);
  cosa_ = std::cos(angle_);

  u_ = axis_angle_ / angle_;

  uut_ = u_ * u_.transpose();
  one_minus_uut_ = eigenmath::Matrix3d::Identity() - uut_;
  tilde_u_ = tilde(u_);

  R_ = Eigen::AngleAxisd(angle_, u_).toRotationMatrix();
}

void AxisAngleKinematics::UpdateAcceleration() {
  dot_angle_ = dot_axis_angle_.dot(u_);

  dot_u_ = (dot_axis_angle_ - u_ * dot_angle_) / angle_;
  udot_ut_dot_uut_ = u_ * dot_u_.transpose() + dot_u_ * u_.transpose();
  dot_tilde_u_ = tilde(dot_u_);
}

void AxisAngleKinematics::CalcMap() {
  map_ = uut_ + sina_ / angle_ * one_minus_uut_ -
         (1.0 - cosa_) / angle_ * tilde_u_ * one_minus_uut_;
}

void AxisAngleKinematics::CalcDotMap() {
  dot_map_ =
      udot_ut_dot_uut_ +
      ((cosa_ * dot_angle_ / angle_ - sina_ * dot_angle_ / pow(angle_, 2)) *
           eigenmath::Matrix3d::Identity() -
       (sina_ * dot_angle_ / angle_ -
        (1.0 - cosa_) * dot_angle_ / pow(angle_, 2)) *
           tilde_u_ -
       (1.0 - cosa_) / angle_ * dot_tilde_u_) *
          one_minus_uut_ -
      (eigenmath::Matrix3d::Identity() * sina_ / angle_ -
       (1.0 - cosa_) / angle_ * tilde_u_) *
          udot_ut_dot_uut_;
}

void AxisAngleKinematics::ToAngularVelocityAcceleration(
    const eigenmath::Vector3d& axis_angle,
    const eigenmath::Vector3d& dot_axis_angle,
    const eigenmath::Vector3d& ddot_axis_angle, eigenmath::Vector3d* omega,
    eigenmath::Vector3d* dot_omega) {
  CHECK(omega != nullptr);
  CHECK(dot_omega != nullptr);

  axis_angle_ = axis_angle;
  dot_axis_angle_ = dot_axis_angle;
  ddot_axis_angle_ = ddot_axis_angle;

  angle_ = axis_angle_.norm();
  if (angle_ > kEpsilon) {
    UpdateVelocity();
    CalcMap();
    const eigenmath::Vector3d omega_b = map_ * dot_axis_angle_;
    *omega = R_ * omega_b;

    UpdateAcceleration();
    CalcDotMap();
    const eigenmath::Vector3d dot_omega_b =
        map_ * ddot_axis_angle_ + dot_map_ * dot_axis_angle_;
    *dot_omega = R_ * dot_omega_b;
  } else {
    // close to zero angle. Transform dot_axis_angle -> omega_b approaches unit
    // matrix (as does R)
    *omega = dot_axis_angle_;
    // limit(dot_map) -> 0
    *dot_omega = ddot_axis_angle_;
  }
}

void AxisAngleKinematics::ToAxisAngleDerivatives(
    const eigenmath::Vector3d& axis_angle, const eigenmath::Vector3d& omega,
    const eigenmath::Vector3d& dot_omega, eigenmath::Vector3d* dot_axis_angle,
    eigenmath::Vector3d* ddot_axis_angle) {
  CHECK(dot_axis_angle != nullptr);
  CHECK(ddot_axis_angle != nullptr);

  axis_angle_ = axis_angle;

  // caveat: this has singularities. possible solution would be to add fixed
  // rotation offset in planning method, so we never call this close to
  // singularities.
  angle_ = axis_angle_.norm();
  if (std::fabs(std::fmod(angle_, M_PI)) > kEpsilon) {
    UpdateVelocity();
    CalcMap();
    const eigenmath::Vector3d omega_b = R_.transpose() * omega;
    dot_axis_angle_ = map_.partialPivLu().solve(omega_b);

    const eigenmath::Vector3d dot_omega_b = R_.transpose() * dot_omega;
    UpdateAcceleration();
    CalcDotMap();
    ddot_axis_angle_ =
        map_.partialPivLu().solve(dot_omega_b - dot_map_ * dot_axis_angle_);

  } else {
    if (angle_ <= kEpsilon) {
      // close to zero angle: limit(map) -> Identity
      dot_axis_angle_ = omega;
      // limit(dot_map) -> 0
      ddot_axis_angle_ = dot_omega;
    } else {
      LOG(FATAL) << "Unhandled singular case.";
    }
  }
  *dot_axis_angle = dot_axis_angle_;
  *ddot_axis_angle = ddot_axis_angle_;
}

void AxisAngleKinematics::ToAxisAngleDerivative(
    const eigenmath::Vector3d& axis_angle, const eigenmath::Vector3d& omega,
    eigenmath::Vector3d* dot_axis_angle) {
  CHECK(dot_axis_angle != nullptr);

  axis_angle_ = axis_angle;

  // caveat: this has singularities. possible solution would be to add fixed
  // rotation offset
  //         in planning method, so we never call this close to singularities.
  angle_ = axis_angle_.norm();
  if (std::fabs(std::fmod(angle_, M_PI)) > kEpsilon) {
    UpdateVelocity();
    CalcMap();
    const eigenmath::Vector3d omega_b = R_.transpose() * omega;
    dot_axis_angle_ = map_.partialPivLu().solve(omega_b);
  } else {
    if (angle_ <= kEpsilon) {
      // close to zero angle: limit(map) -> Identity
      dot_axis_angle_ = omega;
    } else {
      LOG(FATAL) << "Unhandled singular case.";
    }
  }
  *dot_axis_angle = dot_axis_angle_;
}
}  // namespace control
}  // namespace intrinsic
