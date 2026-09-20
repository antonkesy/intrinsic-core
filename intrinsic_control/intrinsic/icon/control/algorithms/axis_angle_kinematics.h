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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_AXIS_ANGLE_KINEMATICS_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_AXIS_ANGLE_KINEMATICS_H_

#include "intrinsic/eigenmath/types.h"

namespace intrinsic {
namespace control {

/// Utility class to convert to angular velocity and angular acceleration fro
/// axis*angle, and back again.
class AxisAngleKinematics {
 public:
  // Computes angular velocity and acceleration from axis*angle and its
  // derivative
  // axis_angle: axis_angle axis*angle
  // dot_axis_angle: time derivative of axis_angle
  // ddot_axis_angle: second time derivative of axis_angle
  // omega: angular velocity
  // dot_omega: angular acceleration
  void ToAngularVelocityAcceleration(const eigenmath::Vector3d& axis_angle,
                                     const eigenmath::Vector3d& dot_axis_angle,
                                     const eigenmath::Vector3d& ddot_axis_angle,
                                     eigenmath::Vector3d* omega,
                                     eigenmath::Vector3d* dot_omega);

  // Computes derivatives of axis*angle from axis and angular velocity
  // axis_angle: axis_angle axis*angle
  // dot_axis_angle: time derivative of axis_angle
  // ddot_axis_angle: second time derivative of axis_angle
  // omega: angular velocity
  // dot_omega: angular acceleration
  void ToAxisAngleDerivatives(const eigenmath::Vector3d& axis_angle,
                              const eigenmath::Vector3d& omega,
                              const eigenmath::Vector3d& dot_omega,
                              eigenmath::Vector3d* dot_axis_angle,
                              eigenmath::Vector3d* ddot_axis_angle);
  // Computes derivative of axis*angle from axis and angular velocity
  // axis_angle: axis*angle
  // omega angular: velocity
  // dot_axis_angle: time derivative of axis_angle
  void ToAxisAngleDerivative(const eigenmath::Vector3d& axis_angle,
                             const eigenmath::Vector3d& omega,
                             eigenmath::Vector3d* dot_axis_angle);
  // true if close to unhandled singularity
  // axis_angle: axis*angle vector
  bool CloseToUnhandledSingularity(const eigenmath::Vector3d& axis_angle);

 private:
  void UpdateVelocity();
  void UpdateAcceleration();

  void CalcMap();
  void CalcDotMap();

  // Scratch variables. Use compact names to avoid extremely long code for the
  // equations.
  // rotation angle
  double angle_ = 0.0;
  // sin(angle_)
  double sina_ = 0.0;
  // cos(angle_)
  double cosa_ = 1.0;
  // axis*angle
  eigenmath::Vector3d axis_angle_ = eigenmath::Vector3d::Zero();
  // d(axis_angle)/dt
  eigenmath::Vector3d dot_axis_angle_ = eigenmath::Vector3d::Zero();
  // d^2(axis_angle)/dt^2
  eigenmath::Vector3d ddot_axis_angle_ = eigenmath::Vector3d::Zero();
  // rotation axis
  eigenmath::Vector3d u_ = eigenmath::Vector3d::Zero();
  // rotation matrix
  eigenmath::Matrix3d R_ = eigenmath::Matrix3d::Zero();
  // u*u^T
  eigenmath::Matrix3d uut_ = eigenmath::Matrix3d::Zero();
  // I-u*u^T (I: identity matrix)
  eigenmath::Matrix3d one_minus_uut_ = eigenmath::Matrix3d::Zero();
  // spin (cross product) matrix for rotation matrix
  eigenmath::Matrix3d tilde_u_ = eigenmath::Matrix3d::Zero();
  // matrix that maps axis-angle derivative to angular velocity
  eigenmath::Matrix3d map_ = eigenmath::Matrix3d::Zero();
  // d(angle)/dt
  double dot_angle_ = 0;
  // d(u)/dt
  eigenmath::Vector3d dot_u_ = eigenmath::Vector3d::Zero();
  // u*(d(u)/dt)^T  + d(u)/dt*u^T
  eigenmath::Matrix3d udot_ut_dot_uut_ = eigenmath::Matrix3d::Zero();
  // spin matrix for d(u)/dt
  eigenmath::Matrix3d dot_tilde_u_ = eigenmath::Matrix3d::Zero();
  // d(map)/dt
  eigenmath::Matrix3d dot_map_ = eigenmath::Matrix3d::Zero();
};

}  // namespace control
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_AXIS_ANGLE_KINEMATICS_H_
