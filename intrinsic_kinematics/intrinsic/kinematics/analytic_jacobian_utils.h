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

#ifndef INTRINSIC_KINEMATICS_ANALYTIC_JACOBIAN_UTILS_H_
#define INTRINSIC_KINEMATICS_ANALYTIC_JACOBIAN_UTILS_H_

#include "intrinsic/eigenmath/so3.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {
namespace kinematics {

// A helper function to compute the mapper from an Analytic Jacobian Matrix to a
// Geometric Jacobian Matrix, as a pre-multiplication matrix.
// This function returns a 6x6 mapper matrix which converts a 6xn_dof
// Analytic Jacobian into the 6xn_dof Geometric Jacobian as in:
// geometric_jacobian = mapper * analytic_jacobian;
// Implementation of Equations (2.106) on page 25, (2.191) on page 41, and
// (2.192) on page 41 from "Robot Dynamics Lecture Notes" by You, Jan 6, 2017
// (https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
// , accessed on May 3, 2022, or
// https://drive.google.com/file/d/1JpENZsBUoOG4rTxKtPL1T1L0hdQznYco/view?usp=sharing&resourcekey=0-bzeEF14JZAL6SGrDT5pRMQ).
eigenmath::Matrix6d ComputeAnalyticToGeometricJacobianMapper(
    const eigenmath::SO3d& base_R_target);

// A helper function to compute the mapper from a Geometric Jacobian Matrix to
// an Analytic Jacobian Matrix, as a pre-multiplication matrix,
// This function returns a 6x6 mapper matrix which converts a 6xn_dof Geometric
// Jacobian into the 6xn_dof Analytic Jacobian as in:
// analytic_jacobian = mapper * geometric_jacobian;
// Implementation of Equations (2.107) on page 25, (2.191) on page 41, and
// (2.192) on page 41 from "Robot Dynamics Lecture Notes" by You, Jan 6, 2017
// (https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
// , accessed on May 3, 2022, or
// https://drive.google.com/file/d/1JpENZsBUoOG4rTxKtPL1T1L0hdQznYco/view?usp=sharing&resourcekey=0-bzeEF14JZAL6SGrDT5pRMQ).
eigenmath::Matrix6d ComputeGeometricToAnalyticJacobianMapper(
    const eigenmath::SO3d& base_R_target);

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_ANALYTIC_JACOBIAN_UTILS_H_
