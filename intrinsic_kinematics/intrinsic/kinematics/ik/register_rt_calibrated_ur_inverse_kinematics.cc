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

#include "intrinsic/kinematics/ik/inverse_kinematics_factory.h"
#include "intrinsic/kinematics/ik/rt_calibrated_ur_inverse_kinematics.h"

namespace intrinsic::kinematics {
namespace {

const auto kUnused = []() {
  // Register an IK solver for calibrated UR robots.
  // We define and call the lambda immediately because 1) it allows us to get
  // meaningful status reporting if registration fails and 2) we need a global
  // static trivially destructible object (absl:Status is not).
  CHECK_OK(GetGlobalInverseKinematicsFactory().RegisterRealtimeIKSolver(
      "rt_calibrated_ur3e",
      [](Chain chain, const InverseKinematicsInterface::Options& options) {
        return RtCalibratedUrInverseKinematics::Create(
            std::move(chain), UrRobotType::Ur3e, options);
      }));
  CHECK_OK(GetGlobalInverseKinematicsFactory().RegisterRealtimeIKSolver(
      "rt_calibrated_ur5e",
      [](Chain chain, const InverseKinematicsInterface::Options& options) {
        return RtCalibratedUrInverseKinematics::Create(
            std::move(chain), UrRobotType::Ur5e, options);
      }));
  CHECK_OK(GetGlobalInverseKinematicsFactory().RegisterRealtimeIKSolver(
      "rt_calibrated_ur10e",
      [](Chain chain, const InverseKinematicsInterface::Options& options) {
        return RtCalibratedUrInverseKinematics::Create(
            std::move(chain), UrRobotType::Ur10e, options);
      }));
  CHECK_OK(GetGlobalInverseKinematicsFactory().RegisterRealtimeIKSolver(
      "rt_calibrated_ur16e",
      [](Chain chain, const InverseKinematicsInterface::Options& options) {
        return RtCalibratedUrInverseKinematics::Create(
            std::move(chain), UrRobotType::Ur16e, options);
      }));
  CHECK_OK(GetGlobalInverseKinematicsFactory().RegisterRealtimeIKSolver(
      "rt_calibrated_ur20",
      [](Chain chain, const InverseKinematicsInterface::Options& options) {
        return RtCalibratedUrInverseKinematics::Create(
            std::move(chain), UrRobotType::Ur20, options);
      }));
  return true;
}();

}  // namespace
}  // namespace intrinsic::kinematics
