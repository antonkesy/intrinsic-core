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
#include "intrinsic/kinematics/ik/wrist_offset_inverse_kinematics.h"

namespace intrinsic::kinematics {
namespace {

const auto kUnused = []() {
  // We define and call the lambda immediately because 1) it allows us to get
  // meaningful status reporting if registration fails and 2) we need a global
  // static trivially destructible object (absl:Status is not).
  // NOTE: While not at all obvious, this solver is used at least in
  // //intrinsic/choreographer:end_to_end_large_cell_smoke_test. There the
  // name "wrist_offset_kinematic" seems to be hardcoded in some CNS world file
  // that is, unfortunately, not code-search-able.
  CHECK_OK(GetGlobalInverseKinematicsFactory().RegisterNonRealtimeIKSolver(
      "wrist_offset_kinematics", &WristOffsetInverseKinematics::Create));
  return true;
}();

}  // namespace
}  // namespace intrinsic::kinematics
