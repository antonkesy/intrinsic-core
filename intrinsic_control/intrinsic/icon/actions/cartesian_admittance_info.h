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

#ifndef INTRINSIC_ICON_ACTIONS_CARTESIAN_ADMITTANCE_INFO_H_
#define INTRINSIC_ICON_ACTIONS_CARTESIAN_ADMITTANCE_INFO_H_

#include "intrinsic/icon/actions/cartesian_admittance_action.pb.h"
#include "intrinsic/icon/actions/compliance_info.h"

namespace intrinsic::icon {

struct CartesianAdmittanceInfo : public ComplianceInfo {
  static constexpr char kActionTypeName[] =
      "intrinsic.cartesian_admittance_action";
  static constexpr char kActionDescription[] =
      "Performs Cartesian admittance (compliance) control using an external "
      "force-sensor at the end-effector. Computes joint position commands that "
      "realize a desired (second-order) interaction model with external "
      "forces. The interaction model can be tuned using virtual inertia, "
      "damping and stiffness matrices. Selective compliance control in "
      "arbitrary dimensions can be achieved by providing a custom "
      "selection matrix for the sensed wrench. Uses different motion "
      "generators to create reference impedance trajectories in task space "
      "(and task nullspace). These reference generators support setting speed "
      "overrides, which only affect the feedforward motion (e.g. reference "
      "twist). The CartesianAdmittanceAction requires the 'TareAction' to be "
      "executed immediately before control starts, which resets force-sensor "
      "readings to zero. It also requires the post-sensor inertia to be "
      "configured correctly, which can be identified and set using the "
      "PayloadEstimation skill. The parameter space of this Action is flexible "
      "enough to cover applications ranging from maximum transparency control "
      "in free-space to stiff interaction with environments in assembly tasks.";

  static constexpr char kIsDoneDescription[] =
      "Done when the desired reference pose is reached and both position and "
      "orientation errors are within pre-defined bounds.";

  using FixedParams = ::intrinsic_proto::icon::actions::proto::
      CartesianAdmittanceActionFixedParams;

  using StreamingOutput =
      ::intrinsic_proto::icon::actions::proto::CartesianAdmittanceStatus;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_ACTIONS_CARTESIAN_ADMITTANCE_INFO_H_
