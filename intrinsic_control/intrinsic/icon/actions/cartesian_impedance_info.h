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

#ifndef INTRINSIC_ICON_ACTIONS_CARTESIAN_IMPEDANCE_INFO_H_
#define INTRINSIC_ICON_ACTIONS_CARTESIAN_IMPEDANCE_INFO_H_

#include "intrinsic/icon/actions/cartesian_admittance_action.pb.h"
#include "intrinsic/icon/actions/compliance_info.h"

namespace intrinsic::icon {

struct CartesianImpedanceInfo : public ComplianceInfo {
  static constexpr char kActionTypeName[] =
      "intrinsic.cartesian_impedance_action";
  static constexpr char kActionDescription[] =
      "Performs task-space impedance control while acting on a joint torques "
      "as control input. Uses a motion generator to create a reference "
      "impedance trajectory to be tracked. Currently we support a 'simple' "
      "first order low-pass filter motion generator, Cartesian position "
      "Reflexxes, which generates a time-optimal trajectory, Cartesian "
      "velocity Reflexxes, and a quintic spline reference generator.";

  static constexpr char kIsDoneDescription[] =
      "Done when the desired pose is achieved and both position and "
      "orientation error are within pre-defined bounds.";

  using FixedParams = ::intrinsic_proto::icon::actions::proto::
      CartesianAdmittanceActionFixedParams;

  using StreamingOutput =
      ::intrinsic_proto::icon::actions::proto::CartesianAdmittanceStatus;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_ACTIONS_CARTESIAN_IMPEDANCE_INFO_H_
