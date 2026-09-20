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

#include "intrinsic/icon/control/actions/force_primitive.h"

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/icon/actions/cartesian_admittance_info.h"
#include "intrinsic/icon/actions/cartesian_impedance.pb.h"
#include "intrinsic/icon/actions/cartesian_impedance_info.h"
#include "intrinsic/icon/actions/force_primitive_info.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/actions/cartesian_admittance_action.h"
#include "intrinsic/icon/control/actions/cartesian_impedance_action.h"
#include "intrinsic/icon/control/primitives/force_control/action_builder.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon::force_primitive {

absl::StatusOr<std::unique_ptr<RtclActionInterface>> Create(
    const ForcePrimitiveInfo::FixedParams& params_proto,
    ActionFactoryContext& context) {
  INTR_ASSIGN_OR_RETURN(CartesianAdmittanceInfo::FixedParams action_params,
                        force_primitive::BuildParams(params_proto));

  if (params_proto.force_control_settings().use_impedance_control()) {
    CartesianImpedanceInfo::FixedParams impedance_params;
    *impedance_params.mutable_cartesian_impedance_parameters() =
        action_params.cartesian_impedance_parameters();
    return CartesianImpedanceAction::Create(impedance_params, context);
  }

  return CartesianAdmittanceAction::Create(action_params, context);
}

}  // namespace intrinsic::icon::force_primitive
