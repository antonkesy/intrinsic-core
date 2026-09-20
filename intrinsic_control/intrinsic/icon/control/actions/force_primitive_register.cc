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

#include "intrinsic/icon/actions/force_primitive_info.h"
#include "intrinsic/icon/control/actions/cartesian_admittance_action.h"
#include "intrinsic/icon/control/actions/force_primitive.h"
#include "intrinsic/icon/control/rtcl_action_factory_registry.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic::icon {
namespace {

using ::intrinsic_proto::icon::v1::ActionSignature;

ActionSignature GetSignature() {
  ActionSignature signature = CartesianAdmittanceAction::GetSignature();
  signature.set_action_type_name(ForcePrimitiveInfo::kActionTypeName);
  signature.set_text_description(ForcePrimitiveInfo::kActionDescription);
  return signature;
}

const auto kUnused = GetGlobalRtclActionFactoryRegistry()
                         .RegisterTyped<ForcePrimitiveInfo::FixedParams>(
                             ForcePrimitiveInfo::kActionTypeName,
                             &force_primitive::Create, GetSignature());

}  // namespace
}  // namespace intrinsic::icon
