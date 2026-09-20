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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_FORCE_PRIMITIVE_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_FORCE_PRIMITIVE_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/icon/actions/force_primitive_info.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"

namespace intrinsic::icon::force_primitive {

// Creates a force primitive action.
absl::StatusOr<std::unique_ptr<RtclActionInterface>> Create(
    const ForcePrimitiveInfo::FixedParams& params_proto,
    ActionFactoryContext& context) INTRINSIC_NON_REALTIME_ONLY;

}  // namespace intrinsic::icon::force_primitive

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_FORCE_PRIMITIVE_H_
