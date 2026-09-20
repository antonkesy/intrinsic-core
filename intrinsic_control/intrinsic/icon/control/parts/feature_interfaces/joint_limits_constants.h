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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_LIMITS_CONSTANTS_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_LIMITS_CONSTANTS_H_

namespace intrinsic::icon {

// The "application" velocity, acceleration and jerk limits can at maximum be
// `kMaxApplicationLimitsMultiplier` times the overall limits to allow for
// sufficient headroom for control.
// TODO(b/244134725): Expose the multiplier using a config.
// LINT.IfChange
constexpr double kMaxApplicationLimitsMultiplier = 0.95;
// LINT.ThenChange(
// //intrinsic/frontend/onprem/object_panel/manage_application_limits/application_limits_dialog.ts,
// //intrinsic_sdk/intrinsic/scene/constants.h
// )

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_JOINT_LIMITS_CONSTANTS_H_
