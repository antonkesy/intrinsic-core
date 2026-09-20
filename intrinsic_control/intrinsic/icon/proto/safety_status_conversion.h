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

#ifndef INTRINSIC_ICON_PROTO_SAFETY_STATUS_CONVERSION_H_
#define INTRINSIC_ICON_PROTO_SAFETY_STATUS_CONVERSION_H_

#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/proto/safety_status.pb.h"

namespace intrinsic::icon {

intrinsic_proto::icon::SafetyStatus ToProto(const SafetyStatus& safety_status);

intrinsic_proto::icon::ButtonStatus ToProto(
    const intrinsic_fbs::ButtonStatus& button_status);

intrinsic_proto::icon::ModeOfSafeOperation ToProto(
    const intrinsic_fbs::ModeOfSafeOperation& mode_of_safe_operation);

intrinsic_proto::icon::RequestedBehavior ToProto(
    const intrinsic_fbs::RequestedBehavior& requested_behavior);

SafetyStatus FromProto(
    const intrinsic_proto::icon::SafetyStatus& safety_status);

intrinsic_fbs::ModeOfSafeOperation FromProto(
    const intrinsic_proto::icon::ModeOfSafeOperation& mode_of_safe_operation);

intrinsic_fbs::RequestedBehavior FromProto(
    const intrinsic_proto::icon::RequestedBehavior& requested_behavior);

intrinsic_fbs::ButtonStatus FromProto(
    const intrinsic_proto::icon::ButtonStatus& button_status);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_PROTO_SAFETY_STATUS_CONVERSION_H_
