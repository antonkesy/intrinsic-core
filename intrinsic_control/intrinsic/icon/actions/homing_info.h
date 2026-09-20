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

#ifndef INTRINSIC_ICON_ACTIONS_HOMING_INFO_H_
#define INTRINSIC_ICON_ACTIONS_HOMING_INFO_H_

#include "intrinsic/icon/actions/homing_action.pb.h"

namespace intrinsic {
namespace icon {

struct HomingInfo {
  static constexpr char kActionTypeName[] = "intrinsic.homing";
  static constexpr char kActionDescription[] =
      "Commands the homing of a single robot drive.";
  static constexpr char kSlotName[] = "homing";
  static constexpr char kSlotDescription[] =
      "The action performs the homing operation for a single robot drive of "
      "this part.";
  static constexpr char kIsDoneDescription[] =
      "This Action reports 'is_done==true' as soon as the homing is completed.";

  // The action doesn't accept any parameters.
  using FixedParams =
      ::intrinsic_proto::icon::actions::proto::HomingActionFixedParams;
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_ACTIONS_HOMING_INFO_H_
