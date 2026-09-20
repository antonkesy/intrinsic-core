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

#ifndef INTRINSIC_ICON_ACTIONS_STOP_INFO_H_
#define INTRINSIC_ICON_ACTIONS_STOP_INFO_H_

namespace intrinsic {
namespace icon {

struct StopInfo {
  static constexpr char kIsSettled[] = "is_settled";
  static constexpr char kIsSettledDescription[] =
      "This Action reports 'is_settled==true' as soon as residual oscillations "
      "and/or tracking errors have decayed and the robot has reached a settled "
      "state after the stop motion.";
  static constexpr char kIsSettledUncertainty[] = "is_settled_uncertainty";
  static constexpr char kIsSettledUncertaintyDescription[] =
      "Reports the uncertainty in the belief if the robot has settled or not "
      "as a continuous measure in the range [0,1]. 1 means maximum uncertainty "
      "(robot is not settled), and 0 minimum uncertainty (robot has settled). ";
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_ACTIONS_STOP_INFO_H_
