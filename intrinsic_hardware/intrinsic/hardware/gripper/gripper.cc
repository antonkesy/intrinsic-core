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

#include "intrinsic/hardware/gripper/gripper.h"

#include <memory>
#include <optional>
#include <utility>

#include "absl/base/no_destructor.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::gripper {

absl::StatusOr<std::unique_ptr<PinchGripperInterface>>
PinchGripperFactory::Create(
    absl::string_view pinch_gripper_name,
    const std::optional<google::protobuf::Any>& any_config) {
  auto maybe_planner =
      PinchGripperRegistry::IsRegistered(std::string(pinch_gripper_name));
  if (!maybe_planner) {
    return absl::InvalidArgumentError(
        absl::StrCat("Unregistered pinch gripper class: ", pinch_gripper_name));
  }
  // using 'auto' for easier copybara transformation
  INTR_ASSIGN_OR_RETURN(auto planner,
                        PinchGripperRegistry::Dispatch(
                            std::string(pinch_gripper_name), any_config));
  return std::move(planner);
}

}  // namespace intrinsic::gripper
