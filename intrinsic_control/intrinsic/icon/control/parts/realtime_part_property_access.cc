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

#include "intrinsic/icon/control/parts/realtime_part_property_access.h"

#include <variant>

#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

RealtimeStatusOr<bool> RealtimePartPropertyAccess::GetBoolProperty(
    PartPropertyId id) const {
  if (id.value() >= non_rt_to_rt_values_.size()) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "Part property with ID ", id.value(), " does not exist."));
  }
  const auto& property_value = non_rt_to_rt_values_.at(id.value());
  if (!std::holds_alternative<bool>(property_value)) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        "Type of part property with ID ", id.value(), " is not boolean."));
  }
  return std::get<bool>(property_value);
}

RealtimeStatusOr<double> RealtimePartPropertyAccess::GetDoubleProperty(
    PartPropertyId id) const {
  if (id.value() >= non_rt_to_rt_values_.size()) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "Part property with ID ", id.value(), " does not exist."));
  }
  const auto& property_value = non_rt_to_rt_values_.at(id.value());
  if (!std::holds_alternative<double>(property_value)) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        "Type of part property with ID ", id.value(), " is not double."));
  }
  return std::get<double>(property_value);
}

RealtimeStatus RealtimePartPropertyAccess::SetBoolProperty(PartPropertyId id,
                                                           bool value) {
  if (id.value() >= rt_to_non_rt_values_.size()) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "Part property with ID ", id.value(), " does not exist."));
  }
  auto& property_value = rt_to_non_rt_values_.at(id.value());
  if (!std::holds_alternative<bool>(property_value)) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        "Type of part property with ID ", id.value(), " is not boolean."));
  }
  property_value = value;
  return OkStatus();
}

RealtimeStatus RealtimePartPropertyAccess::SetDoubleProperty(PartPropertyId id,
                                                             double value) {
  if (id.value() >= rt_to_non_rt_values_.size()) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "Part property with ID ", id.value(), " does not exist."));
  }
  auto& property_value = rt_to_non_rt_values_.at(id.value());
  if (!std::holds_alternative<double>(property_value)) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        "Type of part property with ID ", id.value(), " is not double."));
  }
  property_value = value;
  return OkStatus();
}

}  // namespace intrinsic::icon
