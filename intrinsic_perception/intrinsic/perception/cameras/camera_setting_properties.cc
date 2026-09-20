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

#include "intrinsic/perception/cameras/camera_setting_properties.h"

#include <ostream>
#include <string_view>
#include <variant>

#include "absl/functional/overload.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

namespace intrinsic::perception {

std::ostream& operator<<(std::ostream& os,
                         const CameraSettingProperties& properties) {
  const std::string_view name = properties.name;
  std::visit(
      absl::Overload{
          [&](const std::monostate&) { os << "Not set: " << name; },
          [&](const CameraSettingProperties::Integer& properties) {
            os << "Integer: " << name << " = (Range = "
               << (properties.range.has_value()
                       ? absl::StrCat("(", properties.range->minimum, ",",
                                      properties.range->maximum, ")")
                       : "Not set")
               << ", Increment = " << properties.increment
               << ", Unit = " << properties.unit.value_or("Not set") << ")";
          },
          [&](const CameraSettingProperties::Float& properties) {
            os << "Float: " << name << " = (Range = "
               << (properties.range.has_value()
                       ? absl::StrCat("(", properties.range->minimum, ",",
                                      properties.range->maximum, ")")
                       : "Not set")
               << ", Increment = " << properties.increment
               << ", Unit = " << properties.unit.value_or("Not set") << ")";
          },
          [&](const CameraSettingProperties::Enumeration& properties) {
            os << "Enumeration: " << name << " = ("
               << absl::StrJoin(properties.values, ",") << ")";
          },
      },
      properties.properties);
  return os;
}

}  // namespace intrinsic::perception
