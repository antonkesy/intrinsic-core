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

#include "intrinsic/perception/cameras/camera_setting.h"

#include <ostream>
#include <string_view>
#include <variant>

#include "absl/functional/overload.h"

namespace intrinsic::perception {

std::ostream& operator<<(std::ostream& os, const CameraSetting& setting) {
  const std::string_view name = setting.name;
  std::visit(absl::Overload{
                 [&](const std::monostate&) { os << "Not set: " << name; },
                 [&](const CameraSetting::Integer& value) {
                   os << "Integer: " << name << " = " << value.value;
                 },
                 [&](const CameraSetting::Float& value) {
                   os << "Float: " << name << " = " << value.value;
                 },
                 [&](const CameraSetting::Boolean& value) {
                   os << "Boolean: " << name << " = " << value.value;
                 },
                 [&](const CameraSetting::String& value) {
                   os << "String: " << name << " = " << value.value;
                 },
                 [&](const CameraSetting::Enumeration& value) {
                   os << "Enumeration: " << name << " = " << value.value;
                 },
                 [&](const CameraSetting::Command& value) {
                   os << "Command: " << name;
                 },
             },
             setting.value);
  return os;
}

}  // namespace intrinsic::perception
