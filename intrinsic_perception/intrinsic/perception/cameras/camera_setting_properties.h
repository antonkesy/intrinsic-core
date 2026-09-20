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

#ifndef INTRINSIC_PERCEPTION_CAMERAS_CAMERA_SETTING_PROPERTIES_H_
#define INTRINSIC_PERCEPTION_CAMERAS_CAMERA_SETTING_PROPERTIES_H_

#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <variant>
#include <vector>

namespace intrinsic::perception {

struct CameraSettingProperties {
  struct Float {
    using ValueType = double;
    struct Range {
      ValueType minimum = 0.0;
      ValueType maximum = 0.0;
      auto operator<=>(const Range&) const = default;
    };
    std::optional<Range> range;
    ValueType increment = 0.0;
    std::optional<std::string> unit;
    auto operator<=>(const Float&) const = default;
  };
  struct Integer {
    using ValueType = int64_t;
    struct Range {
      ValueType minimum = 0;
      ValueType maximum = 0;
      auto operator<=>(const Range&) const = default;
    };
    std::optional<Range> range;
    ValueType increment = 0;
    std::optional<std::string> unit;
    auto operator<=>(const Integer&) const = default;
  };
  struct Enumeration {
    std::vector<std::string> values;
    auto operator<=>(const Enumeration&) const = default;
  };
  using Properties = std::variant<std::monostate, Float, Integer, Enumeration>;

  std::string name;
  Properties properties = std::monostate();
  auto operator<=>(const CameraSettingProperties&) const = default;
};

std::ostream& operator<<(std::ostream& os,
                         const CameraSettingProperties& properties);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CAMERAS_CAMERA_SETTING_PROPERTIES_H_
