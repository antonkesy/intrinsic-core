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

#ifndef INTRINSIC_SIMULATION_WORLD_MULTI_CAMERA_PLUGIN_SPEC_H_
#define INTRINSIC_SIMULATION_WORLD_MULTI_CAMERA_PLUGIN_SPEC_H_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/status/statusor.h"

namespace intrinsic {
namespace simulation {

// Specification for generating and parsing MultiCameraPlugin SDFormat XML.
struct MultiCameraPluginSpec {
  static constexpr char kPluginName[] = "MultiCameraPlugin";

  struct Sensor {
    int64_t id = 0;
    std::string name;
    std::optional<std::string> image_type;

    bool operator==(const Sensor& other) const = default;
  };

  std::vector<Sensor> sensors;
  std::optional<std::string> camera_identifier_proto;

  bool operator==(const MultiCameraPluginSpec& other) const = default;

  // Parses a MultiCameraPlugin XML string into a MultiCameraPluginSpec.
  static absl::StatusOr<MultiCameraPluginSpec> Parse(std::string_view xml_spec);

  // Serializes this spec into an SDFormat XML string.
  // The generated plugin's name attribute will be
  // `<kPluginName><plugin_name_suffix>`.
  std::string ToString(std::string_view plugin_name_suffix = "") const;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_WORLD_MULTI_CAMERA_PLUGIN_SPEC_H_
