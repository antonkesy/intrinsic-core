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

#ifndef INTRINSIC_SIMULATION_GAZEBO_WORLD_TEMPLATES_WORLD_TEMPLATE_H_
#define INTRINSIC_SIMULATION_GAZEBO_WORLD_TEMPLATES_WORLD_TEMPLATE_H_

#include <string>

#include "absl/status/statusor.h"

namespace intrinsic {
namespace simulation {

// A wrapper for an SDF world template file.
//
// The template file is expected to be a well-formed XML file with a single
// <world> element. The inner XML of the <world> element can be used to
// create a new world in a Gazebo simulation.
class WorldTemplate {
 public:
  // Loads a world template from a string.
  //
  // Returns an error if the string is not a well-formed XML document, or if
  // it does not contain exactly one <world> element.
  static absl::StatusOr<WorldTemplate> LoadFromString(
      std::string_view contents);

  // Loads a world template from a file.
  //
  // Returns an error if the file cannot be read, if it is not a well-formed
  // XML document, or if it does not contain exactly one <world> element.
  static absl::StatusOr<WorldTemplate> LoadFromFile(std::string_view path);

  // The SDF version specified in the template.
  std::string SdfVersion() const { return sdf_version_; }

  // The inner XML of the <world> element.
  const std::string& WorldXml() const { return world_xml_; }

 private:
  explicit WorldTemplate(std::string sdf_version, std::string world_xml);

  std::string sdf_version_;
  std::string world_xml_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_WORLD_TEMPLATES_WORLD_TEMPLATE_H_
