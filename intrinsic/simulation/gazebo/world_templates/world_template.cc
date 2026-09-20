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

#include "intrinsic/simulation/gazebo/world_templates/world_template.h"

#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/file.h"
#include "tinyxml2.h"

namespace intrinsic {
namespace simulation {
namespace {

std::string InnerXml(const tinyxml2::XMLElement* element) {
  tinyxml2::XMLPrinter printer;
  for (const tinyxml2::XMLNode* node = element->FirstChild(); node != nullptr;
       node = node->NextSibling()) {
    node->Accept(&printer);
  }
  return printer.CStr();
}

}  // namespace

absl::StatusOr<WorldTemplate> WorldTemplate::LoadFromString(
    std::string_view contents) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(contents.data()) != tinyxml2::XML_SUCCESS) {
    return absl::InvalidArgumentError(
        absl::StrCat("Failed to parse XML: ", doc.ErrorStr()));
  }

  const tinyxml2::XMLElement* sdf_element = doc.FirstChildElement("sdf");
  if (!sdf_element) {
    return absl::InvalidArgumentError("Missing <sdf> element.");
  }
  const char* version = sdf_element->Attribute("version");
  if (!version) {
    return absl::InvalidArgumentError(
        "Missing 'version' attribute on <sdf> element.");
  }

  std::vector<const tinyxml2::XMLElement*> world_elements;
  for (const tinyxml2::XMLElement* elem =
           sdf_element->FirstChildElement("world");
       elem != nullptr; elem = elem->NextSiblingElement("world")) {
    world_elements.push_back(elem);
  }

  if (world_elements.empty()) {
    return absl::InvalidArgumentError("No <world> element found.");
  }
  if (world_elements.size() > 1) {
    return absl::InvalidArgumentError("Multiple <world> elements found.");
  }

  return WorldTemplate{version, InnerXml(world_elements[0])};
}

absl::StatusOr<WorldTemplate> WorldTemplate::LoadFromFile(
    std::string_view path) {
  std::string contents;
  INTR_RETURN_IF_ERROR(file::GetContents(path, &contents, file::Defaults()));
  return LoadFromString(contents);
}

WorldTemplate::WorldTemplate(std::string sdf_version, std::string world_xml)
    : sdf_version_(std::move(sdf_version)), world_xml_(std::move(world_xml)) {}

}  // namespace simulation
}  // namespace intrinsic
