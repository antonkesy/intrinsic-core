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

#include "intrinsic/simulation/world/multi_camera_plugin_spec.h"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "tinyxml2.h"

namespace intrinsic {
namespace simulation {

absl::StatusOr<MultiCameraPluginSpec> MultiCameraPluginSpec::Parse(
    std::string_view xml_spec) {
  tinyxml2::XMLDocument xml_doc;
  if (xml_doc.Parse(xml_spec.data(), xml_spec.size()) !=
      tinyxml2::XML_SUCCESS) {
    return absl::InvalidArgumentError(
        absl::StrCat("Failed to parse plugin XML: ", xml_doc.ErrorStr()));
  }
  const tinyxml2::XMLElement* xml_elem = xml_doc.RootElement();
  if (xml_elem == nullptr) {
    return absl::InvalidArgumentError("Plugin XML contains no root element");
  }

  absl::flat_hash_set<int64_t> seen_sensor_ids;
  MultiCameraPluginSpec result;

  // Parse sensors.
  for (const tinyxml2::XMLElement* sensor_elem =
           xml_elem->FirstChildElement("sensor");
       sensor_elem != nullptr;
       sensor_elem = sensor_elem->NextSiblingElement("sensor")) {
    const tinyxml2::XMLElement* id_elem = sensor_elem->FirstChildElement("id");
    if (id_elem == nullptr) {
      return absl::InvalidArgumentError("Missing id element in sensor");
    }
    if (id_elem->GetText() == nullptr) {
      return absl::InvalidArgumentError("Empty id element in sensor");
    }
    int64_t id = 0;
    if (id_elem->QueryInt64Text(&id) != tinyxml2::XML_SUCCESS) {
      return absl::InvalidArgumentError(
          absl::StrCat("Invalid id element in sensor: ", id_elem->GetText()));
    }
    if (!seen_sensor_ids.insert(id).second) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Duplicate id element (", id, ") found within a sensor element"));
    }
    const tinyxml2::XMLElement* name_elem =
        sensor_elem->FirstChildElement("name");
    if (name_elem == nullptr) {
      return absl::InvalidArgumentError("Missing name element in sensor");
    }
    if (name_elem->GetText() == nullptr) {
      return absl::InvalidArgumentError("Empty name element in sensor");
    }
    MultiCameraPluginSpec::Sensor sensor;
    sensor.id = id;
    sensor.name = name_elem->GetText();
    if (const tinyxml2::XMLElement* image_type_elem =
            sensor_elem->FirstChildElement("image_type");
        image_type_elem != nullptr) {
      if (image_type_elem->GetText() == nullptr) {
        return absl::InvalidArgumentError("Empty image_type element in sensor");
      }
      sensor.image_type = image_type_elem->GetText();
    }
    result.sensors.push_back(std::move(sensor));
  }

  // Optionally parse camera identifier.
  const tinyxml2::XMLElement* camera_identifier_elem =
      xml_elem->FirstChildElement("camera_identifier");
  if (camera_identifier_elem != nullptr) {
    if (camera_identifier_elem->GetText() == nullptr) {
      return absl::InvalidArgumentError("Empty camera_identifier element");
    }
    result.camera_identifier_proto = camera_identifier_elem->GetText();
  }

  return result;
}

std::string MultiCameraPluginSpec::ToString(
    std::string_view plugin_name_suffix) const {
  static constexpr char kPluginXml[] =
      R"xml(<plugin filename="static://intrinsic::simulation::MultiCameraPlugin"/>)xml";
  tinyxml2::XMLDocument xml_doc;
  CHECK_EQ(xml_doc.Parse(kPluginXml), tinyxml2::XML_SUCCESS);
  std::string name = absl::StrCat(kPluginName, plugin_name_suffix);
  xml_doc.RootElement()->SetAttribute("name", name.c_str());

  // Write sensors.
  for (const auto& sensor : sensors) {
    auto* e = xml_doc.NewElement("sensor");
    auto* id = xml_doc.NewElement("id");
    auto* name_elem = xml_doc.NewElement("name");
    id->SetText(sensor.id);
    name_elem->SetText(sensor.name.c_str());
    e->InsertEndChild(id);
    e->InsertEndChild(name_elem);
    if (sensor.image_type.has_value()) {
      auto* image_type = xml_doc.NewElement("image_type");
      image_type->SetText(sensor.image_type->c_str());
      e->InsertEndChild(image_type);
    }
    xml_doc.RootElement()->InsertEndChild(e);
  }

  // Write camera identifier proto.
  if (camera_identifier_proto.has_value() &&
      !camera_identifier_proto->empty()) {
    auto* camera_identifier_elem = xml_doc.NewElement("camera_identifier");
    camera_identifier_elem->SetText(camera_identifier_proto->c_str());
    xml_doc.RootElement()->InsertEndChild(camera_identifier_elem);
  }

  tinyxml2::XMLPrinter xml_printer;
  xml_doc.Print(&xml_printer);
  return xml_printer.CStr();
}

}  // namespace simulation
}  // namespace intrinsic
