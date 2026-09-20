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

#include "intrinsic/simulation/world/inlined_plugins_util.h"

#include <optional>
#include <string>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/wrappers.pb.h"
#include "intrinsic/icon/hal/proto/v1/digital_input_output.pb.h"
#include "intrinsic/scene/constants.h"
#include "intrinsic/simulation/world/multi_camera_plugin_spec.h"
#include "intrinsic/simulation/world/sim_plugins.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/user_data_component.h"
#include "intrinsic/world/entity.h"
#include "tinyxml2.h"

namespace intrinsic {
namespace simulation {

absl::StatusOr<InlinedPluginsInfo> ParseInlinedPlugins(
    const WorldEntity* entity) {
  InlinedPluginsInfo result;
  if (entity == nullptr) {
    return result;
  }
  auto user_data_comp = entity->GetComponent<UserDataComponent>();
  if (!user_data_comp.ok()) {
    return result;
  }
  std::string gazebo_plugins_str;
  const auto& user_data_protos = (*user_data_comp)->UserDataProtos();
  if (auto proto_itr = user_data_protos.find(::intrinsic::sdf::kGazeboPlugins);
      proto_itr != user_data_protos.end()) {
    google::protobuf::StringValue str_val;
    if (proto_itr->second.UnpackTo(&str_val)) {
      gazebo_plugins_str = str_val.value();
    }
  }
  const auto& user_data_map = (*user_data_comp)->UserDataMap();
  if (auto map_itr = user_data_map.find(::intrinsic::sdf::kGazeboPlugins);
      map_itr != user_data_map.end()) {
    if (!gazebo_plugins_str.empty()) {
      absl::StrAppend(&gazebo_plugins_str, "\n", map_itr->second);
    } else {
      gazebo_plugins_str = map_itr->second;
    }
  }

  if (gazebo_plugins_str.empty()) {
    return result;
  }

  // Wraps in a dummy root to handle multiple plugins.
  std::string wrapped_xml =
      absl::StrCat("<plugins>", gazebo_plugins_str, "</plugins>");
  tinyxml2::XMLDocument doc;
  if (doc.Parse(wrapped_xml.c_str()) != tinyxml2::XML_SUCCESS) {
    return absl::InvalidArgumentError(
        absl::StrCat("Failed to parse gazebo_plugins for entity ",
                     entity->GetAlias(), ": ", doc.ErrorStr()));
  }

  tinyxml2::XMLElement* root = doc.RootElement();
  std::vector<tinyxml2::XMLElement*> elements_to_delete;

  for (tinyxml2::XMLElement* elem = root->FirstChildElement("plugin");
       elem != nullptr; elem = elem->NextSiblingElement("plugin")) {
    const char* filename = elem->Attribute("filename");
    if (filename == nullptr) {
      LOG(WARNING) << "Malformed inlined plugin data in world for entity "
                   << entity->GetLocalName();
      continue;
    }
    if (FilenameMatchesPluginSpec<
            intrinsic_proto::icon::v1::DigitalInputOutput>(filename)) {
      tinyxml2::XMLPrinter printer;
      elem->Accept(&printer);
      std::string dio_xml(printer.CStr());
      if (!result.dios_proto.has_value()) {
        result.dios_proto = intrinsic_proto::icon::v1::DigitalInputOutput();
      }
      INTR_RETURN_IF_ERROR(
          PluginSdfTrait<intrinsic_proto::icon::v1::DigitalInputOutput>::Parse(
              dio_xml, &result.dios_proto.value()));
      elements_to_delete.push_back(elem);
    } else if (FilenameMatchesPluginSpec<MultiCameraPluginSpec>(filename)) {
      tinyxml2::XMLPrinter printer;
      elem->Accept(&printer);
      std::string multi_cam_xml(printer.CStr());
      INTR_ASSIGN_OR_RETURN(result.multi_camera_plugin_spec,
                            MultiCameraPluginSpec::Parse(multi_cam_xml));
      elements_to_delete.push_back(elem);
    }
  }

  if (elements_to_delete.empty()) {
    result.unparsed_inlined_plugins = gazebo_plugins_str;
    return result;
  }

  for (auto* elem : elements_to_delete) {
    root->DeleteChild(elem);
  }

  // Serializes the remaining plugins back.
  tinyxml2::XMLPrinter remaining_printer;
  for (const tinyxml2::XMLNode* node = root->FirstChild(); node != nullptr;
       node = node->NextSibling()) {
    node->Accept(&remaining_printer);
  }
  result.unparsed_inlined_plugins = remaining_printer.CStr();

  return result;
}

}  // namespace simulation
}  // namespace intrinsic
