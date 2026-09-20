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

#ifndef INTRINSIC_SIMULATION_WORLD_SIM_PLUGINS_H_
#define INTRINSIC_SIMULATION_WORLD_SIM_PLUGINS_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/hal/proto/v1/digital_input_output.pb.h"
#include "intrinsic/scene/proto/v1/simulation_spec.pb.h"
#include "intrinsic/world/proto/generic_action.pb.h"
#include "intrinsic/world/proto/robot_component.pb.h"
#include "intrinsic/world/proto/sensor_component.pb.h"

namespace intrinsic {
namespace simulation {
/*
 * PluginSdfTrait (through template specialization) gives the matching plugin
 * filename, the Parse() function and the ToString() function for proto
 * messages corresponding to a simulation plugin specification.
 *
 * If the proto message contains the full spec of the plugin then the
 * specialization should follow the template interface below. Otherwise, a
 * different interface can be used. E.g., the IconSimDevice specialization
 * below.
 *
 */
template <class PluginSpecT>
struct PluginSdfTrait {
  static_assert(
      sizeof(PluginSpecT) == 0,
      "PluginSdfTrait must be explicitly instantiated in order to properly "
      "support the plugin spec. Did you forget to implement a plugin trait "
      "for this component type?");
};

// TODO(b/427817454): Remove once all users have removed references from
// .sdf files.
template <>
struct PluginSdfTrait<
    intrinsic_proto::world::SensorComponent::ForceTorqueDevicePluginSpec> {
  static constexpr char kPluginName[] = "ForceTorqueDevicePlugin";
};

template <>
struct PluginSdfTrait<
    intrinsic_proto::world::SensorComponent::CameraPluginSpec> {
  static constexpr char kPluginName[] = "CameraPlugin";
  static absl::StatusOr<
      intrinsic_proto::world::SensorComponent::CameraPluginSpec>
  Parse(absl::string_view xml_spec);
  static std::string ToString(
      const intrinsic_proto::world::SensorComponent::CameraPluginSpec& plugin,
      int index);
};

template <>
struct PluginSdfTrait<
    intrinsic_proto::scene_object::v1::MultiCameraPluginSpec> {
  static constexpr char kPluginName[] = "MultiCameraPlugin";
  static std::string ToString(
      const intrinsic_proto::scene_object::v1::MultiCameraPluginSpec& plugin,
      std::optional<int> index = std::nullopt);
};

template <>
struct PluginSdfTrait<
    intrinsic_proto::world::RobotComponent::IconSimPluginSpec> {
  static constexpr char kPluginName[] = "TimeslicerWorld";
  static absl::StatusOr<
      intrinsic_proto::world::RobotComponent::IconSimPluginSpec>
  Parse(absl::string_view xml_spec);
  static std::string ToString(
      const intrinsic_proto::world::RobotComponent::IconSimPluginSpec& plugin,
      int index);
};

template <>
struct PluginSdfTrait<intrinsic_proto::world::RobotComponent::IconSimDevice> {
  static constexpr char kPluginName[] = "DeviceContainerPlugin";
  // The ICON sim devices parsed from the xml_spec is appended to devices.
  static absl::Status Parse(
      absl::string_view xml_spec,
      std::vector<intrinsic_proto::world::RobotComponent::IconSimDevice>*
          devices);
  // This method is meant to be used for tests only.
  static absl::StatusOr<std::string> GetOutputTopic(
      absl::string_view xml_spec, absl::string_view icon_sim_device_name);
  // This method is meant to be used for tests only.
  static absl::StatusOr<std::string> GetInputTopic(
      absl::string_view xml_spec, absl::string_view icon_sim_device_name);
  static absl::StatusOr<std::string> ToString(
      absl::Span<const intrinsic_proto::world::RobotComponent::IconSimDevice>
          devices,
      absl::flat_hash_map<std::string, std::string> device_name_to_output_topic,
      absl::flat_hash_map<std::string, std::string> device_name_to_input_topic,
      int index);
};

template <>
struct PluginSdfTrait<intrinsic_proto::icon::v1::DigitalInputOutput> {
  static constexpr char kPluginName[] = "DigitalInputOutput";
  // Parses `xml_spec` into `digital_input_output` (see
  // intrinsic/simulation/gazebo/plugins/digital_input_output.h for
  // the XML tag definition)
  //
  // Returns AlreadyExistsError if there are input or output blocks in
  // `xml_spec` with names that are already in `digital_input_output`.
  static absl::Status Parse(
      absl::string_view xml_spec,
      intrinsic_proto::icon::v1::DigitalInputOutput* digital_input_output);
  static absl::StatusOr<std::string> ToString(
      const intrinsic_proto::icon::v1::DigitalInputOutput& digital_input_output,
      const absl::flat_hash_map<std::string, std::string>&
          block_name_to_output_topic,
      const absl::flat_hash_map<std::string, std::string>&
          block_name_to_input_topic);
};

// TODO(b/427817454): Remove once all users have removed references from
// .sdf files.
template <>
struct PluginSdfTrait<
    intrinsic_proto::world::SensorComponent::RangeFinderDevicePluginSpec> {
  static constexpr char kPluginName[] = "RangeFinderDevicePlugin";
};

template <>
struct PluginSdfTrait<
    intrinsic_proto::world::generic_action::GenericActionPluginSpec> {
  static constexpr char kPluginName[] = "GenericActionPlugin";
  static absl::StatusOr<
      intrinsic_proto::world::generic_action::GenericActionPluginSpec>
  Parse(absl::string_view xml_spec);
  static std::string ToString(
      const intrinsic_proto::world::generic_action::GenericActionPluginSpec&
          plugin);
};

// Check if the sdf plugin filename matches the given spec.
template <typename PluginSpecT>
bool FilenameMatchesPluginSpec(absl::string_view filename) {
  absl::string_view plugin_name;
  if constexpr (requires { PluginSpecT::kPluginName; }) {
    plugin_name = PluginSpecT::kPluginName;
  } else {
    plugin_name = PluginSdfTrait<PluginSpecT>::kPluginName;
  }
  return filename == absl::StrCat("static://giza::simulation::", plugin_name) ||
         filename ==
             absl::StrCat("static://intrinsic::simulation::", plugin_name);
}

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_WORLD_SIM_PLUGINS_H_
