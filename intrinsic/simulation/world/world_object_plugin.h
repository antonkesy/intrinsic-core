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

#ifndef INTRINSIC_SIMULATION_WORLD_WORLD_OBJECT_PLUGIN_H_
#define INTRINSIC_SIMULATION_WORLD_WORLD_OBJECT_PLUGIN_H_

#include <optional>
#include <string>
#include <string_view>
#include <variant>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"
#include "intrinsic/hardware/gripper/gripper_equipment.pb.h"
#include "intrinsic/hardware/gripper/service_asset/pinch_gripper_opcua_service_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/pinch_gripper_realtime_control_service_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/suction_gripper_opcua_service_config.pb.h"
#include "intrinsic/hardware/gripper/service_asset/suction_gripper_realtime_control_service_config.pb.h"
#include "intrinsic/icon/hal/proto/hardware_module_config.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/proto/v1/camera_config.pb.h"
#include "intrinsic/simulation/world/gripper_plugin_spec.h"
#include "intrinsic/world/proto/sensor_component.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace simulation {

// Helper class to configure simulation plugins from resource instance
// configurations and the Intrinsic World state.
//
// Gripper objects:
// Based on the type of gripper configuration (e.g. PinchGripper,
// SuctionGripper) and the kinematics/topology of the corresponding
// entities in the World, this class determines and populates the matching
// simulation gripper specification (either ActuatedGripperPluginSpec or
// FixedJointGripperPluginSpec).
//
// Camera objects:
// Collects and processes runtime overrides for camera sensors (such as custom
// intrinsics or poses) and maps them to the matching simulation specs, along
// with the serialized camera identifier string.
//
// Hardware modules:
// Registers hardware modules and collects their names, allowing other systems
// to customize simulation setup for them (e.g. skipping default joint plugins
// or fixing their base links in the world).
//
// Gz joint control/state plugins:
// Adds Gz joint control and state plugins in `GetAllPluginSpecs` optionally
// for objects with non-fixed joints that are neither ICON hardware modules nor
// grippers.
class WorldObjectPlugin {
 public:
  struct CameraSpec {
    struct IntrinsicParams {
      intrinsic_proto::world::SensorComponent::Intrinsics intrinsics;
      int32_t image_width = 0;
      int32_t image_height = 0;
      double horizontal_fov = 0.0;
    };
    struct SensorProperties {
      // If `intrinsics` is specified, all values will be set.
      std::optional<IntrinsicParams> intrinsics;
      std::optional<Pose3d> parent_t_sensor;
    };
    std::string camera_identifier_proto;
    // Map from sensor entity local name to overrides.
    absl::flat_hash_map<std::string, SensorProperties>
        sensor_properties_overrides;
  };

  using GripperSpec =
      std::variant<ActuatedGripperPluginSpec, FixedJointGripperPluginSpec>;

  // Data structure to store Gazebo joint control plugin parameters
  struct GzJointControl {
    std::string joint_name;
    uint32_t axis_index{0u};
    std::string topic;
  };

  // Data structure to store Gazebo joint state plugin parameters
  struct GzJointState {
    std::string joint_name;
    uint32_t axis_index{0u};
    std::string topic;
  };

  using GzPlugin = std::variant<GzJointControl, GzJointState>;
  using GzPluginsSpec = std::vector<GzPlugin>;

  // Contains the plugin specifications and annotated objects (such as hardware
  // modules) for the world.
  // The map keys are the unique names of the objects (global alias or a
  // fallback based on the collections entity ID and its local name).
  struct AllPluginSpecs {
    absl::flat_hash_map<std::string, GripperSpec> gripper_specs;
    absl::flat_hash_map<std::string, GzPluginsSpec> gz_plugins_specs;
    absl::flat_hash_set<std::string> hardware_module_objects;
    absl::flat_hash_map<std::string, CameraSpec> camera_specs;
  };

  explicit WorldObjectPlugin(const World& world);

  absl::Status ConfigureHardwareModule(
      const intrinsic_proto::icon::HardwareModuleConfig& config,
      std::string_view object_name);

  absl::Status ConfigureEoatGripper(
      const intrinsic_proto::eoat::GripperConfig& config,
      std::string_view object_name);

  absl::Status ConfigurePinchGripper(
      const intrinsic_proto::gripper::PinchGripperPart& config,
      std::string_view object_name);

  absl::Status ConfigureSuctionGripperRealtimeControl(
      const intrinsic_proto::gripper_service::
          SuctionGripperRealtimeControlServiceConfig& config,
      std::string_view object_name);

  absl::Status ConfigurePinchGripperRealtimeControl(
      const intrinsic_proto::gripper_service::
          PinchGripperRealtimeControlServiceConfig& config,
      std::string_view object_name);

  absl::Status ConfigureSuctionGripperOpcua(
      const intrinsic_proto::gripper_service::SuctionGripperOpcuaServiceConfig&
          config,
      std::string_view object_name);

  absl::Status ConfigurePinchGripperOpcua(
      const intrinsic_proto::gripper_service::PinchGripperOpcuaServiceConfig&
          config,
      std::string_view object_name);

  // Configures the camera or multi-camera spec for the specified world object
  // from a version 1 CameraConfig protobuf.
  //
  // Returns an error if the world object does not have matching camera sensor
  // entities, or if the config has invalid camera properties (e.g. image width
  // or height are non-positive).
  absl::Status ConfigureCamera(
      const intrinsic_proto::perception::v1::CameraConfig& camera_config,
      std::string_view object_name);

  absl::StatusOr<AllPluginSpecs> GetAllPluginSpecs(
      bool add_gz_plugins = false) const;

  const absl::flat_hash_map<std::string, GripperSpec>& GetGripperSpecs() const {
    return gripper_specs_;
  }

  const absl::flat_hash_map<std::string, CameraSpec>& GetCameraSpecs() const {
    return camera_specs_;
  }

 private:
  const World& world_;
  absl::flat_hash_map<std::string, GripperSpec> gripper_specs_;
  absl::flat_hash_set<std::string> hardware_module_objects_;
  absl::flat_hash_map<std::string, CameraSpec> camera_specs_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_WORLD_WORLD_OBJECT_PLUGIN_H_
