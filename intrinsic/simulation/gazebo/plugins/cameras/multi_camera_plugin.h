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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_MULTI_CAMERA_PLUGIN_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_MULTI_CAMERA_PLUGIN_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/container/node_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "gz/msgs/MessageTypes.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "gz/transport/MessageInfo.hh"
#include "gz/transport/Node.hh"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/cameras/sensor_image.h"
#include "intrinsic/perception/core/pixel_type.h"
#include "intrinsic/simulation/gazebo/plugins/cameras/camera_service.h"
#include "intrinsic/simulation/gazebo/plugins/cameras/triggered_camera_client.h"
#include "sdf/Element.hh"
#include "sdf/Sensor.hh"

namespace intrinsic {
namespace simulation {

struct CameraSensorProperty {
  int64_t id;
  uint64_t entity;
  std::string name;
  sdf::Sensor sdf;
  perception::PixelType image_type = perception::PixelType::kIntensity;
};

// A plugin for serving multiple camera images from the Gazebo instance to
// the simulation camera server.
// The plugin initializes the camera server if it is not running already.
// The plugin must be included in the model SDF as follows:
// <model name="model_name">
//   ...
//   <link name="link_name">
//     ...
//     <sensor name="sensor1_name" type="rgbd_camera">
//       <topic>/some/rgbd/topic</topic>
//       ...
//       <update_rate>...</update_rate>
//       <camera name="camera_name">
//         ...
//       </camera>
//     </sensor>
//     <sensor name="sensor2_name" type="camera">
//       <topic>/another/image/topic</topic>
//       ...
//       <update_rate>...</update_rate>
//       <camera name="camera_name">
//         ...
//       </camera>
//     </sensor>
//   </link>
//   <plugin
//       filename="static://intrinsic::simulation::MultiCameraPlugin"
//       name="intrinsic::simulation::MultiCameraPlugin">
//       <camera_identifier>
//         plenoptic_unit {
//           device_id: "ips_device_id"
//         }
//       </camera_identifier>
//     <sensor>
//       <id>0</id>
//       <name>sensor1_name</name>
//       <image_type>intensity</image_type>
//     </sensor>
//     <sensor>
//       <id>1</id>
//       <name>sensor1_name</name>
//       <image_type>point</image_type>
//     </sensor>
//     <sensor>
//       <id>2</id>
//       <name>sensor2_name</name>
//     </sensor>
//   </plugin>
// </model>
//
// The plugin takes a single <camera_identifier> parameter:
//   <camera_identifier>: Serialized CameraIdentifier proto which is
//     automatically added during the world to sdf conversion based on the
//     associated camera configuration and should not be set explicitly outside
//     of tests.
// The plugin also takes a repeated list of <sensor> parameters:
//   <sensor>
//     <id>:         Unique sensor ID across all camera image outputs.
//     <name>:       Name of sensor in this model.
//     <image_type>: (Optional for `camera`, required for `rgbd_camera`).
//                   Supported image types: 'intensity', 'depth', 'point',
//                   'normal'.
//
// Limitations:
//   - For `camera` sensors: Only 'intensity' image type is supported (default).
//   - For `rgbd_camera` sensors: `image_type` must be explicitly specified for
//     every sensor ID mapping to an `rgbd_camera`. Omitting `image_type` for an
//     RGBD sensor results in an error during configuration and the camera will
//     not be registered.
//   - Sensor IDs must be unique across all sensors and image types.
//   - Supported image formats are RGB_INT8, L_INT8 (intensity), and R_FLOAT32
//     (depth/point/normal).
class MultiCameraPlugin : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPostUpdate {
 public:
  using SensorPropertiesMap =
      absl::flat_hash_map<int64_t, CameraSensorProperty>;

  ~MultiCameraPlugin() override;

  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& eventMgr) final;

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) final;

  SensorPropertiesMap GetSensorProperties() const { return sensor_properties_; }

  absl::StatusOr<intrinsic::perception::GazeboCameraConnection*>
  GetCameraConnection() const;

 protected:
  void InitCameraConnection();
  void OnImage(const gz::msgs::Image& msg,
               const gz::transport::MessageInfo& info)
      ABSL_LOCKS_EXCLUDED(image_mutex_);
  void SetSensorImages() ABSL_EXCLUSIVE_LOCKS_REQUIRED(image_mutex_);

  // Triggers sensors associated with the given `sensor_ids`. If `sensor_ids`
  // is empty, all sensors for the camera device are triggered.
  absl::Status TriggerSensors(
      const absl::flat_hash_set<int64_t>& sensor_ids = {});

  // Status of the camera connection initialization / plugin configuration.
  absl::Status connection_status_ =
      absl::FailedPreconditionError("Plugin has not been configured.");

  // Model name of the multi-camera device.
  std::string model_name_;

  // Camera identifier for the multi-camera resource.
  perception::CameraIdentifier camera_identifier_;

  // Map of sensor id to sensor properties.
  SensorPropertiesMap sensor_properties_;

  gz::transport::Node node_;

  std::unique_ptr<intrinsic::perception::GazeboCameraConnection> connection_;

  // Last generated sensor images
  std::vector<intrinsic::perception::SensorImage> sensor_images_
      ABSL_GUARDED_BY(image_mutex_);

  // A unique set of sensors that have received sensor images from sim
  // when processing a capture request
  absl::flat_hash_set<int64_t> received_sensor_ids_
      ABSL_GUARDED_BY(image_mutex_);

  // A unique set of sensors that are set to be active when processing a
  // capture request
  absl::flat_hash_set<int64_t> active_sensor_ids_ ABSL_GUARDED_BY(image_mutex_);

  // A map of sensor topic to sensor ids
  absl::flat_hash_map<std::string, absl::flat_hash_set<int64_t>> sensor_topics_;

  // A map of trigger topic to triggered camera clients.
  absl::node_hash_map<std::string, TriggeredCameraClient>
      triggered_camera_clients_;

  // Mutex to protect sensor images in OnImage callback
  absl::Mutex image_mutex_;

  // Flag to track if all sensor topics are set
  bool sensor_topics_set_{false};

  // True if there is an active request; this will be reset after all requested
  // image are sent.
  bool image_request_active_ ABSL_GUARDED_BY(image_mutex_) = false;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_MULTI_CAMERA_PLUGIN_H_
