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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_TRIGGERED_CAMERA_CLIENT_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_TRIGGERED_CAMERA_CLIENT_H_

#include <optional>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/flags/declare.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "gz/common/Event.hh"
#include "gz/sim/EventManager.hh"
#include "gz/transport/Node.hh"

ABSL_DECLARE_FLAG(absl::Duration, render_engine_initialization_timeout);

namespace intrinsic {
namespace simulation {

// A helper class for managing triggered camera behavior in Gazebo.
// It handles waiting for the Gazebo render engine to initialize and then
// publishing a trigger message to the camera's trigger topic.
class TriggeredCameraClient {
 public:
  TriggeredCameraClient(const std::string& camera_model_name,
                        const std::string& trigger_topic,
                        gz::transport::Node& node,
                        gz::sim::EventManager& eventMgr);
  ~TriggeredCameraClient();

  // Trigger the camera .
  // Blocks until the render engine is initialized or a timeout occurs.
  absl::Status Trigger();

 private:
  const std::string camera_model_name_;
  std::optional<gz::transport::Node::Publisher> trigger_pub_;

  // Tracks whether the render engine has been initialized. Used to block
  // image requests before publishing trigger messages.
  bool render_engine_initialized_
      ABSL_GUARDED_BY(render_engine_initializated_mutex_) = false;
  absl::Mutex render_engine_initializated_mutex_;
  gz::common::ConnectionPtr scene_update_connection_;
};

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_TRIGGERED_CAMERA_CLIENT_H_
