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

#include "intrinsic/simulation/gazebo/plugins/cameras/triggered_camera_client.h"

#include <string>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "gz/msgs/MessageTypes.hh"
#include "gz/sim/rendering/Events.hh"
#include "gz/transport/Node.hh"
#include "intrinsic/util/macros.h"

ABSL_FLAG(absl::Duration, render_engine_initialization_timeout,
          absl::Seconds(30),
          "Time to block on image request for render engine to be ready.");

namespace intrinsic {
namespace simulation {

TriggeredCameraClient::TriggeredCameraClient(
    const std::string& camera_model_name, const std::string& trigger_topic,
    gz::transport::Node& node, gz::sim::EventManager& eventMgr)
    : camera_model_name_(camera_model_name) {
  LOG(INFO) << "Trigger topic: " << trigger_topic
            << " for camera: " << camera_model_name_;
  trigger_pub_ = node.Advertise<gz::msgs::Boolean>(trigger_topic);
  LOG_IF(ERROR, !trigger_pub_.value())
      << "Failed to advertise trigger topic: " << trigger_topic
      << " for camera: " << camera_model_name_;

  // Listen on SceneUpdate events published from the Sensors system. The first
  // event indicates that the render engine has been initialized.
  scene_update_connection_ =
      eventMgr.Connect<gz::sim::events::SceneUpdate>([this]() {
        absl::MutexLock l(&render_engine_initializated_mutex_);
        render_engine_initialized_ = true;
      });
}

TriggeredCameraClient::~TriggeredCameraClient() {
  // Disconnect the scene update connection to prevent dangling pointers.
  scene_update_connection_.reset();
}

absl::Status TriggeredCameraClient::Trigger() {
  if (!trigger_pub_.has_value()) {
    return FailedPreconditionErrorBuilder()
           << "Failed to advertise on trigger topic for camera ["
           << camera_model_name_ << "].";
  }

  // Ensure render engine is initialized before publishing on the trigger topic.
  // Otherwise the message will be ignored in the render engine.
  {
    absl::MutexLock l(&render_engine_initializated_mutex_);
    LOG(INFO) << "Waiting for render engine to initialize for camera: "
              << camera_model_name_;
    if (!render_engine_initialized_) {
      auto initialized = [this]() {
        render_engine_initializated_mutex_.AssertHeld();
        return render_engine_initialized_;
      };
      bool did_initialize = render_engine_initializated_mutex_.AwaitWithTimeout(
          absl::Condition(&initialized),
          absl::GetFlag(FLAGS_render_engine_initialization_timeout));
      if (!did_initialize) {
        return UnavailableErrorBuilder()
               << "Render engine was not initialized within timeout for "
                  "camera ["
               << camera_model_name_ << "].";
      }
    }
    // Reset the SceneUpdate event connection since we no longer need it.
    scene_update_connection_.reset();
  }

  if (!trigger_pub_->HasConnections()) {
    return AbortedErrorBuilder()
           << "Cannot activate camera. Render engine is initialized, but "
              "no subscribers were found on the camera trigger topic. "
              "Sensor initialization has likely failed for camera ["
           << camera_model_name_ << "].";
  }

  if (!trigger_pub_->Publish(gz::msgs::Boolean{})) {
    return InternalErrorBuilder()
           << "Failed to publish on trigger topic for camera ["
           << camera_model_name_ << "].";
  }
  return absl::OkStatus();
}

}  // namespace simulation
}  // namespace intrinsic
