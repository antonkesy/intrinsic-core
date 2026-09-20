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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_CAMERA_SERVICES_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_CAMERA_SERVICES_H_

#include <memory>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/server.h"
#include "intrinsic/simulation/gazebo/plugins/cameras/camera_service.h"

namespace intrinsic {
namespace perception {

class GazeboCameraGrpcServices {
 public:
  static GazeboCameraGrpcServices& StartCameraServicesSingleton();

  perception::GazeboCameraGrpcService* service() const;

 private:
  bool ServerExists() const ABSL_EXCLUSIVE_LOCKS_REQUIRED(server_exists_mtx_);
  std::shared_ptr<perception::GazeboCameraGrpcService> camera_service_;
  absl::Mutex server_exists_mtx_;
  bool server_exists_ ABSL_GUARDED_BY(server_exists_mtx_) = false;
  std::unique_ptr<grpc::Server> server_;

  void StartServerAndBlock();
};

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_CAMERA_SERVICES_H_
