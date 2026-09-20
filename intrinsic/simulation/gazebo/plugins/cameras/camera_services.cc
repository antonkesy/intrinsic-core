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

#include "intrinsic/simulation/gazebo/plugins/cameras/camera_services.h"

#include <cstdlib>
#include <functional>
#include <memory>

#include "absl/base/no_destructor.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/synchronization/mutex.h"
#include "grpc/grpc.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server_builder.h"
#include "intrinsic/simulation/gazebo/plugins/cameras/camera_service.h"
#include "intrinsic/util/thread/thread.h"

namespace intrinsic {
namespace perception {

GazeboCameraGrpcServices&
GazeboCameraGrpcServices::StartCameraServicesSingleton() {
  struct BackgroundCameraService {
    BackgroundCameraService()
        : thread(std::bind_front(&GazeboCameraGrpcServices::StartServerAndBlock,
                                 &services)) {}
    GazeboCameraGrpcServices services;
    Thread thread;
  };
  static absl::NoDestructor<BackgroundCameraService> kCameraService;
  absl::MutexLock lock(kCameraService->services.server_exists_mtx_);
  kCameraService->services.server_exists_mtx_.Await(absl::Condition(
      &kCameraService->services, &GazeboCameraGrpcServices::ServerExists));
  return kCameraService->services;
}

perception::GazeboCameraGrpcService* GazeboCameraGrpcServices::service() const {
  return camera_service_.get();
}

bool GazeboCameraGrpcServices::ServerExists() const { return server_exists_; }

void GazeboCameraGrpcServices::StartServerAndBlock() {
  camera_service_ = std::make_shared<perception::GazeboCameraGrpcService>();
  const char* server_address = getenv("CAMERA_SERVICE_ADDRESS");
  if (server_address == nullptr) {
    server_address = "0.0.0.0:15333";
  }
  grpc::ServerBuilder builder;
  builder.AddChannelArgument(GRPC_ARG_ALLOW_REUSEPORT, 0);
  builder.AddListeningPort(
      server_address,
      grpc::InsecureServerCredentials());  // NOLINT (insecure)
  builder.RegisterService(camera_service_.get());
  server_ = builder.BuildAndStart();
  CHECK(server_) << "Can't set up CameraServices";
  {
    absl::MutexLock lock(server_exists_mtx_);
    server_exists_ = true;
  }
  LOG(INFO) << "Started Gazebo CameraServices at: " << server_address;
  server_->Wait();
}

}  // namespace perception
}  // namespace intrinsic
