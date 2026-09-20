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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_KINEMATICS_CALIBRATION_SERVICE_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_KINEMATICS_CALIBRATION_SERVICE_H_

#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/server.h"
#include "grpcpp/support/status.h"
#include "intrinsic/icon/hardware_modules/universal_robots/ur_calibration_utils.h"
#include "intrinsic/util/invalid_until_set.h"
#include "intrinsic/world/service/robot_calibration/robot_calibration_data_service.grpc.pb.h"
#include "intrinsic/world/service/robot_calibration/robot_calibration_data_service.pb.h"

namespace intrinsic::icon {

class UrKinematicsCalibrationDataService
    : public ::intrinsic_proto::world::RobotCalibrationDataService::Service {
 public:
  UrKinematicsCalibrationDataService() = default;
  // Make RobotKinematicsUpdate for calibrated UR robots available. No
  // processing is done in this service implementation."
  UrKinematicsCalibrationDataService(
      const std::vector<EntityAndTransform>& robot_chain_updates,
      absl::string_view ik_solver_key, absl::string_view ik_solver_tip);

  ::grpc::Status GetCalibratedKinematics(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::world::CalibratedKinematicsRequest* request,
      ::intrinsic_proto::world::CalibratedKinematicsResponse* response)
      override;

  // Set or overwrite the existing data.
  void SetCalibratedKinematicsResponse(
      const std::vector<EntityAndTransform>& robot_chain_updates,
      absl::string_view ik_solver_key, absl::string_view ik_solver_tip);

 private:
  absl::Mutex mutex_;
  InvalidUntilSet<intrinsic_proto::world::CalibratedKinematicsResponse>
      calibrated_kinematics_response_ ABSL_GUARDED_BY(mutex_);
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_UNIVERSAL_ROBOTS_UR_KINEMATICS_CALIBRATION_SERVICE_H_
