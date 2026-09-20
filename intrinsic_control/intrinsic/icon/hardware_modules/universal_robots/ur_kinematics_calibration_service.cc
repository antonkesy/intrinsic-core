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

#include "intrinsic/icon/hardware_modules/universal_robots/ur_kinematics_calibration_service.h"

#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "grpcpp/server.h"
#include "grpcpp/support/status.h"
#include "intrinsic/icon/hardware_modules/universal_robots/ur_calibration_utils.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "intrinsic/scene/proto/v1/object_properties.pb.h"
#include "intrinsic/util/status/status_builder_grpc.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"
#include "intrinsic/world/service/robot_calibration/robot_calibration_data_service.pb.h"

namespace intrinsic::icon {

UrKinematicsCalibrationDataService::UrKinematicsCalibrationDataService(
    const std::vector<EntityAndTransform>& robot_chain_updates,
    absl::string_view ik_solver_key, absl::string_view ik_solver_tip) {
  SetCalibratedKinematicsResponse(robot_chain_updates, ik_solver_key,
                                  ik_solver_tip);
}

grpc::Status UrKinematicsCalibrationDataService::GetCalibratedKinematics(
    ::grpc::ServerContext* context,
    const ::intrinsic_proto::world::CalibratedKinematicsRequest* request,
    ::intrinsic_proto::world::CalibratedKinematicsResponse* response) {
  absl::MutexLock lock(mutex_);
  if (!calibrated_kinematics_response_.has_value()) {
    return FailedPreconditionErrorBuilderGrpc()
           << "No robot data available. Likely due to no connection to the "
              "robot.";
  }
  LOG(INFO) << "GetKinematicsUpdate called.";
  *response = calibrated_kinematics_response_.value();
  LOG(INFO) << "Response:" << response;

  return grpc::Status::OK;
}

void UrKinematicsCalibrationDataService::SetCalibratedKinematicsResponse(
    const std::vector<EntityAndTransform>& robot_chain_updates,
    absl::string_view ik_solver_key, absl::string_view ik_solver_tip) {
  intrinsic_proto::world::CalibratedKinematicsResponse
      calibrated_kinematics_response;
  for (const auto& entity_and_transform : robot_chain_updates) {
    if (entity_and_transform.is_joint) {
      auto* joint_update = calibrated_kinematics_response
                               .mutable_scene_object_kinematics_update()
                               ->mutable_update_joints_request()
                               ->mutable_parent_t_inboard();
      joint_update->insert({entity_and_transform.entity_name,
                            ToProto(entity_and_transform.parent_t_entity)});
    } else {
      auto* link_update = calibrated_kinematics_response
                              .mutable_scene_object_kinematics_update()
                              ->add_entity_pose_updates();
      link_update->set_entity_name(entity_and_transform.entity_name);
      *link_update->mutable_parent_t_this() =
          ToProto(entity_and_transform.parent_t_entity);
    }
  }
  intrinsic_proto::scene_object::v1::SetIKSolversUpdate set_ik_solvers_update;
  intrinsic_proto::scene_object::v1::IkSolver* ik_solver =
      set_ik_solvers_update.add_ik_solvers();
  ik_solver->set_ik_solver(ik_solver_key);
  ik_solver->set_tip_link_name(ik_solver_tip);
  *calibrated_kinematics_response.mutable_scene_object_kinematics_update()
       ->mutable_set_ik_solvers_update() = set_ik_solvers_update;

  absl::MutexLock lock(mutex_);
  calibrated_kinematics_response_ = std::move(calibrated_kinematics_response);
}
}  // namespace intrinsic::icon
