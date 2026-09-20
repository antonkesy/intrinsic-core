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

#include "intrinsic/perception/skills/calibration/calibrate_camera_to_robot.h"

#include <memory>
#include <string>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "grpcpp/client_context.h"
#include "intrinsic/assets/dependencies/utils.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/perception/proto/v1/calibration_service.grpc.pb.h"
#include "intrinsic/perception/proto/v1/calibration_service.pb.h"
#include "intrinsic/perception/proto/v1/camera_to_robot_calibration.pb.h"
#include "intrinsic/perception/skills/calibration/calibrate_camera_to_robot.pb.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace skills {

char kCalibrationInterface[] =
    "grpc://intrinsic_proto.perception.v1.CalibrationService";
namespace {

using intrinsic_proto::perception::v1::
    CAMERA_TO_ROBOT_CALIBRATION_TYPE_UNSPECIFIED;

absl::Status IsCalibrationErrorWithinThresholds(
    const intrinsic_proto::perception::v1::CameraToRobotCalibrationResult&
        calibration_result,
    double translation_root_mean_square_error_threshold,
    double rotation_root_mean_square_error_threshold) {
  std::vector<std::string> error_messages;
  if (calibration_result.translation_root_mean_square_error() >
      translation_root_mean_square_error_threshold) {
    error_messages.push_back(
        absl::StrFormat("translation_root_mean_square_error is higher than the "
                        "given threshold: %.5f > %.5f",
                        calibration_result.translation_root_mean_square_error(),
                        translation_root_mean_square_error_threshold));
  }
  if (calibration_result.rotation_root_mean_square_error_in_degrees() >
      rotation_root_mean_square_error_threshold) {
    error_messages.push_back(absl::StrFormat(
        "rotation_root_mean_square_error is higher than the "
        "given threshold: %.5f > %.5f",
        calibration_result.rotation_root_mean_square_error_in_degrees(),
        rotation_root_mean_square_error_threshold));
  }
  if (!error_messages.empty()) {
    return absl::FailedPreconditionError(absl::StrJoin(error_messages, ", "));
  }
  return absl::OkStatus();
}

}  // namespace

std::unique_ptr<SkillInterface> CalibrateCameraToRobot::CreateSkill() {
  return std::make_unique<CalibrateCameraToRobot>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
CalibrateCameraToRobot::Execute(const ExecuteRequest& request,
                                ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(
      auto input_params,
      request.params<intrinsic_proto::skills::CalibrateCameraToRobotParams>());
  if (input_params.calibration_type() ==
      CAMERA_TO_ROBOT_CALIBRATION_TYPE_UNSPECIFIED) {
    return absl::InvalidArgumentError("Invalid CameraToRobotCalibrationType.");
  }

  // Connect to the Calibration service.
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> channel,
      intrinsic::assets::dependencies::Connect(
          input_params.calibration_service(), kCalibrationInterface,
          connect::UnlimitedMessageSizeGrpcChannelArgs()));
  auto calibration_service_stub =
      intrinsic_proto::perception::v1::CalibrationService::NewStub(channel);

  intrinsic_proto::perception::v1::CalibrationRequest calibration_request;
  calibration_request.set_calibrate_camera_to_robot(true);
  calibration_request.set_camera_to_robot_calibration_type(
      input_params.calibration_type());
  intrinsic_proto::perception::v1::CalibrationResult calibration_response;
  ::grpc::ClientContext ctx;
  INTR_RETURN_IF_ERROR(ToAbslStatus(calibration_service_stub->Calibrate(
      &ctx, calibration_request, &calibration_response)));
  LOG(INFO) << "camera_to_robot_calibration_results: "
            << calibration_response.camera_to_robot_calibration_results(0)
                   .DebugString();
  if (!calibration_response.intrinsic_calibration_results().empty()) {
    return absl::InternalError(
        "Intrinsic calibration performed while camera-to-robot calibration had"
        " been requested.");
  }
  if (calibration_response.camera_to_robot_calibration_results().empty()) {
    return absl::InternalError(
        "Camera-to-robot calibration failed to produce any results.");
  }
  absl::Status calibration_error_within_threshold =
      IsCalibrationErrorWithinThresholds(
          calibration_response.camera_to_robot_calibration_results(0),
          input_params.translation_root_mean_square_error_threshold(),
          input_params.rotation_root_mean_square_error_threshold());
  if (!calibration_error_within_threshold.ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("Calibration error is too high: ",
                     calibration_error_within_threshold.message(),
                     " Try again using a different parameter set, e.g. "
                     "increasing the sampling volume or sampling more poses."));
  }

  // TODO(b/354670221): Later on, we may need to implement the logic for
  // computing the relative pose of the calibration object in the backend, and
  // to apply the new pose to the world from within the skill. This will enable
  // parity with the old hand-eye calibration skill. It is not yet clear if this
  // is needed.

  auto return_value =
      std::make_unique<intrinsic_proto::skills::CalibrateCameraToRobotResult>();
  *return_value->mutable_calibration_results() =
      calibration_response.camera_to_robot_calibration_results();
  return return_value;
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
CalibrateCameraToRobot::Preview(const PreviewRequest& request,
                                PreviewContext& context) {
  return std::make_unique<
      intrinsic_proto::skills::CalibrateCameraToRobotResult>();
}

absl::StatusOr<intrinsic_proto::skills::Footprint>
CalibrateCameraToRobot::GetFootprint(const GetFootprintRequest& request,
                                     GetFootprintContext& context) const {
  intrinsic_proto::skills::Footprint result;
  // We lock the universe and thus prevent parallel execution for the execution
  // of calibration.
  result.set_lock_the_universe(true);
  return result;
}

}  // namespace skills
}  // namespace intrinsic
