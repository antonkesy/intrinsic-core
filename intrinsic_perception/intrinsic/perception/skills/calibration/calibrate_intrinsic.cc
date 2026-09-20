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

#include "intrinsic/perception/skills/calibration/calibrate_intrinsic.h"

#include <memory>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "grpcpp/client_context.h"
#include "intrinsic/assets/dependencies/utils.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/perception/proto/v1/calibration_service.grpc.pb.h"
#include "intrinsic/perception/proto/v1/calibration_service.pb.h"
#include "intrinsic/perception/skills/calibration/calibrate_intrinsic.pb.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace skills {

char kCalibrationInterface[] =
    "grpc://intrinsic_proto.perception.v1.CalibrationService";

std::unique_ptr<SkillInterface> CalibrateCameraIntrinsic::CreateSkill() {
  return std::make_unique<CalibrateCameraIntrinsic>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
CalibrateCameraIntrinsic::Execute(const ExecuteRequest& request,
                                  ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request
          .params<intrinsic_proto::skills::CalibrateCameraIntrinsicParams>());
  // Connect to the Calibration service.
  INTR_ASSIGN_OR_RETURN(std::shared_ptr<grpc::Channel> channel,
                        intrinsic::assets::dependencies::Connect(
                            params.calibration_service(), kCalibrationInterface,
                            connect::UnlimitedMessageSizeGrpcChannelArgs()));
  auto calibration_service_stub =
      intrinsic_proto::perception::v1::CalibrationService::NewStub(channel);

  intrinsic_proto::perception::v1::CalibrationRequest calibration_request;
  calibration_request.set_calibrate_intrinsics(true);
  intrinsic_proto::perception::v1::CalibrationResult calibration_response;
  ::grpc::ClientContext ctx;
  INTR_RETURN_IF_ERROR(ToAbslStatus(calibration_service_stub->Calibrate(
      &ctx, calibration_request, &calibration_response)));
  if (calibration_response.intrinsic_calibration_results().empty()) {
    return absl::InternalError(
        "Intrinsic calibration failed to produce any results.");
  }
  auto return_value = std::make_unique<
      intrinsic_proto::skills::CalibrateCameraIntrinsicResult>();
  *return_value->mutable_calibration_results() =
      calibration_response.intrinsic_calibration_results();
  LOG(INFO)
      << "intrinsic_calibration_results: "
      << calibration_response.intrinsic_calibration_results(0).DebugString();
  return return_value;
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
CalibrateCameraIntrinsic::Preview(const PreviewRequest& request,
                                  PreviewContext& context) {
  return std::make_unique<
      intrinsic_proto::skills::CalibrateCameraIntrinsicResult>();
}

absl::StatusOr<intrinsic_proto::skills::Footprint>
CalibrateCameraIntrinsic::GetFootprint(const GetFootprintRequest& request,
                                       GetFootprintContext& context) const {
  intrinsic_proto::skills::Footprint result;
  // We lock the universe and thus prevent parallel execution for the execution
  // of calibration.
  result.set_lock_the_universe(true);
  return result;
}

}  // namespace skills
}  // namespace intrinsic
