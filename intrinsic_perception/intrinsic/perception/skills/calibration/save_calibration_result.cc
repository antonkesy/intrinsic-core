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

#include "intrinsic/perception/skills/calibration/save_calibration_result.h"

#include <memory>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "grpcpp/client_context.h"
#include "intrinsic/assets/dependencies/utils.h"
#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/geometry/proto/oriented_bounding_box.pb.h"
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/perception/proto/v1/calibration_service.grpc.pb.h"
#include "intrinsic/perception/proto/v1/calibration_service.pb.h"
#include "intrinsic/perception/skills/calibration/save_calibration_result.pb.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace skills {

namespace {
constexpr absl::string_view kCalibrationInterface =
    "grpc://intrinsic_proto.perception.v1.CalibrationService";
}  // namespace

std::unique_ptr<SkillInterface> SaveCalibrationResult::CreateSkill() {
  return std::make_unique<SaveCalibrationResult>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
SaveCalibrationResult::Execute(const ExecuteRequest& request,
                               ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<intrinsic_proto::skills::SaveCalibrationResultParams>());

  // Connect to the Calibration service.
  grpc::ClientContext ctx;
  INTR_ASSIGN_OR_RETURN(std::shared_ptr<grpc::Channel> channel,
                        intrinsic::assets::dependencies::Connect(
                            params.calibration_service(), kCalibrationInterface,
                            connect::UnlimitedMessageSizeGrpcChannelArgs()));
  std::unique_ptr<
      intrinsic_proto::perception::v1::CalibrationService::StubInterface>
      calibration_service_stub =
          intrinsic_proto::perception::v1::CalibrationService::NewStub(channel);

  intrinsic_proto::perception::v1::CalibrationResult calibration_result =
      params.calibration_result();
  google::protobuf::Empty empty_response;
  INTR_RETURN_IF_ERROR(ToAbslStatus(calibration_service_stub->Save(
      &ctx, calibration_result, &empty_response)));
  LOG(INFO) << "Saved calibration results.";
  return nullptr;
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
SaveCalibrationResult::Preview(const PreviewRequest& request,
                               PreviewContext& context) {
  return nullptr;
}

absl::StatusOr<intrinsic_proto::skills::Footprint>
SaveCalibrationResult::GetFootprint(const GetFootprintRequest& request,
                                    GetFootprintContext& context) const {
  intrinsic_proto::skills::Footprint result;
  // We lock the universe and thus prevent parallel execution for the execution
  // of calibration.
  result.set_lock_the_universe(true);
  return result;
}

}  // namespace skills
}  // namespace intrinsic
