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

#include "intrinsic/icon/skills/disable_realtime_control.h"

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/message.h"
#include "grpcpp/channel.h"
#include "intrinsic/assets/dependencies/utils.h"
#include "intrinsic/assets/interface_utils.h"
#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/icon/cc_client/client.h"
#include "intrinsic/icon/proto/v1/service.pb.h"
#include "intrinsic/icon/skills/disable_realtime_control.pb.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace skills {
namespace {

std::string IconApiInterfaceUri() {
  return absl::StrCat(::intrinsic::assets::kGrpcUriPrefix,
                      intrinsic_proto::icon::v1::IconApi::service_full_name());
}

}  // namespace

// static
std::unique_ptr<SkillInterface> DisableRealtimeControl::CreateSkill() {
  return std::make_unique<DisableRealtimeControl>();
}

absl::StatusOr<intrinsic_proto::skills::Footprint>
DisableRealtimeControl::GetFootprint(const GetFootprintRequest& request,
                                     GetFootprintContext& context) const {
  return intrinsic_proto::skills::Footprint();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
DisableRealtimeControl::Execute(const ExecuteRequest& request,
                                ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<intrinsic_proto::skills::DisableRealtimeControlParams>());
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> channel,
      assets::dependencies::Connect(params.robot(), IconApiInterfaceUri()));

  icon::Client icon_client(std::make_shared<intrinsic::Channel>(channel));
  icon::Client::HardwareGroup group = icon::Client::kAllHardware;
  if (params.skip_cell_control_hardware()) {
    group = icon::Client::kOperationalHardwareOnly;
  }
  INTR_RETURN_IF_ERROR(icon_client.Disable(group));

  return nullptr;
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
DisableRealtimeControl::Preview(const PreviewRequest& request,
                                PreviewContext& context) {
  return nullptr;
}

}  // namespace skills
}  // namespace intrinsic
