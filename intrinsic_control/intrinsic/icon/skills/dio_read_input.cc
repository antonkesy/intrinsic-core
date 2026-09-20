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

#include "intrinsic/icon/skills/dio_read_input.h"

#include <cstdint>
#include <memory>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "google/protobuf/message.h"
#include "intrinsic/icon/cc_client/client.h"
#include "intrinsic/icon/equipment/channel_factory.h"
#include "intrinsic/icon/equipment/equipment_utils.h"
#include "intrinsic/icon/equipment/icon_equipment.pb.h"
#include "intrinsic/icon/proto/io_block.pb.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/skills/dio_read_input.pb.h"
#include "intrinsic/icon/skills/util/adio_util.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/cc/skill_utils.h"
#include "intrinsic/util/grpc/channel_interface.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::skills {

namespace {

using ::intrinsic_proto::icon::DigitalSignal;
using ::intrinsic_proto::skills::DioReadInputReturnValue;

absl::Status CopyOrderedSignals(
    const google::protobuf::Map<uint32_t, DigitalSignal>& input,
    DioReadInputReturnValue& read_values) {
  read_values.clear_values();
  for (int index = 0; index < input.size(); ++index) {
    if (!input.contains(index)) {
      return absl::OutOfRangeError(absl::StrFormat(
          "Expected index '%d' in values but failed to locate it.", index));
    }
    read_values.add_values(input.at(index).value());
  }
  return absl::OkStatus();
}

}  // namespace

std::unique_ptr<SkillInterface> DioReadInput::CreateSkill() {
  return std::make_unique<DioReadInput>(
      std::make_unique<icon::DefaultChannelFactory>());
}

absl::StatusOr<intrinsic_proto::skills::Footprint> DioReadInput::GetFootprint(
    const GetFootprintRequest& request, GetFootprintContext& context) const {
  intrinsic_proto::skills::Footprint result;
  // Do not lock the universe to allow parallel skill execution.
  // This is the default, but we want to be explicit here.
  result.set_lock_the_universe(false);
  return std::move(result);
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
DioReadInput::Execute(const ExecuteRequest& request, ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<intrinsic_proto::skills::DioReadInputParams>(),
      _.LogError());

  if (!params.robot_name().empty()) {
    LOG(WARNING) << "Ignoring obsolete 'robot_name' parameter with value: "
                 << params.robot_name();
  }

  INTR_ASSIGN_OR_RETURN(const auto timeout, ToAbslDuration(params.timeout()));

  INTR_ASSIGN_OR_RETURN(const auto& handle,
                        context.equipment().GetHandle(kEquipmentSlot));
  INTR_ASSIGN_OR_RETURN(const auto connection_config,
                        skills::GetConnectionParamsFromHandle(handle));
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<ChannelInterface> channel,
      icon_channel_factory_->MakeChannel(connection_config, timeout));
  icon::Client icon_client(channel);
  INTR_ASSIGN_OR_RETURN(auto config, icon_client.GetConfig());

  auto config_it = handle.resource_data().find(icon::kIcon2AdioPartKey);
  if (config_it == handle.resource_data().end()) {
    return absl::NotFoundError("ADIO config not found for resource handle.");
  }
  const auto& equipment_config = config_it->second;
  intrinsic_proto::icon::Icon2AdioPart adio_equipment_config;
  if (!equipment_config.contents().UnpackTo(&adio_equipment_config)) {
    return absl::InvalidArgumentError("Failed to unpack equipment config.");
  }

  INTR_ASSIGN_OR_RETURN(auto block_to_part_name,
                        GetBlockToPartNameMap(adio_equipment_config, config,
                                              {.digital_in = true}));

  auto it = block_to_part_name.find(params.block_name());
  if (it == block_to_part_name.end()) {
    return absl::NotFoundError(absl::StrCat(
        "Block '", params.block_name(),
        "' not found in the generic part config of any ADIO part."));
  }
  const std::string& part_name = it->second;

  INTR_ASSIGN_OR_RETURN(intrinsic_proto::icon::PartStatus part_status,
                        icon_client.GetSinglePartStatus(part_name));

  if (!part_status.has_adio_state()) {
    return absl::NotFoundError(absl::StrCat("Part status of part '", part_name,
                                            "' is missing ADIO state in '",
                                            kSkillName, "' skill."));
  }

  auto return_value =
      std::make_unique<intrinsic_proto::skills::DioReadInputReturnValue>();
  const auto& adio_state = part_status.adio_state();
  auto dio_block = adio_state.digital_inputs().find(params.block_name());
  if (dio_block == adio_state.digital_inputs().end()) {
    return absl::NotFoundError(absl::StrCat("Block '", params.block_name(),
                                            "' not found in part status of '",
                                            part_name, "'."));
  }
  const auto& signals = dio_block->second.signals();
  INTR_RETURN_IF_ERROR(CopyOrderedSignals(signals, *return_value));
  return return_value;
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
DioReadInput::Preview(const PreviewRequest& request, PreviewContext& context) {
  return std::make_unique<intrinsic_proto::skills::DioReadInputReturnValue>();
}

}  // namespace intrinsic::skills
