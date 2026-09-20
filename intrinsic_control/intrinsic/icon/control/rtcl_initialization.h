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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_INITIALIZATION_H_
#define INTRINSIC_ICON_CONTROL_RTCL_INITIALIZATION_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/parts/part_property_registry.h"
#include "intrinsic/icon/control/parts/proto/v1/realtime_part_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_part.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

// Holds a part's configuration bundle.
// This includes the type names of the part itself and its safety action.
struct RealtimePartConfig {
  std::string part_type_name;
  std::string hardware_resource_name;
  std::string safety_action_type_name;
};

// Holds the initialized bundle of a part and its safety action.
struct PartAndSafetyAction {
  RealtimePart part;
  std::unique_ptr<StreamingIoStorage> safety_action_io_storage;
  RtclActionInstance safety_action;
};

// Holds the return values of all Part factories. Each factory produces
// * A PartAndSafetyAction instance, which is used in realtime
// * A PartConfig proto, which is used in non-realtime. For example, we use that
//   proto to service GetConfig() calls and determine Action compatibility.
struct RtclPartsAndConfigs {
  intrinsic::FixedVector<PartAndSafetyAction, kMaxRealtimeParts>
      parts_with_safety_actions;
  std::vector<intrinsic_proto::icon::v1::PartConfig> part_configs;
  intrinsic::FixedVector<std::vector<PartPropertyInitialData>,
                         kMaxRealtimeParts>
      part_property_initial_data_in_index_order;
};

// Same as above, but uses proto-based configuration instead of ConfigNode.
absl::StatusOr<RtclPartsAndConfigs> InitializePartsFromProto(
    const ::intrinsic_proto::icon::v1::ServerConfig& server_config,
    const absl::flat_hash_map<std::string,
                              intrinsic_proto::v1::RealtimePartConfig>&
        realtime_part_configs_by_part_name,
    const Context& context);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_RTCL_INITIALIZATION_H_
