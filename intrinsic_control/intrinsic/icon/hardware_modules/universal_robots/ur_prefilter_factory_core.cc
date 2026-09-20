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

#include <memory>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/hardware_modules/universal_robots/config.pb.h"
#include "intrinsic/icon/hardware_modules/universal_robots/pass_through_filter.h"
#include "intrinsic/icon/hardware_modules/universal_robots/prefilter_interface.h"
#include "intrinsic/icon/hardware_modules/universal_robots/ur_prefilter_factory.h"

namespace intrinsic::icon {

absl::StatusOr<std::unique_ptr<PrefilterInterface>> CreateUrPrefilter(
    absl::string_view robot_model_name,
    const ::intrinsic_proto::icon::UniversalRobotsModuleConfig& proto_config) {
  switch (proto_config.advanced_control_options_case()) {
    // If parameter unset, the default advanced control is used.
    case intrinsic_proto::icon::UniversalRobotsModuleConfig::
        AdvancedControlOptionsCase::ADVANCED_CONTROL_OPTIONS_NOT_SET:
      [[fallthrough]];
    case intrinsic_proto::icon::UniversalRobotsModuleConfig::
        AdvancedControlOptionsCase::kDefaultAdvancedControl: {
      LOG(WARNING)
          << "Advanced control is not available in intrinsic-core builds. "
             "Falling back to PassThroughFilter.";
      return std::make_unique<PassThroughFilter>();
    }

    case intrinsic_proto::icon::UniversalRobotsModuleConfig::
        AdvancedControlOptionsCase::kNoAdvancedControl: {
      return std::make_unique<PassThroughFilter>();
    }

    case intrinsic_proto::icon::UniversalRobotsModuleConfig::
        AdvancedControlOptionsCase::kCustomAdvancedControl: {
      return absl::UnimplementedError(
          "Custom advanced control is not available in intrinsic-core builds.");
    }
  }
}

}  // namespace intrinsic::icon
