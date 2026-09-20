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

#include "intrinsic/icon/control/parts/hal/arm_part/hal_arm_part.h"
#include "intrinsic/icon/control/parts/hal/arm_part/hal_arm_part_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_part_from_proto_factory_registry.h"

// Registers the factory for the HalArmPart.
// We have this separate cc file so no header file gets the linker flag
// alwayslink=True, which would have far reaching effects and is discouraged.

namespace intrinsic::icon {
namespace {

// Proto version.
const auto kUnused =
    GetGlobalRealtimePartFromProtoFactoryRegistry()
        .RegisterTyped<intrinsic_proto::icon::HalArmPartConfig>(
            HalArmPart::kPartTypeName, &HalArmPart::FromProto);

}  // namespace
}  // namespace intrinsic::icon
