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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_PART_STATUS_HELPERS_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_PART_STATUS_HELPERS_H_

#include "absl/time/time.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/realtime_part_status.h"
#include "intrinsic/icon/control/realtime_operational_status.h"

namespace intrinsic::icon {

// Extracts status data from the feature interfaces in
// `feature_interface_registry`, and puts them into a RealtimePartStatus.
// `operational_status` is the enabled/disabled/fault state of the part and is
// copied into the returned RealtimePartStatus.
RealtimePartStatus ExtractRealtimePartStatus(
    const FeatureInterfaceRegistry& feature_interface_registry,
    absl::Duration time_since_server_start,
    const RealtimeOperationalStatus& operational_status);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_PART_STATUS_HELPERS_H_
