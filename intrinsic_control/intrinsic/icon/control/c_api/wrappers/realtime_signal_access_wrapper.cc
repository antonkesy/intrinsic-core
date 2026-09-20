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

#include "intrinsic/icon/control/c_api/wrappers/realtime_signal_access_wrapper.h"

#include <cstdint>

#include "intrinsic/icon/control/c_api/c_realtime_signal_access.h"
#include "intrinsic/icon/control/c_api/c_realtime_status.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/c_api/convert_c_realtime_status.h"
#include "intrinsic/icon/control/c_api/convert_c_types.h"
#include "intrinsic/icon/control/realtime_signal_access.h"
#include "intrinsic/icon/control/realtime_signal_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {
namespace {

IntrinsicIconRealtimeStatus ReadSignal(IntrinsicIconRealtimeSignalAccess* self,
                                       uint64_t id,
                                       IntrinsicIconSignalValue* signal_value) {
  RealtimeSignalAccess* realtime_signal_access =
      reinterpret_cast<RealtimeSignalAccess*>(self);
  RealtimeStatusOr<SignalValue> icon_signal_value =
      realtime_signal_access->ReadSignal(RealtimeSignalId(id));
  if (!icon_signal_value.ok()) {
    return FromRealtimeStatus(icon_signal_value.status());
  }
  *signal_value = Convert(icon_signal_value.value());
  return FromRealtimeStatus(OkStatus());
}

}  // namespace

IntrinsicIconRealtimeSignalAccess* Wrap(
    RealtimeSignalAccess* realtime_signal_access) {
  return reinterpret_cast<IntrinsicIconRealtimeSignalAccess*>(
      realtime_signal_access);
}

const IntrinsicIconRealtimeSignalAccess* Wrap(
    const RealtimeSignalAccess* realtime_signal_access) {
  return reinterpret_cast<const IntrinsicIconRealtimeSignalAccess*>(
      realtime_signal_access);
}

IntrinsicIconRealtimeSignalAccessVtable GetRealtimeSignalAccessVtable() {
  return {
      .read_signal = &ReadSignal,
  };
}

}  // namespace intrinsic::icon
