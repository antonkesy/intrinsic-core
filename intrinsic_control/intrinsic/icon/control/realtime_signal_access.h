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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_SIGNAL_ACCESS_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_SIGNAL_ACCESS_H_

#include "intrinsic/icon/control/realtime_signal_storage.h"
#include "intrinsic/icon/control/realtime_signal_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// We pass this to the Sense() method of real-time Actions so they can read the
// real-time signals. All methods are non-blocking and do not allocate memory.
// This class is not thread safe, since we assume a single realtime thread.
class RealtimeSignalAccess {
 public:
  explicit RealtimeSignalAccess(RealtimeSignalStorage& storage)
      : storage_(storage) {}

  RealtimeStatusOr<SignalValue> ReadSignal(RealtimeSignalId id) {
    if (id.value() >= storage_.signal_values.size()) {
      return icon::NotFoundError(RealtimeStatus::StrCat(
          "No realtime signal found with id: ", id.value()));
    }
    return storage_.signal_values[id.value()];
  }

 private:
  RealtimeSignalStorage& storage_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_SIGNAL_ACCESS_H_
