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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_SIGNAL_STORAGE_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_SIGNAL_STORAGE_H_

#include <string>

#include "absl/container/fixed_array.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/realtime_signal_types.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic::icon {

struct RealtimeSignalStorage {
  absl::FixedArray<SignalValue> signal_values;
};

struct SignalStorageAndIdMap {
  RealtimeSignalStorage signal_storage;
  absl::flat_hash_map<std::string, RealtimeSignalId> signal_id_map;
};

// Returns a RealtimeSignalStorage object for the realtime signals registered in
// `signature`, along with a map from signal name to RealtimeSignalId.
//
// Pass the RealtimeSignalStorage object to the real time thread, and pass the
// map to ActionFactoryContext.
absl::StatusOr<SignalStorageAndIdMap> CreateRealtimeSignalStorage(
    const ::intrinsic_proto::icon::v1::ActionSignature& signature);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_SIGNAL_STORAGE_H_
