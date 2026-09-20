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

#include "intrinsic/icon/control/realtime_signal_storage.h"

#include <string>

#include "absl/container/fixed_array.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/control/realtime_signal_types.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic::icon {

absl::StatusOr<SignalStorageAndIdMap> CreateRealtimeSignalStorage(
    const ::intrinsic_proto::icon::v1::ActionSignature& signature) {
  absl::flat_hash_map<std::string, RealtimeSignalId> signal_id_map;
  for (const ::intrinsic_proto::icon::v1::ActionSignature::RealtimeSignalInfo&
           signal_info : signature.realtime_signal_infos()) {
    auto [signal_id_it, inserted] =
        signal_id_map.try_emplace(signal_info.signal_name());
    if (!inserted) {
      return absl::FailedPreconditionError(
          absl::StrCat("Encountered duplicate Signal name '",
                       signal_info.signal_name(), "'"));
    }
    signal_id_it->second = RealtimeSignalId(signal_id_map.size() - 1);
  }
  return SignalStorageAndIdMap{
      .signal_storage = {.signal_values = absl::FixedArray<SignalValue>(
                             signal_id_map.size())},
      .signal_id_map = signal_id_map,
  };
}

}  // namespace intrinsic::icon
