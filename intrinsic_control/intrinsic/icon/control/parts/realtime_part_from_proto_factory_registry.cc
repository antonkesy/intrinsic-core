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

#include "intrinsic/icon/control/parts/realtime_part_from_proto_factory_registry.h"

#include <functional>
#include <string>
#include <utility>

#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/icon/utils/realtime_guard.h"

namespace intrinsic::icon {

bool RealtimePartFromProtoFactoryRegistry::Register(
    absl::string_view part_type_name, std::function<FactorySignature> factory) {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock lock(mutex_);
  if (registry_.contains(part_type_name)) return false;
  registry_.emplace(std::string(part_type_name), std::move(factory));
  return true;
}

std::function<RealtimePartFromProtoFactoryRegistry::FactorySignature>
RealtimePartFromProtoFactoryRegistry::Get(
    absl::string_view part_type_name) const {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock lock(mutex_);
  auto it = registry_.find(part_type_name);
  if (it == registry_.end()) return {};
  return it->second;
}

void RealtimePartFromProtoFactoryRegistry::ClearForTestingOnly() {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock lock(mutex_);
  registry_.clear();
}

RealtimePartFromProtoFactoryRegistry&
GetGlobalRealtimePartFromProtoFactoryRegistry() {
  INTRINSIC_ASSERT_NON_REALTIME();
  // Avoid destruction of global object.
  static auto* r = new RealtimePartFromProtoFactoryRegistry;
  return *r;
}

}  // namespace intrinsic::icon
