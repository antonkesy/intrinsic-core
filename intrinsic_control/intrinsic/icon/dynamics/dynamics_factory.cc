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

#include "intrinsic/icon/dynamics/dynamics_factory.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/kinematics/skeleton.h"

namespace intrinsic::icon {

absl::StatusOr<std::unique_ptr<RigidBodyInterface>>
DynamicsFactory::CreateDynamicsImpl(
    const std::string& dynamics_name,
    std::unique_ptr<kinematics::Skeleton> skeleton) const {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock lock(mutex_);
  auto it = registry_.find(dynamics_name);
  if (it == registry_.end()) {
    return absl::NotFoundError(
        absl::StrCat("No solver registered with name ", dynamics_name));
  }
  return it->second(std::move(skeleton));
}

bool DynamicsFactory::Register(absl::string_view name,
                               std::function<DynamicsFactoryFunction> func) {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock lock(mutex_);
  if (registry_.contains(name)) return false;
  registry_.emplace(std::string(name), std::move(func));
  return true;
}

void DynamicsFactory::ClearForTestingOnly() {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock l(mutex_);
  registry_.clear();
}

DynamicsFactory& GetGlobalDynamicsFactory() {
  INTRINSIC_ASSERT_NON_REALTIME();
  // Avoid destruction of global object.
  static auto* r = new DynamicsFactory;
  return *r;
}

}  // namespace intrinsic::icon
