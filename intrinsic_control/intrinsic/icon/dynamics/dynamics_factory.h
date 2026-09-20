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

#ifndef INTRINSIC_ICON_DYNAMICS_DYNAMICS_FACTORY_H_
#define INTRINSIC_ICON_DYNAMICS_DYNAMICS_FACTORY_H_

#include <functional>
#include <memory>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/kinematics/skeleton.h"

namespace intrinsic::icon {

class DynamicsFactory {
 public:
  using DynamicsFactoryFunction =
      absl::StatusOr<std::unique_ptr<RigidBodyInterface>>(
          std::unique_ptr<kinematics::Skeleton> skeleton);

  // Returns a unique_ptr to an implementation of RigidBodyDynamicsInterface.
  // Returns nullptr on error
  absl::StatusOr<std::unique_ptr<RigidBodyInterface>> CreateDynamicsImpl(
      const std::string& dynamics_name,
      std::unique_ptr<kinematics::Skeleton> skeleton) const;

  // Register a function for creating a manipulator dynamics interface
  // implementation. Returns false on error, true on successful registration.
  bool Register(absl::string_view name,
                std::function<DynamicsFactoryFunction> func);

  void ClearForTestingOnly();

 private:
  mutable absl::Mutex mutex_;
  absl::flat_hash_map<std::string, std::function<DynamicsFactoryFunction>>
      registry_ ABSL_GUARDED_BY(mutex_);
};

// Access the global registry.
DynamicsFactory& GetGlobalDynamicsFactory();

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_DYNAMICS_DYNAMICS_FACTORY_H_
