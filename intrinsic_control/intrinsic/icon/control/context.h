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

#ifndef INTRINSIC_ICON_CONTROL_CONTEXT_H_
#define INTRINSIC_ICON_CONTROL_CONTEXT_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/services/service_collection.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/server/runtime_options.h"

namespace intrinsic::icon {

// Makes services and hardware module connections available, mostly to parts.
// Does not take ownership.
class Context {
 public:
  // Creates an empty Context with default ServerRuntimeOptions (robot name is
  // "" and mode is kNormal).
  Context() = default;

  // Pointers must outlive the Context.
  Context(ServiceCollection* service_collection,
          HardwareModuleManager* hardware_module_manager,
          const ServerRuntimeOptions& runtime_options)
      : services_(service_collection),
        hardware_module_manager_(hardware_module_manager),
        runtime_options_(runtime_options) {}

  // Get a service from the service collection. Returns nullptr if the service
  // is not registered.
  template <class T>
  T* GetServiceOrNull() const {
    if (!services_) {
      return nullptr;
    }
    return services_->GetServiceOrNull<T>();
  }

  // Returns server runtime options. The returned reference is only valid for
  // the lifetime of this Context object.
  const ServerRuntimeOptions& GetRuntimeOptions() const {
    return runtime_options_;
  }

  // Returns a handle to a hardware interface exported from a hardware module.
  // While the function is thread compatible, obtaining hardware interfaces
  // shall be obtained during initialization phase.
  template <class InterfaceT>
  absl::StatusOr<intrinsic::icon::HardwareInterfaceHandle<InterfaceT>>
  GetHardwareInterfaceHandle(absl::string_view module_name,
                             absl::string_view interface_name) const {
    if (hardware_module_manager_ == nullptr) {
      return absl::FailedPreconditionError("Hardware module manager is null.");
    }
    auto proxy = hardware_module_manager_->GetHardwareModuleProxy(module_name);
    if (proxy == nullptr) {
      return absl::InvalidArgumentError(
          absl::StrCat("Hardware module '", module_name, "' not found."));
    }
    return proxy->GetHardwareInterface<InterfaceT>(interface_name);
  }

  // Returns a mutable handle to a hardware interface exported from a hardware
  // module.
  // While the function is thread compatible, obtaining hardware interfaces
  // shall be obtained during initialization phase.
  template <class InterfaceT>
  absl::StatusOr<intrinsic::icon::MutableHardwareInterfaceHandle<InterfaceT>>
  GetMutableHardwareInterfaceHandle(absl::string_view module_name,
                                    absl::string_view interface_name) const {
    if (hardware_module_manager_ == nullptr) {
      return absl::FailedPreconditionError("Hardware module manager is null.");
    }
    auto proxy = hardware_module_manager_->GetHardwareModuleProxy(module_name);
    if (proxy == nullptr) {
      return absl::InvalidArgumentError(
          absl::StrCat("Hardware module '", module_name, "' not found."));
    }
    return proxy->GetMutableHardwareInterface<InterfaceT>(interface_name);
  }

  HardwareModuleManager* GetHardwareModuleManager() const {
    return hardware_module_manager_;
  }

 private:
  ServiceCollection* services_ = nullptr;
  HardwareModuleManager* hardware_module_manager_ = nullptr;
  ServerRuntimeOptions runtime_options_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_CONTEXT_H_
