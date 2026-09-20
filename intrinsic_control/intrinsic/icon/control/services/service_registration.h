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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_SERVICE_REGISTRATION_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_SERVICE_REGISTRATION_H_

#include <memory>
#include <string>
#include <utility>

#include "absl/hash/hash.h"
#include "cppregpattern/registry.h"
#include "intrinsic/icon/control/services/service.h"
#include "intrinsic/icon/release/source_location.h"

/*
 * Macros for registering a Service.
 *
 * Services must be registered before they are available to Context objects.
 * Register a service with:
 *
 *     INTRINSIC_REGISTER_SERVICE(MyServiceImpl, MyService);
 *
 * Registering a service associates together (1) a name, (2) an implementation
 * class, and (3) an interface class. The name and interface class must each be
 * globally unique across all registered services. The
 * INTRINSIC_REGISTER_SERVICE macro uses the first argument, stringified, as the
 * name, e.g. "MyServiceImpl" in the example above.
 *
 * The interface type is used to lookup a Service at runtime, e.g.:
 *
 *    MyService* svc = ctx->GetServiceOrDie<MyService>();
 */

namespace intrinsic::icon {

using ServiceRegistry =
    registry::Registry<std::pair<std::string, std::string>,
                       std::unique_ptr<Service>(),
                       registry::MissingKeyPolicy::exception,
                       absl::Hash<std::pair<std::string, std::string>>>;

// Registers a Service with a custom name.
//
// This function associates together (1) a name (used in config files), (2) an
// implementation class, and (3) an interface class. The name and implementation
// class must each be globally unique across all registered services.
template <class ServiceClass, class InterfaceClass>
bool RegisterService(const std::string& name,
                     const intrinsic::SourceLocation& loc) {
  return ::intrinsic::icon::ServiceRegistry::Register(
      {name, intrinsic::icon::Service::Key<InterfaceClass>()},
      [name]() { return std::make_unique<ServiceClass>(); });
}

}  // namespace intrinsic::icon

#define __INTRINSIC_REGISTERER_CAT_INNER(x, y) x##y
#define __INTRINSIC_REGISTERER_CAT(x, y) __INTRINSIC_REGISTERER_CAT_INNER(x, y)
#define __INTRINSIC_REGISTERER_NAME() \
  __INTRINSIC_REGISTERER_CAT(registerer_, __LINE__)

// INTRINSIC_REGISTER_SERVICE registers the Service implementation using its
// class name.
#define INTRINSIC_REGISTER_SERVICE(ImplClass, InterfaceClass)        \
  const bool __INTRINSIC_REGISTERER_NAME() =                         \
      ::intrinsic::icon::RegisterService<ImplClass, InterfaceClass>( \
          #ImplClass, INTRINSIC_LOC);

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_SERVICE_REGISTRATION_H_
