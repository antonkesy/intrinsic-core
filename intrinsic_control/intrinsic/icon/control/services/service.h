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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_SERVICE_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_SERVICE_H_

#include <string>
#include <tuple>
#include <typeinfo>
#include <utility>
#include <vector>

#include "absl/meta/type_traits.h"
#include "absl/utility/utility.h"
#include "intrinsic/util/demangle.h"

// Service are registered by an interface name at static initialization.
// Service implementations can be swapped easily. They can depend on other
// services.
//
// Services are managed by a ServiceCollection object. See
// service_registration.h for details about service registration
// and lookup.
//
// Each service has an interface and an implementation. The interface is any
// (pure virtual) abstract base class.
//
//   // example_service.h:
//   class ExampleService {
//    public:
//     virtual void DoSomething() = 0;
//   };
//
// The implementation must inherit from
// ServiceImplBase<ImplClass, ItfClass, DependantServices...>. For example:
//
//   // example_service_impl.h:
//   class ExampleServiceImpl : public ServiceImplBase<ExampleServiceImpl,
//                                                     ExampleService,
//                                                     DependantOtherService>
//                                                     {
//    public:
//     util::Status Initialize(const ConfigNode* config,
//                             DependantOtherService* other);
//     void DoSomething() override;
//   };
//
//   // example_service.cc:
//   INTRINSIC_REGISTER_SERVICE(ExampleServiceImpl, ExampleService);
//
// An implementation may depend on other Services. These dependencies are
// inferred from the template parameters for ServiceImpleBase.

namespace intrinsic::icon {

// Core abstract base class for service implementations.
class Service {
 public:
  struct InitParameters {
    std::string server_name;
    double control_frequency_hz;
    // ID of the resource that ICON is associated with. Empty if not running as
    // a resource.
    std::string resource_id;
  };

  Service() = default;
  virtual ~Service() = default;

  // This is how we determine the key for a service. It is not intended to be
  // used outside the framework code.
  template <typename ServiceType>
  static std::string Key();

  // Gets the name of a Service class from an instance of that Service.
  // This is provided by ServiceImplBase.
  virtual std::string InterfaceKey() = 0;

  // Obtain the service's interface.
  virtual void* GetInterface() = 0;
};

namespace service_details {}  // namespace service_details

// Templated base class for service implementations. The template parameters
// determine the implementation type, interface type, and list of dependencies.
// `Dependencies` are the interface types of the other services interfaces that
// this implementation depends on.
template <typename ServiceType, typename InterfaceType,
          typename... Dependencies>
class ServiceImplBase : public Service, public InterfaceType {
 public:
  using ServiceInterfaceType = InterfaceType;
  using DependencyPtrTuple = std::tuple<absl::add_pointer_t<Dependencies>...>;

  std::string InterfaceKey() override {
    return Service::Key<ServiceInterfaceType>();
  }

  void* GetInterface() final { return static_cast<InterfaceType*>(this); }
};

template <typename ServiceType>
std::string Service::Key() {
  return intrinsic::Demangle<ServiceType>();
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_SERVICE_H_
