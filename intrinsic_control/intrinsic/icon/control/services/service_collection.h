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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_SERVICE_COLLECTION_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_SERVICE_COLLECTION_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/services/service.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// A ServiceCollection owns a collection of Services and is used to obtain
// service handles.
class ServiceCollection {
 public:
  // Takes ownership of a Service object and makes it available.
  // It is expected that the service has been initialized prior to
  // calling this.
  absl::Status AddInitializedService(std::unique_ptr<Service>&& service);

  // Constructs a ServiceCollection by taking ownership of a list of Service
  // objects. It is expected that all services have been initialized prior to
  // calling this.
  //
  // Returns AlreadyExistsError if the list contains duplicated service keys.
  ABSL_DEPRECATED(
      "Use AddInitializedService or CreateFromInitializedServices instead.")
  static absl::StatusOr<ServiceCollection> CreateFromServiceListNoInitialize(
      std::vector<std::unique_ptr<Service>> services);

  // Constructs a ServiceCollection by taking ownership of a list of Service
  // objects. It is expected that all services have been initialized prior to
  // calling this.
  //
  // Works around a C++ limitation that containers and initializer_list cannot
  // create inplace with move-only types.
  static absl::StatusOr<ServiceCollection> CreateFromInitializedServices(
      std::unique_ptr<Service>&& service) {
    ServiceCollection collection;
    INTR_RETURN_IF_ERROR(collection.AddInitializedService(std::move(service)));
    return collection;
  }
  template <typename... Args>
  static absl::StatusOr<ServiceCollection> CreateFromInitializedServices(
      std::unique_ptr<Service>&& service, Args&&... args) {
    INTR_ASSIGN_OR_RETURN(auto collection, CreateFromInitializedServices(
                                               std::forward<Args>(args)...));
    INTR_RETURN_IF_ERROR(collection.AddInitializedService(std::move(service)));
    return collection;
  }

  // Obtains a service interface. T is the interface type. Returns
  // `nullptr` if the Service is not available. The returned pointer is valid
  // for the lifetime of this ServiceCollection.
  //
  // Example:
  //  ExampleService* svc = services->GetServiceOrNull<ExampleService>();
  template <class T>
  T* GetServiceOrNull() const;

  // Creates an empty ServiceCollection with no Services.
  ServiceCollection() = default;
  // Disable copy but not move.
  ServiceCollection(const ServiceCollection& other) = delete;
  ServiceCollection& operator=(const ServiceCollection& other) = delete;
  ServiceCollection(ServiceCollection&& other) = default;
  ServiceCollection& operator=(ServiceCollection&& other) = default;

 private:
  // Construct from map of Service objects, keyed by service Service key, taking
  // ownership of the passed-in services.
  explicit ServiceCollection(
      absl::flat_hash_map<std::string, std::unique_ptr<Service>> services)
      : services_(std::move(services)) {}

  // Lookup a Service owned by this ServiceCollection given its internal key.
  intrinsic::icon::Service* GetService(absl::string_view key) const;

  // Map holding Services owned by this ServiceCollection, keyed by Service key
  // (e.g. Service::Key<T>()).
  absl::flat_hash_map<std::string, std::unique_ptr<Service>> services_;
};

template <class T>
T* ServiceCollection::GetServiceOrNull() const {
  Service* svc = GetService(Service::Key<T>());
  if (!svc) {
    return nullptr;
  }
  return static_cast<T*>(svc->GetInterface());
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_SERVICE_COLLECTION_H_
