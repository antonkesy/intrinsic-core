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

#include "intrinsic/icon/control/services/service_collection.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/services/service.h"

namespace intrinsic::icon {

absl::StatusOr<ServiceCollection>
ServiceCollection::CreateFromServiceListNoInitialize(
    std::vector<std::unique_ptr<Service>> services) {
  absl::flat_hash_map<std::string, std::unique_ptr<Service>> service_by_key;
  // Check for duplicate keys before transferring ownership.
  for (const std::unique_ptr<Service>& service : services) {
    const std::string key = service->InterfaceKey();
    if (service_by_key.contains(key)) {
      return absl::AlreadyExistsError(
          absl::StrCat("Duplicate service key \"", key,
                       "\" in ServiceCollection::CreateFromServiceList"));
    }
    service_by_key[key] = nullptr;
  }
  // Transfer ownership of each Service from input list to hash map.
  for (std::unique_ptr<Service>& service : services) {
    service_by_key[service->InterfaceKey()] = std::move(service);
  }
  return ServiceCollection(std::move(service_by_key));
}

absl::Status ServiceCollection::AddInitializedService(
    std::unique_ptr<Service>&& service) {
  const std::string key = service->InterfaceKey();
  if (services_.contains(key)) {
    return absl::AlreadyExistsError(
        absl::StrCat("Duplicate service key \"", key,
                     "\" in ServiceCollection::AddInitializedService"));
  }
  services_[key] = std::move(service);
  return absl::OkStatus();
}

Service* ServiceCollection::GetService(absl::string_view key) const {
  if (!services_.contains(key)) {
    return nullptr;
  }
  return services_.at(key).get();
}

}  // namespace intrinsic::icon
