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

#include "intrinsic/icon/control/parts/part_property_registry.h"

#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/common/part_properties.h"

namespace intrinsic::icon {

absl::StatusOr<PartPropertyId> PartPropertyRegistry::RegisterBoolProperty(
    absl::string_view name, bool initial_value) {
  // PartPropertyIds correspond to the property's index in the property data
  // array.
  PartPropertyId id(initial_data_.size());
  if (!property_names_.insert(std::string(name)).second) {
    return absl::AlreadyExistsError(absl::StrCat("Already registered: ", name));
  }
  initial_data_.push_back(PartPropertyInitialData{
      .name = std::string(name), .initial_value = initial_value});
  return id;
}

absl::StatusOr<PartPropertyId> PartPropertyRegistry::RegisterDoubleProperty(
    absl::string_view name, double initial_value) {
  PartPropertyId id(initial_data_.size());
  if (!property_names_.insert(std::string(name)).second) {
    return absl::AlreadyExistsError(absl::StrCat("Already registered: ", name));
  }
  initial_data_.push_back(PartPropertyInitialData{
      .name = std::string(name), .initial_value = initial_value});
  return id;
}

}  // namespace intrinsic::icon
