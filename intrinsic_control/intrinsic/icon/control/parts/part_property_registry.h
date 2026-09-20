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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_PART_PROPERTY_REGISTRY_H_
#define INTRINSIC_ICON_CONTROL_PARTS_PART_PROPERTY_REGISTRY_H_

#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/common/part_properties.h"

namespace intrinsic::icon {

// This is used at initialization time. It holds name and default value (and
// implicitly, the type) of a single property.
struct PartPropertyInitialData {
  std::string name;
  PartPropertyValue initial_value;
};

// This class allows realtime part factories to register properties by string
// name.
// Part factories get a PartPropertyId in return, which they can use to access
// those properties in O(1) during cyclic realtime operation (where string
// comparisons are potentially dangerous).
class PartPropertyRegistry {
 public:
  // Creates a PartPropertyRegistry.
  // `initial_data` should be empty when it is passed to this constructor. After
  // the part factory has finished, `initial_data` holds information about all
  // of its properties, including their initial values.
  explicit PartPropertyRegistry(std::vector<PartPropertyInitialData>&
                                    initial_data ABSL_ATTRIBUTE_LIFETIME_BOUND)
      : initial_data_(initial_data) {}

  // Registers a boolean part property with the given name and initial value.
  //
  // Returns AlreadyExistsError if a property with the same name already exists
  // (even if it has a different type).
  absl::StatusOr<PartPropertyId> RegisterBoolProperty(
      absl::string_view name, bool initial_value = false);

  // Registers a double part property with the given name and initial value.
  //
  // Returns AlreadyExistsError if a property with the same name already exists
  // (even if it has a different type).
  absl::StatusOr<PartPropertyId> RegisterDoubleProperty(
      absl::string_view name, double initial_value = 0.0);

 private:
  std::vector<PartPropertyInitialData>& initial_data_;
  // Used to prevent duplicate names
  absl::flat_hash_set<std::string> property_names_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_PART_PROPERTY_REGISTRY_H_
