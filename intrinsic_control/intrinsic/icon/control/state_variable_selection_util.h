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

// This file contains utility functions needed for the state variable selection
// functionality.
#ifndef INTRINSIC_ICON_CONTROL_STATE_VARIABLE_SELECTION_UTIL_H_
#define INTRINSIC_ICON_CONTROL_STATE_VARIABLE_SELECTION_UTIL_H_

#include <optional>

#include "absl/strings/string_view.h"
#include "intrinsic/icon/common/state_variable_path_constants.h"
#include "intrinsic/icon/control/parts/realtime_part_status.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/state_variable_selection_types.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// Gives access to class members given a member ptr. Useful for hiding boiler
// plate code for access to member variables using template functions/lambdas.
// Example:
// struct Data{
//   int x;
// };
// Data d;
// int x = AccessClassMember(d, &Data::x);
template <typename C, typename C2, typename T>
const T& AccessClassMember(const C& cls, T C2::* member) {
  static_assert(std::is_base_of_v<C2, C>,
                "Trying to access member of an unrelated class");
  return (cls.*member);
}

// Extracts the actual type of a class member ptr. Function has no
// implementation, so can only be used with decltype, etc.
// Example:
//   struct Data{
//     int x;
//   }
//   decltype(GetTypeOfMemberPtr(&Data::x)) val; // <- val would be of type int.
template <class C, typename T>
T GetTypeOfMemberPtr(T C::* v);

// Creates a function object for a vector field in RealtimePartStatus, such as
// the sensed_position field, usable in a realtime thread. This function wraps
// all the boilerplate code for safe access to such a field.
// The nested vector field is specified by a combination of `field_ptr`
// and `subfield_ptr` since all vector fields in RealtimePartStatus are
// structured this way. `field_ptr` and `subfield_ptr` need to be of a member
// pointer type such as `&MyClass::my_member_`, which is used to access
// this value in `output` from the part_status at `part_index`. This function
// avoids the boiler plate code when accessing the `output` members since all
// members are in a specific part_status and are additionally optional.
//
// Args:
//   - `field_ptr` must point to a class member that has the std::optional<>
//     interface and has an array subtype.
//   - `subfield_ptr` must point to a class that has an array interface, i.e.
//      has a operator[int] function.
//   - `part_index` must contain the index of the part_status where the
//     requested field is available.
//   - `index` specifies the element in the subfield array that should be
//     returned.
//   - `part_name` and `field_name` are only used for error messages and should
//     contain the accessed part_name and a describing name of the accessed
//     field.
//
// Returns a std::function object with bound variables needed for
// execution. When called, the function object accesses a nested field in an
// optional field of the RealtimePartStatus and returns the value found at
// `index` as a variant.
//
// Example:
//    return CreateSelectionFunctionForVector(
//          &RealtimePartStatus::sensed_position, &JointStateP::position,
//          part_index, joint_index, part_name,
//          kSensedPositionNodeName);
template <typename OptionalFieldPtr, typename SubFieldPtr>
StateVariableFieldSelectionFunction CreateSelectionFunctionForVector(
    OptionalFieldPtr field_ptr, SubFieldPtr subfield_ptr, size_t part_index,
    size_t index, absl::string_view part_name,
    absl::string_view field_name) INTRINSIC_NON_REALTIME_ONLY {
  const FixedString<kMaxNodeNameLength> part_name_copy(part_name);
  const FixedString<kMaxNodeNameLength> field_name_copy(field_name);
  return [field_ptr, subfield_ptr, part_index, index, field_name_copy,
          part_name_copy](const StateVariableFieldSelectionData& selection_data)
             -> RealtimeStatusOr<PartStatusVariant> {
    if (part_index >= selection_data.robot_status.part_statuses.size()) {
      return OutOfRangeError(RealtimeStatus::StrCat(
          "Part index out of range: requested index: ", part_index,
          " size:", selection_data.robot_status.part_statuses.size()));
    }
    const RealtimePartStatus& status =
        selection_data.robot_status.part_statuses[part_index];
    // Generic access to a field of the `publish_output` variable by using a
    // class member pointer. The field must be an optional.
    const auto& optional_value = AccessClassMember(status, field_ptr);
    if (!optional_value.has_value()) {
      return NotFoundError(RealtimeStatus::StrCat("Field '", field_name_copy,
                                                  "' not set in part status ",
                                                  part_name_copy, "!"));
    }
    // Access a subfield of the accessed field above. The subfield must be
    // an array type with []-operator and size() function.
    const auto& field_value =
        AccessClassMember(optional_value.value(), subfield_ptr);
    if (index >= field_value.size()) {
      return OutOfRangeError(RealtimeStatus::StrCat(
          "Field index out of range: requested index: ", index,
          " size: ", field_value.size()));
    }
    return PartStatusVariant(field_value[index]);
  };
}

// Selects a value from the field in `robot_status` given in the class member
// pointer `field_ptr` from the part_status at `part_index`.
// This function avoids the boiler plate code when accessing the
// `robot_status` members since all members are in a specific part_status and
// are optional.
//
// `field_ptr` must point to a class member that has the std::optional<>
// interface, e.g. `&MyClass::my_member_`.
// `part_name` and `field_name` are only used for error messages.
//
// The function returns a RealtimeStatusOr<pointer> to the field
// referenced in `field_ptr`. The pointer is of the type of the field referenced
// in `field_ptr`. The returned pointer is valid as long as `robot_status` is
// unchanged and valid. The function returns an error when the `part_index` is
// out of range or if the field is not set.
//
// Example:
//    INTRINSIC_RT_ASSIGN_OR_RETURN(const Twist* twist,
//            SelectCheckedValue(
//                robot_status, &RealtimePartStatus::base_twist_tip_sensed, 1,
//                "arm",
//                "translational twist magnitude"));
//    double magnitude = twist->head<3>().norm();
template <typename OptionalFieldPtr>
auto SelectCheckedValue(const AggregatedRobotStatus& robot_status,
                        OptionalFieldPtr field_ptr, size_t part_index,
                        absl::string_view part_name,
                        absl::string_view field_name)
    -> RealtimeStatusOr<
        typename decltype(GetTypeOfMemberPtr(field_ptr))::value_type const*> {
  using ValueType =
      typename decltype(GetTypeOfMemberPtr(field_ptr))::value_type;
  if (part_index >= robot_status.part_statuses.size()) {
    return OutOfRangeError(RealtimeStatus::StrCat(
        "Part index out of range: requested index: ", part_index,
        " size:", robot_status.part_statuses.size()));
  }
  const RealtimePartStatus& status = robot_status.part_statuses[part_index];
  const std::optional<ValueType>& optional_value =
      AccessClassMember(status, field_ptr);
  if (!optional_value.has_value()) {
    return NotFoundError(RealtimeStatus::StrCat(
        "Field '", field_name, "' not set in part status '", part_name, "'!"));
  }
  return &optional_value.value();
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_STATE_VARIABLE_SELECTION_UTIL_H_
