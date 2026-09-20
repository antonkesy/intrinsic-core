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

#ifndef INTRINSIC_ICON_CONTROL_STATE_VARIABLE_SELECTION_H_
#define INTRINSIC_ICON_CONTROL_STATE_VARIABLE_SELECTION_H_

#include <cstddef>
#include <optional>
#include <string>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/common/state_variable_path_util.h"
#include "intrinsic/icon/control/state_variable_selection_types.h"

namespace intrinsic::icon {

// Generates for a given state variable path string a real-time capable
// std::function that selects the desired entry from a PublishOutput instance.
//
// How to extend the class to access a new robot system state field with a
// state variable path:
//    The parsing of a state variable path begins with the first node to see if
//    it belongs to a part specific or safety field and then passes control
//    to dedicated functions.
//
//    State variable paths have the pattern <part_name>.<part_type>.<nodes...>.
//    The number of nodes is variable and can be chosen as needed for a specific
//    field. The part_name is used to find the corresponding part instance where
//    as the part_type is used to find the matching parsing function. The
//    part_name must be user-configurable in the user API functions (located at
//    intrinsic/icon/cc_client/state_variable_path.h). Non-part
//    status paths have the pattern <category_name>.<nodes...>, e.g. safety.
//
//    To add a new field,
//      - find the corresponding (part)-function in
//      intrinsic/icon/control/state_variable_selection.cc
//      - add a new case for the new field
//      - return a lambda that selects the field value from PublishOutput in a
//        real-time compatible manner.
//    Simple operations on the values are acceptable, e.g. norm(). Avoid complex
//    operations. Use util functions to avoid repeating boiler plate code such
//    as array index checking (e.g. SelectCheckedValue() and
//    CreateSelectionFunctionForVector()).
//    In general follow the pattern used by the existing cases.
//
//    Place any string literals used for field identification as constants in
//    intrinsic/icon/common/state_variable_path_constants.h, so that
//    they can also be used in path generator functions. Make sure to copy all
//    needed information (e.g. joint index) into the created lambda and *do not*
//    capture references in the lambda. The lambda can outlive anything that is
//    accessible from this RobotSystemStateFieldSelector instance. Though, do
//    not copy in large objects into the lambda. Keep it simple and lean.
//
//    After adding the new field to the `RobotSystemStateFieldSelector`, it
//    needs to be made publicly available in the python, C++ and golang user
//    API: Add a suitable path generator function in
//    intrinsic/icon/cc_client/state_variable_path.h and
//    intrinsic/icon/python/state_variable_path.py following the
//    pattern found in those files. Do not forget to extend the tests in
//    intrinsic/icon/python/state_variable_path_test.py that check
//    that both API functions (C++ and Python) return the same value.
class RobotSystemStateFieldSelector {
 public:
  // Creates a RobotSystemStateFieldSelector instance.
  // RobotSystemStateFieldSelector keeps references to both
  // `part_name_to_index_map` and `robot_status` for its entire lifetime. Thus,
  // the passed in objects *must* outlive the RobotSystemStateFieldSelector
  // instance! `robot_status` should be the most recent since it is used for
  // evaluation of the created selection functions.
  RobotSystemStateFieldSelector(
      const absl::flat_hash_map<std::string, size_t>& part_name_to_index_map
          ABSL_ATTRIBUTE_LIFETIME_BOUND,
      const StateVariableFieldSelectionData& selection_data
          ABSL_ATTRIBUTE_LIFETIME_BOUND)
      : part_name_to_index_map_(part_name_to_index_map),
        selection_data_(selection_data) {}

  // Creates a robot system field selection function object based on
  // `state_variable_path`.
  // Returns a std::function that can be executed in a real-time
  // thread, but the function object itself might have allocated memory on the
  // heap.  Thus, make sure the lambda is allocated in a non-rt thread and
  // only its pointer is given to the RT thread.
  // Returns an error status on any problem with the `state_variable_path`,
  // `part_name_to_index_map` or the `robot_status` an error with specific
  // information in the message is returned.
  // Sets `required_part_index` if the field selection function needs read
  // access to current part status for that part.
  absl::StatusOr<StateVariableFieldSelectionFunction>
  CreateRobotSystemStateFieldSelector(
      absl::string_view state_variable_path,
      std::optional<size_t>& required_part_index);

 private:
  // Selects the functions for parts (in contrast to non-part functions such as
  // safety).
  absl::StatusOr<StateVariableFieldSelectionFunction> CreateFunctionForParts(
      absl::string_view part_name, absl::string_view part_type_name,
      size_t part_index, absl::Span<StateVariablePathNode> part_field_nodes);

  const absl::flat_hash_map<std::string, size_t>& part_name_to_index_map_;
  const StateVariableFieldSelectionData& selection_data_;
};
}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_STATE_VARIABLE_SELECTION_H_
