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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_CONDITION_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_CONDITION_H_

#include <stdbool.h>

#include <cstddef>
#include <optional>
#include <string>
#include <variant>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/state_variable_selection_types.h"
#include "intrinsic/icon/proto/v1/condition_types.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

inline constexpr size_t kMaxConditionElements = 64;
inline constexpr size_t kMaxClauseElements = 8;
const size_t kMaxStateVariableNameLength = 64;

// An AllOfClause evaluates to 'true' if all referenced elements evaluate to
// 'true'. An empty AllOf evaluates to 'true'.
struct AllOfClause {
  intrinsic::FixedVector<size_t, kMaxClauseElements> element_indices;
  friend bool operator==(const AllOfClause& lhs, const AllOfClause& rhs);
  friend bool operator!=(const AllOfClause& lhs, const AllOfClause& rhs);
};

// An AnyOfClause evaluates to 'true' if at least one of the referenced elements
// evaluates to 'true'. An empty AnyOfClause evaluates to 'false'.
struct AnyOfClause {
  intrinsic::FixedVector<size_t, kMaxClauseElements> element_indices;
  friend bool operator==(const AnyOfClause& lhs, const AnyOfClause& rhs);
  friend bool operator!=(const AnyOfClause& lhs, const AnyOfClause& rhs);
};

// A NotClause evaluates to 'true' if the referenced element evaluates to
// 'false'.
struct NotClause {
  size_t element_index;
  friend bool operator==(const NotClause& lhs, const NotClause& rhs);
  friend bool operator!=(const NotClause& lhs, const NotClause& rhs);
};

// A RealtimeComparison is the corresponding struct of the Comparison class
// (intrinsic/icon/cc_client/condition.h) that only has members that
// can be accessed and called in a realtime thread. This struct is *NOT* safe to
// copy, create non-empty, or destroy in realtime due to
// StateVariableFieldSelectionFunction!
struct RealtimeComparison {
  RealtimeComparison() INTRINSIC_CHECK_REALTIME_SAFE = default;
  RealtimeComparison(
      const std::variant<FixedString<kMaxStateVariableNameLength>,
                         StateVariableFieldSelectionFunction>& operand,
      std::optional<size_t> required_part_index,
      intrinsic_proto::icon::v1::Comparison::OpEnum operation,
      ComparisonValue value,
      double max_abs_error = kDefaultMaxAbsError) INTRINSIC_NON_REALTIME_ONLY
      : operand(operand),
        required_part_index(required_part_index),
        operation(operation),
        value(value),
        max_abs_error(max_abs_error) {}
  RealtimeComparison(const RealtimeComparison& other)
      INTRINSIC_NON_REALTIME_ONLY = default;
  ~RealtimeComparison() INTRINSIC_NON_REALTIME_ONLY = default;
  RealtimeComparison& operator=(const RealtimeComparison& other)
      INTRINSIC_NON_REALTIME_ONLY = default;
  RealtimeComparison(RealtimeComparison&& other)
      INTRINSIC_CHECK_REALTIME_SAFE = default;
  RealtimeComparison& operator=(RealtimeComparison&& other)
      INTRINSIC_CHECK_REALTIME_SAFE = default;

  // Name of state variable to use or a state variable path selection
  // function used as first operand.
  std::variant<FixedString<kMaxStateVariableNameLength>,
               StateVariableFieldSelectionFunction>
      operand;
  // Part index that needs fresh status in AggregatedRobotStatus to evaluate
  // this condition.
  std::optional<size_t> required_part_index;
  // Comparison operation to perform.
  intrinsic_proto::icon::v1::Comparison::OpEnum operation;
  // Value to use as second operand.
  ComparisonValue value;
  // Epsilon to use for approx comparisons.
  double max_abs_error = kDefaultMaxAbsError;
};

// Create a RealtimeComparison from a `comparison`. Uses
// `part_name_to_realtime_index` and `robot_status` to create a state variable
// path selection function, if the `state_variable_name` of `comparison`
// contains a state variable path.
absl::StatusOr<RealtimeComparison> ToRealtimeComparison(
    const Comparison& comparison,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_realtime_index,
    const AggregatedRobotStatus& robot_status);

// ConditionElements are nodes in the RealtimeCondition tree. A Comparison is a
// leaf node.
using ConditionElement =
    std::variant<AnyOfClause, AllOfClause, NotClause, RealtimeComparison>;

// A RealtimeCondition is a flattened tree of ConditionElements stored as
// indexed elements.
struct RealtimeCondition {
  // elements[0] is the top level condition
  intrinsic::FixedVector<ConditionElement, kMaxConditionElements> elements;

  // Parts that need fresh status in AggregatedRobotStatus to evaluate
  // this condition.
  intrinsic::FixedVector<bool, kMaxRealtimeParts> required_parts_by_index;
};

// Creates a RealtimeCondition from a Condition Proto
absl::StatusOr<RealtimeCondition> RealtimeConditionFromProto(
    const intrinsic_proto::icon::v1::Condition& proto,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_realtime_index,
    const AggregatedRobotStatus& robot_status);

// The operator== definitions are required to make ConditionElement comparable,
// which is helpful in tests.

inline bool operator==(const AllOfClause& lhs, const AllOfClause& rhs) {
  return lhs.element_indices == rhs.element_indices;
}

inline bool operator!=(const AllOfClause& lhs, const AllOfClause& rhs) {
  return !(lhs == rhs);
}

inline bool operator==(const AnyOfClause& lhs, const AnyOfClause& rhs) {
  return lhs.element_indices == rhs.element_indices;
}

inline bool operator!=(const AnyOfClause& lhs, const AnyOfClause& rhs) {
  return !(lhs == rhs);
}

inline bool operator==(const NotClause& lhs, const NotClause& rhs) {
  return lhs.element_index == rhs.element_index;
}

inline bool operator!=(const NotClause& lhs, const NotClause& rhs) {
  return !(lhs == rhs);
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_CONDITION_H_
