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

#include "intrinsic/icon/control/realtime_condition.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/state_variable_path_constants.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/state_variable_selection.h"
#include "intrinsic/icon/control/state_variable_selection_types.h"
#include "intrinsic/icon/proto/v1/condition_types.pb.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
namespace {

// Safely reserves an element in elements and returns its index.
absl::StatusOr<size_t> ReserveElementIndex(
    intrinsic::FixedVector<ConditionElement, kMaxConditionElements>& elements) {
  if (elements.size() >= elements.capacity()) {
    return absl::OutOfRangeError(absl::StrFormat(
        "RealtimeCondition exceeds maximum capacity of %d elements.",
        kMaxConditionElements));
  }
  size_t index = elements.size();
  elements.emplace_back();
  return index;
}

absl::StatusOr<size_t> InsertConditionFromProto(
    const intrinsic_proto::icon::v1::Condition& proto,
    intrinsic::FixedVector<ConditionElement, kMaxConditionElements>& elements,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_realtime_index,
    const AggregatedRobotStatus& robot_status);

absl::StatusOr<size_t> InsertConjunctionConditionFromProto(
    const intrinsic_proto::icon::v1::ConjunctionCondition& proto,
    intrinsic::FixedVector<ConditionElement, kMaxConditionElements>& elements,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_realtime_index,
    const AggregatedRobotStatus& robot_status) {
  absl::string_view clause_name;
  switch (proto.operation()) {
    case (intrinsic_proto::icon::v1::ConjunctionCondition::ALL_OF):
      clause_name = "AllOfClause";
      break;
    case (intrinsic_proto::icon::v1::ConjunctionCondition::ANY_OF):
      clause_name = "AnyOfClause";
      break;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported ConjunctionCondition operation type: ",
                       static_cast<int64_t>(proto.operation())));
  }

  if (proto.conditions_size() > kMaxClauseElements) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Too many elements for %s: elements[%d] > capacity[%d].", clause_name,
        proto.conditions_size(), kMaxClauseElements));
  }

  INTR_ASSIGN_OR_RETURN(size_t element_index, ReserveElementIndex(elements));

  intrinsic::FixedVector<size_t, kMaxClauseElements> element_indices;
  for (const auto& c : proto.conditions()) {
    INTR_ASSIGN_OR_RETURN(
        size_t child_index,
        InsertConditionFromProto(c, elements, part_name_to_realtime_index,
                                 robot_status));
    element_indices.emplace_back(child_index);
  }

  switch (proto.operation()) {
    case (intrinsic_proto::icon::v1::ConjunctionCondition::ALL_OF):
      elements[element_index] = AllOfClause{std::move(element_indices)};
      break;
    case (intrinsic_proto::icon::v1::ConjunctionCondition::ANY_OF):
      elements[element_index] = AnyOfClause{std::move(element_indices)};
      break;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported ConjunctionCondition operation type: ",
                       static_cast<int64_t>(proto.operation())));
  }
  return element_index;
}

absl::StatusOr<size_t> InsertNegatedConditionFromProto(
    const intrinsic_proto::icon::v1::NegatedCondition& proto,
    intrinsic::FixedVector<ConditionElement, kMaxConditionElements>& elements,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_realtime_index,
    const AggregatedRobotStatus& robot_status) {
  INTR_ASSIGN_OR_RETURN(size_t element_index, ReserveElementIndex(elements));
  INTR_ASSIGN_OR_RETURN(
      size_t child_index,
      InsertConditionFromProto(proto.condition(), elements,
                               part_name_to_realtime_index, robot_status));
  elements[element_index] = NotClause{child_index};
  return element_index;
}

absl::StatusOr<size_t> InsertConditionFromProto(
    const intrinsic_proto::icon::v1::Condition& proto,
    intrinsic::FixedVector<ConditionElement, kMaxConditionElements>& elements,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_realtime_index,
    const AggregatedRobotStatus& robot_status) {
  switch (proto.condition_case()) {
    case (intrinsic_proto::icon::v1::Condition::kComparison): {
      INTR_ASSIGN_OR_RETURN(size_t element_index,
                            ReserveElementIndex(elements));
      INTR_ASSIGN_OR_RETURN(auto comparison, FromProto(proto.comparison()));
      INTR_ASSIGN_OR_RETURN(
          elements[element_index],
          ToRealtimeComparison(comparison, part_name_to_realtime_index,
                               robot_status));
      return element_index;
    }
    case (intrinsic_proto::icon::v1::Condition::kConjunctionCondition):
      return InsertConjunctionConditionFromProto(
          proto.conjunction_condition(), elements, part_name_to_realtime_index,
          robot_status);
    case (intrinsic_proto::icon::v1::Condition::kNegatedCondition):
      return InsertNegatedConditionFromProto(
          proto.negated_condition(), elements, part_name_to_realtime_index,
          robot_status);
    case (intrinsic_proto::icon::v1::Condition::CONDITION_NOT_SET):
      return absl::InvalidArgumentError("Condition proto is empty / not set.");
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported condition type: ",
                       static_cast<int64_t>(proto.condition_case())));
  }
}

}  // namespace

// Recursively unpacks the proto.
absl::StatusOr<RealtimeCondition> RealtimeConditionFromProto(
    const intrinsic_proto::icon::v1::Condition& proto,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_realtime_index,
    const AggregatedRobotStatus& robot_status) {
  RealtimeCondition condition;

  INTR_RETURN_IF_ERROR(InsertConditionFromProto(proto, condition.elements,
                                                part_name_to_realtime_index,
                                                robot_status)
                           .status());

  condition.required_parts_by_index.resize(kMaxRealtimeParts, false);
  for (const auto& element : condition.elements) {
    if (const RealtimeComparison* comparison =
            std::get_if<RealtimeComparison>(&element);
        comparison != nullptr) {
      if (!comparison->required_part_index.has_value()) continue;
      if (comparison->required_part_index.value() >= kMaxRealtimeParts) {
        return absl::OutOfRangeError(
            absl::StrFormat("Too large part index for RealtimeCondition: "
                            "required_parts_by_index[%d] "
                            ">= kMaxRealtimeParts",
                            comparison->required_part_index.value()));
      }
      condition
          .required_parts_by_index[comparison->required_part_index.value()] =
          true;
    }
  }
  return condition;
}

absl::StatusOr<RealtimeComparison> ToRealtimeComparison(
    const Comparison& comparison,
    const absl::flat_hash_map<std::string, size_t>& part_name_to_realtime_index,
    const AggregatedRobotStatus& robot_status) {
  StateVariableFieldSelectionData selection_data = {.robot_status =
                                                        robot_status};
  RobotSystemStateFieldSelector selector(part_name_to_realtime_index,
                                         selection_data);
  std::variant<FixedString<kMaxStateVariableNameLength>,
               StateVariableFieldSelectionFunction>
      operand;
  std::optional<size_t> required_part_index;

  // A state variable name must start with the prefix to be a state variable
  // path
  if (absl::StartsWith(comparison.state_variable_name(),
                       kStateVariablePathPrefix)) {
    absl::StatusOr<StateVariableFieldSelectionFunction> selection_function =
        selector.CreateRobotSystemStateFieldSelector(
            comparison.state_variable_name(), required_part_index);
    if (!selection_function.ok()) {
      return absl::Status(
          selection_function.status().code(),
          absl::StrCat("Creating part status field function failed for path '",
                       comparison.state_variable_name(), "' with ",
                       selection_function.status().code(),
                       " error: ", selection_function.status().message()));
    }
    operand = std::move(*selection_function);
  } else {
    if (comparison.state_variable_name().size() > kMaxStateVariableNameLength) {
      return absl::InvalidArgumentError(absl::StrCat(
          "The action state variable name '", comparison.state_variable_name(),
          "' is too long. Max size: ", kMaxStateVariableNameLength,
          " actual size: ", comparison.state_variable_name().size()));
    }
    operand = comparison.state_variable_name();
  }

  return RealtimeComparison(std::move(operand), required_part_index,
                            comparison.operation(), comparison.value(),
                            comparison.max_abs_error());
}
}  // namespace intrinsic::icon
