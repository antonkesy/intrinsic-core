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

#include "intrinsic/icon/control/realtime_actions_reactions.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <type_traits>
#include <variant>

#include "absl/types/span.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_condition.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/state_variable_selection_types.h"
#include "intrinsic/icon/proto/v1/condition_types.pb.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {
namespace {

RealtimeStatus CheckDoubleInValidInt64Range(const double value) {
  // We allow comparisons between int and double, but we need to do a range
  // check since not all double values above kMaxValidInt64AsDouble can be
  // converted to int64.
  if (std::abs(value) > kMaxValidInt64AsDouble) {
    return OutOfRangeError(RealtimeStatus::StrCat(
        "number ", value,
        " too large for int64_t. Max value: ", kMaxValidInt64AsDouble));
  }

  return OkStatus();
}

RealtimeStatusOr<double> SafeInt64ToDouble(const int64_t value) {
  // We allow comparisons between int and double, but we need to do a range
  // check since not all double values above kMaxValidInt64AsDouble can be
  // converted to int64.
  constexpr int64_t maxValidInt64AsDouble =
      static_cast<int64_t>(kMaxValidInt64AsDouble);
  if (std::abs(value) > maxValidInt64AsDouble) {
    return OutOfRangeError(RealtimeStatus::StrCat(
        "cannot safely convert ", value,
        " from int64_t to double. Max value: ", kMaxValidInt64AsDouble));
  }
  return static_cast<double>(value);
}

struct VariableComparisonVisitor {
  // `comparison` must outlive the visitor since it is stored by reference.
  explicit VariableComparisonVisitor(const RealtimeComparison& comparison)
      : comparison(comparison) {}

  template <typename T1, typename T2>
  struct assert_false : std::false_type {};
  template <typename T1, typename T2>
  RealtimeStatusOr<bool> operator()(const T1 v, const T2 other_value) {
    static_assert(
        assert_false<T1, T2>::value,
        "There is no implementation for this comparison overload. Please "
        "implement it.");
    return OkStatus();
  }

  const RealtimeComparison& comparison;
};

template <>
RealtimeStatusOr<bool> VariableComparisonVisitor::operator()(
    const bool v, const bool other_value) {
  switch (comparison.operation) {
    case intrinsic_proto::icon::v1::Comparison::EQUAL:
      return v == other_value;
    case intrinsic_proto::icon::v1::Comparison::NOT_EQUAL:
      return v != other_value;
    default:
      return InvalidArgumentError("comparison operator not supported");
  }
}

template <>
RealtimeStatusOr<bool> VariableComparisonVisitor::operator()(
    const bool v, const double other_value) {
  return InvalidArgumentError("bool and double cannot be compared");
}

template <>
RealtimeStatusOr<bool> VariableComparisonVisitor::operator()(
    const double v, const bool other_value) {
  return InvalidArgumentError("double and bool cannot be compared");
}

template <>
RealtimeStatusOr<bool> VariableComparisonVisitor::operator()(
    const bool v, const int64_t other_value) {
  return InvalidArgumentError("bool and int64 cannot be compared");
}

template <>
RealtimeStatusOr<bool> VariableComparisonVisitor::operator()(
    const int64_t v, const bool other_value) {
  return InvalidArgumentError("int64 and bool cannot be compared");
}

template <>
RealtimeStatusOr<bool> VariableComparisonVisitor::operator()(
    const double v, const double other_value) {
  switch (comparison.operation) {
      // EQUAL and NOT_EQUAL are not supported for float values
    case intrinsic_proto::icon::v1::Comparison::APPROX_EQUAL:
      return std::abs(v - other_value) <= comparison.max_abs_error;
    case intrinsic_proto::icon::v1::Comparison::APPROX_NOT_EQUAL:
      return std::abs(v - other_value) > comparison.max_abs_error;
    case intrinsic_proto::icon::v1::Comparison::LESS_THAN_OR_EQUAL:
      return v <= other_value;
    case intrinsic_proto::icon::v1::Comparison::LESS_THAN:
      return v < other_value;
    case intrinsic_proto::icon::v1::Comparison::GREATER_THAN_OR_EQUAL:
      return v >= other_value;
    case intrinsic_proto::icon::v1::Comparison::GREATER_THAN:
      return v > other_value;
    default:
      return InvalidArgumentError("comparison operator not supported");
  }
}

template <>
RealtimeStatusOr<bool> VariableComparisonVisitor::operator()(
    const double v, const int64_t other_value_int64) {
  if (comparison.operation == intrinsic_proto::icon::v1::Comparison::EQUAL ||
      comparison.operation ==
          intrinsic_proto::icon::v1::Comparison::NOT_EQUAL) {
    return InvalidArgumentError(
        "EQUAL/NOT_EQUAL operation not possible between integer and "
        "double.");
  }
  INTRINSIC_RT_RETURN_IF_ERROR(CheckDoubleInValidInt64Range(v));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double other_value,
                                SafeInt64ToDouble(other_value_int64));
  return (*this)(v, other_value);
}

template <>
RealtimeStatusOr<bool> VariableComparisonVisitor::operator()(
    const int64_t v, const int64_t other_value) {
  switch (comparison.operation) {
      // APPROX and NOT_APPROX are not supported for int64_t values
    case intrinsic_proto::icon::v1::Comparison::EQUAL:
      return v == other_value;
    case intrinsic_proto::icon::v1::Comparison::NOT_EQUAL:
      return v != other_value;
    case intrinsic_proto::icon::v1::Comparison::LESS_THAN_OR_EQUAL:
      return v <= other_value;
    case intrinsic_proto::icon::v1::Comparison::LESS_THAN:
      return v < other_value;
    case intrinsic_proto::icon::v1::Comparison::GREATER_THAN_OR_EQUAL:
      return v >= other_value;
    case intrinsic_proto::icon::v1::Comparison::GREATER_THAN:
      return v > other_value;
    default:
      return InvalidArgumentError(RealtimeStatus::StrCat(
          "comparison operator not supported, enum value: ",
          static_cast<int>(comparison.operation)));
  }
}

template <>
RealtimeStatusOr<bool> VariableComparisonVisitor::operator()(
    const int64_t v_int64, const double other_value) {
  if (comparison.operation == intrinsic_proto::icon::v1::Comparison::EQUAL ||
      comparison.operation ==
          intrinsic_proto::icon::v1::Comparison::NOT_EQUAL) {
    return InvalidArgumentError(
        "EQUAL/NOT_EQUAL operation not possible between integer and "
        "double.");
  }
  INTRINSIC_RT_RETURN_IF_ERROR(CheckDoubleInValidInt64Range(other_value));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double v, SafeInt64ToDouble(v_int64));
  return (*this)(v, other_value);
}

RealtimeStatusOr<bool> Evaluate(const StateVariableValue& variable,
                                const RealtimeComparison& comparison) {
  return std::visit(VariableComparisonVisitor(comparison), variable,
                    comparison.value);
}

RealtimeStatusOr<bool> EvaluateConditionElementRtcl(
    const RtclActionInterface& action,
    absl::Span<const ConditionElement> elements,
    const AggregatedRobotStatus& robot_status, size_t recursion_depth,
    size_t index);

struct ConditionElementVisitorRtcl {
  RealtimeStatusOr<bool> operator()(const RealtimeComparison& element) {
    return LookupAndEvaluate(action, element, robot_status);
  }

  // Returns 'true' if all elements evaluate to 'true'.
  RealtimeStatusOr<bool> operator()(const AllOfClause& element) {
    bool value = true;
    for (const auto& element_idx : element.element_indices) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          bool element_value,
          EvaluateConditionElementRtcl(action, elements, robot_status,
                                       recursion_depth, element_idx));
      value &= element_value;
    }
    // All elements of this clause evaluated to 'true'.
    return value;
  }

  // Returns 'true' if at least one element evaluate to 'true'
  RealtimeStatusOr<bool> operator()(const AnyOfClause& element) {
    bool value = false;
    // All elements need to evaluate to false
    for (const auto& element_idx : element.element_indices) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          bool element_value,
          EvaluateConditionElementRtcl(action, elements, robot_status,
                                       recursion_depth, element_idx));
      value |= element_value;
    }
    // All elements of this clause evaluated to 'false'.
    return value;
  }

  // Returns the negated value of the element.
  RealtimeStatusOr<bool> operator()(const NotClause& element) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const bool value,
        EvaluateConditionElementRtcl(action, elements, robot_status,
                                     recursion_depth, element.element_index));
    return !value;
  }

  const RtclActionInterface& action;
  const AggregatedRobotStatus& robot_status;
  absl::Span<const ConditionElement> elements;
  const size_t recursion_depth;
};

// The 'recursion_depth' parameter is used to track and limit the recursion
// depth.
RealtimeStatusOr<bool> EvaluateConditionElementRtcl(
    const RtclActionInterface& action,
    absl::Span<const ConditionElement> elements,
    const AggregatedRobotStatus& robot_status, size_t recursion_depth,
    size_t index) {
  if (index >= elements.size()) {
    return OutOfRangeError(
        "The index of the ConditionElement is out of range.");
  }
  if (recursion_depth >= elements.size()) {
    return InternalError(
        "Condition indices are invalid or have circular references.");
  }
  return std::visit(
      ConditionElementVisitorRtcl{.action = action,
                                  .robot_status = robot_status,
                                  .elements = elements,
                                  .recursion_depth = recursion_depth + 1},
      elements[index]);
}

struct OperandVisitorRtcl {
  OperandVisitorRtcl(const RtclActionInterface& action,
                     const RealtimeComparison& comparison,
                     const AggregatedRobotStatus& robot_status)
      : action(action), comparison(comparison), robot_status(robot_status) {}

  RealtimeStatusOr<bool> operator()(
      const StateVariableFieldSelectionFunction& selection_function) {
    const StateVariableFieldSelectionData data{.robot_status = robot_status};
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto variable_value,
                                  selection_function(data));
    auto result = Evaluate(variable_value, comparison);
    if (!result.ok()) {
      return RealtimeStatus(
          result.status().code(),
          RealtimeStatus::StrCat(
              "Comparison failed for a part status path with: ",
              result.status().message()));
    }
    return result;
  }

  RealtimeStatusOr<bool> operator()(
      const FixedString<kMaxStateVariableNameLength>&
          action_state_variable_name) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto variable_value,
        action.GetStateVariable(action_state_variable_name));
    auto result = Evaluate(variable_value, comparison);
    if (!result.ok()) {
      return RealtimeStatus(
          result.status().code(),
          RealtimeStatus::StrCat("Comparison failed for ",
                                 action_state_variable_name,
                                 " with: ", result.status().message()));
    }
    return result;
  }
  const RtclActionInterface& action;
  const RealtimeComparison& comparison;
  const AggregatedRobotStatus& robot_status;
};

}  // namespace

RealtimeStatusOr<bool> LookupAndEvaluate(
    const RtclActionInterface& action, const RealtimeComparison& comparison,
    const AggregatedRobotStatus& robot_status) {
  OperandVisitorRtcl visitor(action, comparison, robot_status);
  return std::visit(visitor, comparison.operand);
}

RealtimeStatusOr<bool> LookupAndEvaluate(
    const RtclActionInterface& action, const RealtimeCondition& condition,
    const AggregatedRobotStatus& robot_status) {
  return EvaluateConditionElementRtcl(action, condition.elements, robot_status,
                                      /*recursion_depth =*/0, /*index =*/0);
}

}  // namespace intrinsic::icon
