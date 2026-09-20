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

#include "intrinsic/icon/control/actions/adio_action.h"

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/action_utils.h"
#include "intrinsic/icon/actions/adio.pb.h"
#include "intrinsic/icon/actions/adio_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/io_block.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

using ::intrinsic_proto::icon::actions::proto::AnalogDigitalInExpectations;
using ::intrinsic_proto::icon::actions::proto::Comparison;
using ::intrinsic_proto::icon::actions::proto::SetAnalogDigitalOutputs;
using ::intrinsic_proto::icon::v1::BehaviorOverrideRequest;

// Helper struct so ParseExpectations doesn't return a std::pair.
struct ParsedExpectations {
  std::vector<ADIOAction::AnalogInputExpectation> analog_input_expectations;
  std::vector<ADIOAction::DigitalInputExpectation> digital_input_expectations;
};

// Functor (for use with absl::StrJoin) that appends the key of a MapPair to an
// output string.
template <class K, class V>
struct KeyOnlyFormatter {
  void operator()(std::string* out,
                  const google::protobuf::MapPair<K, V>& key_value) const {
    out->append(key_value.first);
  }
};

// Parses the AnalogDigitalInExpectations proto passed in `expectations`.
//
// For every AnalogInputBlock/DigitalInputBlock in the proto
//   1. Verifies that a block of the correct type and with the correct name
//      exists in `adio_config`.
//   2. Obtains the pointer from the ADIO Interface.
//   3. Initializes FixedVectors for values/comparisons and mask (only for
//   digital inputs).
//   4. Iterates over the proto value map and
//     1. Verifies that the index is valid.
//     2. Sets the expected value/comparison for that index.
//     3. Sets the corresponding bit in the mask. (only for digital inputs)
// Returns an error if any of those steps fail.
absl::StatusOr<ParsedExpectations> ParseExpectations(
    const ::intrinsic_proto::icon::GenericAdioConfig& adio_config,
    const AnalogDigitalInExpectations& expectations) {
  ParsedExpectations parsed;

  if (expectations.analog_inputs_size() > ADIO::kMaxBlocks) {
    return absl::OutOfRangeError(absl::StrCat(
        "Provided number of AnalogInputBlocks (",
        expectations.analog_inputs_size(),
        ") exceeds maximum number of blocks (", ADIO::kMaxBlocks, ")."));
  }
  if (expectations.digital_inputs_size() > ADIO::kMaxBlocks) {
    return absl::OutOfRangeError(absl::StrCat(
        "Provided number of DigitalInputBlocks (",
        expectations.digital_inputs_size(),
        ") exceeds maximum number of blocks (", ADIO::kMaxBlocks, ")."));
  }

  parsed.analog_input_expectations.reserve(expectations.analog_inputs_size());
  for (const auto& [block_name, block] : expectations.analog_inputs()) {
    const auto& input_block_name_and_config =
        adio_config.analog_input_blocks().find(block_name);
    if (input_block_name_and_config ==
        adio_config.analog_input_blocks().end()) {
      return absl::NotFoundError(absl::StrCat(
          "AnalogInputBlock '", block_name,
          "' is not defined. Valid block names: \n",
          absl::StrJoin(
              adio_config.analog_input_blocks(), ", ",
              KeyOnlyFormatter<std::string,
                               ::intrinsic_proto::icon::GenericAdioConfig::
                                   AnalogInputOutputConfig>())));
    }
    size_t num_inputs = input_block_name_and_config->second.block_size();
    FixedVector<ADIOAction::Comparison, AnalogBlock::kMaxValuesPerBlock>
        comparisons(num_inputs);

    for (const auto& [index, comparison] : block.comparisons_by_index()) {
      if (index >= num_inputs) {
        return absl::OutOfRangeError(
            absl::StrCat("The analog input trigger for block '", block_name,
                         "' refers to the out-of-range index ", index,
                         ". The highest valid index is ", num_inputs - 1, "."));
      }
      ADIOAction::Comparison expectation{.expected_value =
                                             comparison.expected_value()};
      switch (comparison.operation()) {
        case Comparison::APPROX_EQUAL: {
          expectation.operation =
              ADIOAction::Comparison::FloatOperator::kApproxEqual;
          break;
        }
        case Comparison::APPROX_NOT_EQUAL: {
          expectation.operation =
              ADIOAction::Comparison::FloatOperator::kApproxNotEqual;
          break;
        }
        case Comparison::LESS_THAN_OR_EQUAL: {
          expectation.operation =
              ADIOAction::Comparison::FloatOperator::kLessThanOrEqual;
          break;
        }
        case Comparison::LESS_THAN: {
          expectation.operation =
              ADIOAction::Comparison::FloatOperator::kLessThan;
          break;
        }
        case Comparison::GREATER_THAN_OR_EQUAL: {
          expectation.operation =
              ADIOAction::Comparison::FloatOperator::kGreaterThanOrEqual;
          break;
        }
        case Comparison::GREATER_THAN: {
          expectation.operation =
              ADIOAction::Comparison::FloatOperator::kGreaterThan;
          break;
        }
        default:
          return absl::InvalidArgumentError(
              absl::StrCat("The comparison operation '",
                           Comparison::OpEnum_Name(comparison.operation()),
                           "' is not supported."));
      }
      comparisons[index] = expectation;
    }

    INTR_ASSIGN_OR_RETURN(
        auto expectation,
        ADIOAction::AnalogInputExpectation::CreateAnalogInputExpectation(
            input_block_name_and_config->first,
            input_block_name_and_config->second.units_size(),
            std::move(comparisons)));
    parsed.analog_input_expectations.emplace_back(std::move(expectation));
  }
  // Similarly initialize the DigitalInputBlocks.
  parsed.digital_input_expectations.reserve(expectations.digital_inputs_size());
  for (const auto& [block_name, block] : expectations.digital_inputs()) {
    const auto& input_block_name_and_config =
        adio_config.digital_input_blocks().find(block_name);
    if (input_block_name_and_config ==
        adio_config.digital_input_blocks().end()) {
      return absl::NotFoundError(absl::StrCat(
          "DigitalInputBlock '", block_name,
          "' is not defined. Valid block names: \n",
          absl::StrJoin(
              adio_config.digital_input_blocks(), ", ",
              KeyOnlyFormatter<std::string,
                               ::intrinsic_proto::icon::GenericAdioConfig::
                                   DigitalInputOutputConfig>())));
    }
    // The number of values in input_block_ptr is checked in `Create()`.
    size_t num_inputs = input_block_name_and_config->second.block_size();
    FixedVector<bool, DioBlock::kMaxValuesPerBlock> values(num_inputs, false);
    FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask(num_inputs, false);

    for (const auto& [index, value] : block.values_by_index()) {
      if (index >= num_inputs) {
        return absl::OutOfRangeError(
            absl::StrCat("The digital input trigger for block '", block_name,
                         "' refers to the out-of-range index ", index,
                         ". The highest valid index is ", num_inputs - 1, "."));
      }
      mask[index] = true;
      values[index] = value;
    }
    INTR_ASSIGN_OR_RETURN(
        auto expectation,
        ADIOAction::DigitalInputExpectation::CreateDigitalInputExpectation(
            block_name, input_block_name_and_config->second.block_size(),
            std::move(values), std::move(mask)));

    parsed.digital_input_expectations.emplace_back(std::move(expectation));
  }
  return parsed;
}

// Parses the SetAnalogDigitalOutputs proto passed in `set_outputs` and extracts
// the digital outputs.
//
// For every DigitalOutputBlock in the proto:
//   1. Verifies that a block of the correct type and with the correct name
//      exists in `adio_config`.
//   2. Obtains the pointer from the ADIO Interface.
//   3. Initializes FixedVectors for values and mask.
//   4. Iterates over the proto value map and
//     1. Verifies that the index is valid.
//     2. Sets the output value for that index.
//     3. Sets the corresponding bit in the mask.
// Returns an error if any of those steps fail.
absl::StatusOr<std::vector<ADIOAction::DigitalOutputBlock>> ParseDigitalOutputs(
    const ::intrinsic_proto::icon::GenericAdioConfig& adio_config,
    const SetAnalogDigitalOutputs& set_outputs) {
  if (set_outputs.digital_outputs_size() > ADIO::kMaxBlocks) {
    return absl::OutOfRangeError(absl::StrCat(
        "Provided number of DigitalOutputBlocks (",
        set_outputs.digital_outputs_size(),
        ") exceeds maximum number of blocks (", ADIO::kMaxBlocks, ")."));
  }
  std::vector<ADIOAction::DigitalOutputBlock> outputs;
  outputs.reserve(set_outputs.digital_outputs_size());
  for (const auto& [block_name, block] : set_outputs.digital_outputs()) {
    const auto& output_block_name_and_size =
        adio_config.digital_output_blocks().find(block_name);
    if (output_block_name_and_size ==
        adio_config.digital_output_blocks().end()) {
      return absl::NotFoundError(absl::StrCat(
          "DigitalOutputBlock '", block_name,
          "' is not defined. Valid block names: \n",
          absl::StrJoin(
              adio_config.digital_output_blocks(), ", ",
              KeyOnlyFormatter<std::string,
                               ::intrinsic_proto::icon::GenericAdioConfig::
                                   DigitalInputOutputConfig>())));
    }
    // The number of values in output_block_ptr is checked in `Create()`.
    size_t num_outputs = output_block_name_and_size->second.block_size();
    FixedVector<bool, DioBlock::kMaxValuesPerBlock> values(num_outputs, false);
    FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask(num_outputs, false);

    for (const auto& [index, value] : block.values_by_index()) {
      if (index >= num_outputs) {
        return absl::OutOfRangeError(absl::StrCat(
            "The digital output for block '", block_name,
            "' refers to the out-of-range index ", index,
            ". The highest valid index is ", num_outputs - 1, "."));
      }
      mask[index] = true;
      values[index] = value;
    }
    INTR_ASSIGN_OR_RETURN(
        auto output_block,
        ADIOAction::DigitalOutputBlock::CreateDigitalOutputBlock(
            output_block_name_and_size->first,
            output_block_name_and_size->second.block_size(), std::move(values),
            std::move(mask)));
    outputs.emplace_back(std::move(output_block));
  }
  return outputs;
}

// Parses the SetAnalogDigitalOutputs proto passed in `set_outputs` and extracts
// the analog outputs.
//
// For every AnalogOutputBlock in the proto:
//   1. Verifies that a block of the correct type and with the correct name
//      exists in `adio_config`.
//   2. Obtains the pointer from the ADIO Interface.
//   3. Initializes FixedVectors for values and mask.
//   4. Iterates over the proto value map and
//     1. Verifies that the index is valid.
//     2. Sets the output value for that index.
//     3. Sets the corresponding bit in the mask.
// Returns an error if any of those steps fail.
absl::StatusOr<std::vector<ADIOAction::AnalogOutputBlock>> ParseAnalogOutputs(
    const ::intrinsic_proto::icon::GenericAdioConfig& adio_config,
    const SetAnalogDigitalOutputs& set_outputs) {
  if (set_outputs.analog_outputs_size() > ADIO::kMaxBlocks) {
    return absl::OutOfRangeError(absl::StrCat(
        "Provided number of AnalogOutputBlocks (",
        set_outputs.analog_outputs_size(),
        ") exceeds maximum number of blocks (", ADIO::kMaxBlocks, ")."));
  }
  std::vector<ADIOAction::AnalogOutputBlock> outputs;
  outputs.reserve(set_outputs.analog_outputs_size());
  for (const auto& [block_name, block] : set_outputs.analog_outputs()) {
    const auto& output_block_name_and_size =
        adio_config.analog_output_blocks().find(block_name);
    if (output_block_name_and_size ==
        adio_config.analog_output_blocks().end()) {
      return absl::NotFoundError(absl::StrCat(
          "AnalogOutputBlock '", block_name,
          "' is not defined. Valid block names: \n",
          absl::StrJoin(
              adio_config.analog_output_blocks(), ", ",
              KeyOnlyFormatter<std::string,
                               ::intrinsic_proto::icon::GenericAdioConfig::
                                   AnalogInputOutputConfig>())));
    }
    // The number of values in output_block_ptr is checked in `Create()`.
    size_t num_outputs = output_block_name_and_size->second.block_size();
    FixedVector<double, DioBlock::kMaxValuesPerBlock> values(num_outputs, 0);
    FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask(num_outputs, false);

    for (const auto& [index, value] : block.values_by_index()) {
      if (index >= num_outputs) {
        return absl::OutOfRangeError(absl::StrCat(
            "The analog output for block '", block_name,
            "' refers to the out-of-range index ", index,
            ". The highest valid index is ", num_outputs - 1, "."));
      }
      mask[index] = true;
      values[index] = value;
    }
    INTR_ASSIGN_OR_RETURN(
        auto output_block,
        ADIOAction::AnalogOutputBlock::CreateAnalogOutputBlock(
            output_block_name_and_size->first,
            output_block_name_and_size->second.block_size(), std::move(values),
            std::move(mask)));
    outputs.emplace_back(std::move(output_block));
  }
  return outputs;
}

}  // namespace

bool ADIOAction::Comparison::Evaluate(double analog_value) const {
  switch (operation) {
    case FloatOperator::kNone:
      // Returns true so that the check in Sense() works as expected.
      return true;
    case FloatOperator::kApproxEqual:
      return std::abs(analog_value - expected_value) <= kMaxAbsError;
    case FloatOperator::kApproxNotEqual:
      return std::abs(analog_value - expected_value) > kMaxAbsError;
    case FloatOperator::kLessThanOrEqual:
      return analog_value <= expected_value;
    case FloatOperator::kLessThan:
      return analog_value < expected_value;
    case FloatOperator::kGreaterThanOrEqual:
      return analog_value >= expected_value;
    case FloatOperator::kGreaterThan:
      return analog_value > expected_value;
  }
}

// static
absl::StatusOr<ADIOAction::AnalogInputExpectation>
ADIOAction::AnalogInputExpectation::CreateAnalogInputExpectation(
    absl::string_view input_block_name, size_t input_block_size,
    FixedVector<Comparison, AnalogBlock::kMaxValuesPerBlock> comparisons) {
  if (comparisons.size() != input_block_size) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to 'CreateAnalogInputExpectation'. The size of 'comparisons' (",
        comparisons.size(), ") doesn't match the size of analog input block '",
        input_block_name, "' (", input_block_size, ")."));
  }

  return AnalogInputExpectation(input_block_name, std::move(comparisons));
}

// static
absl::StatusOr<ADIOAction::DigitalInputExpectation>
ADIOAction::DigitalInputExpectation::CreateDigitalInputExpectation(
    absl::string_view input_block_name, size_t input_block_size,
    FixedVector<bool, DioBlock::kMaxValuesPerBlock> expected_values,
    FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask) {
  if (expected_values.size() != mask.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to 'CreateDigitalInputExpectation'. The size of "
        "'expected_values' (",
        expected_values.size(), ") doesn't match the size of 'mask' (",
        mask.size(), ")) "));
  } else if (expected_values.size() != input_block_size) {
    return absl::InvalidArgumentError(
        absl::StrCat("Failed to 'CreateDigitalInputExpectation'. The size of "
                     "'expected_values' (",
                     expected_values.size(),
                     ") doesn't match the size of digital input block '",
                     input_block_name, "' (", input_block_size, ")) "));
  }

  return DigitalInputExpectation(input_block_name, std::move(expected_values),
                                 std::move(mask));
}

// static
absl::StatusOr<ADIOAction::DigitalOutputBlock>
ADIOAction::DigitalOutputBlock::CreateDigitalOutputBlock(
    absl::string_view output_block_name, size_t output_block_size,
    FixedVector<bool, DioBlock::kMaxValuesPerBlock> values,
    FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask) {
  if (values.size() != mask.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to 'CreateDigitalOutputBlock'. The size of 'values' (",
        values.size(), ") doesn't match the size of 'mask' (", mask.size(),
        ")."));
  } else if (values.size() != output_block_size) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to 'CreateDigitalOutputBlock'. The size of 'values' (",
        values.size(), ") doesn't match the size of digital output block '",
        output_block_name, "' (", output_block_size, ")."));
  }

  return DigitalOutputBlock(output_block_name, std::move(values),
                            std::move(mask));
}

// static
absl::StatusOr<ADIOAction::AnalogOutputBlock>
ADIOAction::AnalogOutputBlock::CreateAnalogOutputBlock(
    absl::string_view output_block_name, size_t output_block_size,
    FixedVector<double, AnalogBlock::kMaxValuesPerBlock> values,
    FixedVector<bool, AnalogBlock::kMaxValuesPerBlock> mask) {
  if (values.size() != mask.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to 'CreateAnalogOutputBlock'. The size of 'values' (",
        values.size(), ") doesn't match the size of 'mask' (", mask.size(),
        ")."));
  } else if (values.size() != output_block_size) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to 'CreateAnalogOutputBlock'. The size of 'values' (",
        values.size(), ") doesn't match the size of analog output block '",
        output_block_name, "' (", output_block_size, ")."));
  }
  return AnalogOutputBlock(output_block_name, std::move(values),
                           std::move(mask));
}

absl::StatusOr<std::unique_ptr<ADIOAction>> ADIOAction::Create(
    const ADIOActionInfo::FixedParams& params, ActionFactoryContext& context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(SlotInfo adio_slot_info,
                        context.GetSlotInfo(ADIOActionInfo::kAdioSlotName));
  const ::intrinsic_proto::icon::GenericPartConfig& generic_config =
      adio_slot_info.config.generic_config();
  if (!generic_config.has_adio_config()) {
    return absl::FailedPreconditionError(
        absl::StrCat("Part assigned to slot '", ADIOActionInfo::kAdioSlotName,
                     "' does not have the ADIO feature interface."));
  }
  if (!params.has_expectations() && !params.has_outputs()) {
    return absl::InvalidArgumentError(
        "At least one of {'expectations', 'outputs'} must be non-empty.");
  }

  INTR_ASSIGN_OR_RETURN(
      auto parsed_expectations,
      ParseExpectations(generic_config.adio_config(), params.expectations()));

  std::vector<DigitalOutputBlock> digital_outputs;
  INTR_ASSIGN_OR_RETURN(
      digital_outputs,
      ParseDigitalOutputs(generic_config.adio_config(), params.outputs()));

  std::vector<AnalogOutputBlock> analog_outputs;
  INTR_ASSIGN_OR_RETURN(
      analog_outputs,
      ParseAnalogOutputs(generic_config.adio_config(), params.outputs()));

  FixedParams realtime_params{
      .analog_input_expectations =
          {parsed_expectations.analog_input_expectations.begin(),
           parsed_expectations.analog_input_expectations.end()},
      .analog_outputs = {analog_outputs.begin(), analog_outputs.end()},
      .digital_input_expectations =
          {parsed_expectations.digital_input_expectations.begin(),
           parsed_expectations.digital_input_expectations.end()},
      .digital_outputs = {digital_outputs.begin(), digital_outputs.end()}};

  return absl::WrapUnique(
      new ADIOAction(adio_slot_info.slot_id, std::move(realtime_params)));
}

ADIOAction::ADIOAction(RealtimeSlotId slot_id, FixedParams params)
    : slot_id_(slot_id), params_(std::move(params)) {}

RealtimeStatus ADIOAction::OnEnter(OnEnterParameters params) {
  all_inputs_match_ = false;
  outputs_set_ = false;
  outputs_set_buffer_ = false;

  return OkStatus();
}

RealtimeStatus ADIOAction::Sense(SenseParameters params) {
  const auto* adio = params.slot_map.GetInterfaceForSlot<ADIO>(slot_id_);
  if (adio == nullptr) {
    return InternalError(
        RealtimeStatus::StrCat("Slot at SlotId ", slot_id_.value(),
                               " does not have ADIO FeatureInterface."));
  }
  // Update the output state first to enable early return for the input state.
  outputs_set_ = outputs_set_buffer_;

  // No inputs are defined, so the StateVariable should be `false`.
  if (params_.analog_input_expectations.empty() &&
      params_.digital_input_expectations.empty()) {
    all_inputs_match_ = false;
    return OkStatus();
  }

  for (const auto& input : params_.analog_input_expectations) {
    const auto* input_block = adio->AnalogInputBlock(input.input_block_name_);
    if (input_block == nullptr) {
      return InternalError(
          RealtimeStatus::StrCat("Slot does not have analog input block '",
                                 input.input_block_name_, "'"));
    }
    for (size_t i = 0; i < input.comparisons_.size(); i++) {
      // Return early if any expectation is false.
      // Default-constructed comparisons evaluate to true so that we don't need
      // to use a mask.
      if (!input.comparisons_[i].Evaluate(input_block->Values()[i])) {
        all_inputs_match_ = false;
        return OkStatus();
      }
    }
  }
  for (const auto& input : params_.digital_input_expectations) {
    const auto* input_block = adio->DigitalInputBlock(input.input_block_name_);
    if (input_block == nullptr) {
      return InternalError(
          RealtimeStatus::StrCat("Slot does not have digital input block '",
                                 input.input_block_name_, "'"));
    }
    for (size_t i = 0; i < input.expected_values_.size(); i++) {
      // Return early if any expectation is false.
      if (input.mask_[i] &&
          input.expected_values_[i] != input_block->Values()[i]) {
        all_inputs_match_ = false;
        return OkStatus();
      }
    }
  }

  all_inputs_match_ = true;

  return OkStatus();
}

RealtimeStatus ADIOAction::Control(ControlParameters params) {
  auto* adio = params.slot_map.GetMutableInterfaceForSlot<ADIO>(slot_id_);
  if (adio == nullptr) {
    return InternalError(
        RealtimeStatus::StrCat("Slot at SlotId ", slot_id_.value(),
                               " does not have ADIO FeatureInterface."));
  }

  if (params.requested_behavior_override ==
      BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_PAUSE) {
    INTRINSIC_RT_LOG_THROTTLED(INFO) << "PAUSE requested. Not setting values.";
    outputs_set_buffer_ = false;
    return OkStatus();
  }

  for (const auto& output : params_.digital_outputs) {
    auto* output_block =
        adio->MutableDigitalOutputBlock(output.output_block_name_);
    if (output_block == nullptr) {
      return InternalError(
          RealtimeStatus::StrCat("Slot does not have digital output block '",
                                 output.output_block_name_, "'"));
    }
    for (size_t i = 0; i < output.values_.size(); i++) {
      if (output.mask_[i]) {
        output_block->MutableValues()[i] = output.values_[i];
      }
    }
    outputs_set_buffer_ = true;
  }
  for (const auto& output : params_.analog_outputs) {
    auto* output_block =
        adio->MutableAnalogOutputBlock(output.output_block_name_);
    if (output_block == nullptr) {
      return InternalError(
          RealtimeStatus::StrCat("Slot does not have analog output block '",
                                 output.output_block_name_, "'"));
    }
    for (size_t i = 0; i < output.values_.size(); i++) {
      if (output.mask_[i]) {
        output_block->MutableValues()[i] = output.values_[i];
      }
    }
    outputs_set_buffer_ = true;
  }
  return OkStatus();
}

RealtimeStatusOr<StateVariableValue> ADIOAction::GetStateVariable(
    absl::string_view name) const {
  if (name == ADIOActionInfo::kAllInputsMatch) {
    return StateVariableValue(all_inputs_match_);
  }
  if (name == ADIOActionInfo::kOutputsSet) {
    return StateVariableValue(outputs_set_);
  }
  return NotFoundError(RealtimeStatus::StrCat(
      ADIOActionInfo::kActionTypeName, ", state variable not found ", name));
}

// static
intrinsic_proto::icon::v1::ActionSignature ADIOAction::GetSignature() {
  ActionSignatureBuilder builder(ADIOActionInfo::kActionTypeName,
                                 ADIOActionInfo::kActionDescription);
  CHECK_OK(builder.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                        StateVariableInfo::TYPE_BOOL>(
      ADIOActionInfo::kAllInputsMatch,
      ADIOActionInfo::kAllInputsMatchDescription));
  CHECK_OK(builder.AddPartSlot(
      ADIOActionInfo::kAdioSlotName, ADIOActionInfo::kAdioSlotDescription,
      {intrinsic_proto::icon::v1::FeatureInterfaceTypes::
           FEATURE_INTERFACE_ADIO}));
  CHECK_OK(builder.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                        StateVariableInfo::TYPE_BOOL>(
      ADIOActionInfo::kOutputsSet, ADIOActionInfo::kOutputsSetDescription));
  CHECK_OK(builder.AddSupportedBehaviorOverride(
      intrinsic_proto::icon::v1::BehaviorOverrideRequest::
          BEHAVIOR_OVERRIDE_REQUEST_PAUSE,
      "Pauses setting analog and digital outputs."));
  CHECK_OK(builder.SetFixedParametersType<ADIOActionInfo::FixedParams>());
  return builder.Finish();
}

}  // namespace intrinsic::icon
