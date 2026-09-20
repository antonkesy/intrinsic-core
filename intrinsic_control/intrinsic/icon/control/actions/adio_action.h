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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_ADIO_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_ADIO_ACTION_H_

#include <any>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "intrinsic/icon/actions/adio_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/io_block.h"
#include "intrinsic/icon/control/reaction.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

// Action to react to Analog/Digital Inputs and set Digital Outputs.
class ADIOAction final : public RtclActionInterface {
 public:
  // A Comparison evaluates an `analog_value` against an `expected_value` using
  // a `FloatOperator`.
  // A default-constructed Comparison is initialized with "kNone" and "0.0".
  struct Comparison {
    // The set of operations available for floating point comparisons.
    enum class FloatOperator {
      // Needs to stay compatible with
      // intrinsic/icon/actions/adio.proto.
      // kNone - Used for default construction. Always evaluates to true.
      kNone,
      kApproxEqual,
      kApproxNotEqual,
      kLessThanOrEqual,
      kLessThan,
      kGreaterThanOrEqual,
      kGreaterThan,
    };
    FloatOperator operation = FloatOperator::kNone;
    double expected_value = 0.0;
    static constexpr double kMaxAbsError = 0x1p-10;
    // Evaluates `analog_value` against the stored `expected_value` using
    // `operation`.
    // Returns true if `analog_value` `operation` `expected_value` .
    // E.g. returns `true` for an `analog_value` of 1.0, an `operation` of
    // `kLessThan` and an `expected_value` 1.5.
    bool Evaluate(double analog_value) const;
  };

  // An AnalogInputExpectation holds all the information required to evaluate
  // one AnalogInput block.
  class AnalogInputExpectation {
   public:
    // Required to use this class with std::vector.
    AnalogInputExpectation();

    // 'expected_values' where the corresponding 'mask' entries are false are
    // ignored by the action.
    // Returns an error if there is a size mismatch between 'expected_values'
    // and `input_block_size`.
    static absl::StatusOr<AnalogInputExpectation> CreateAnalogInputExpectation(
        absl::string_view input_block_name, size_t input_block_size,
        FixedVector<Comparison, AnalogBlock::kMaxValuesPerBlock> comparisons);

    const std::string input_block_name_;
    // All default-initialized comparisons evaluate to true.
    const FixedVector<Comparison, AnalogBlock::kMaxValuesPerBlock> comparisons_;

   private:
    AnalogInputExpectation(
        absl::string_view input_block_name,
        FixedVector<Comparison, AnalogBlock::kMaxValuesPerBlock> comparisons)
        : input_block_name_(input_block_name),
          comparisons_(std::move(comparisons)) {}
  };

  // A DigitalInputExpectation holds all the information required to evaluate
  // one DigitalInput block.
  class DigitalInputExpectation {
   public:
    // Required to use this class with std::vector.
    DigitalInputExpectation();

    // 'expected_values' where the corresponding 'mask' entries are false are
    // ignored by the action.
    // Returns an error if there is a size mismatch between 'expected_values',
    // 'mask' and input_block_size'.
    static absl::StatusOr<DigitalInputExpectation>
    CreateDigitalInputExpectation(
        absl::string_view input_block_name, size_t input_block_size,
        FixedVector<bool, DioBlock::kMaxValuesPerBlock> expected_values,
        FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask);

    const std::string input_block_name_;
    const FixedVector<bool, DioBlock::kMaxValuesPerBlock> expected_values_;
    // The action ignores values where the respective mask entries are false.
    const FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask_;

   private:
    DigitalInputExpectation(
        absl::string_view input_block_name,
        FixedVector<bool, DioBlock::kMaxValuesPerBlock> expected_values,
        FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask)
        : input_block_name_(input_block_name),
          expected_values_(std::move(expected_values)),
          mask_(std::move(mask)) {}
  };

  // A DigitalOutputBlock holds all the information required to mutate one
  // DigitalOutput block.
  class DigitalOutputBlock {
   public:
    // Required to use this class with std::vector.
    DigitalOutputBlock();

    // 'values' where the corresponding 'mask' entries are false are ignored by
    // the action.
    // Returns an error if there is a size mismatch between 'values', 'mask' and
    // `output_block_size`).
    static absl::StatusOr<DigitalOutputBlock> CreateDigitalOutputBlock(
        absl::string_view output_block_name, size_t output_block_size,
        FixedVector<bool, DioBlock::kMaxValuesPerBlock> values,
        FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask);

    const std::string output_block_name_;
    const FixedVector<bool, DioBlock::kMaxValuesPerBlock> values_;
    // The action ignores values where the respective mask entries are false.
    const FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask_;

   private:
    DigitalOutputBlock(absl::string_view output_block_name,
                       FixedVector<bool, DioBlock::kMaxValuesPerBlock> values,
                       FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask)
        : output_block_name_(output_block_name),
          values_(std::move(values)),
          mask_(std::move(mask)) {}
  };

  class AnalogOutputBlock {
   public:
    // Required to use this class with std::vector.
    AnalogOutputBlock();

    // 'values' where the corresponding 'mask' entries are false are ignored by
    // the action.
    // Returns an InvalidArgumentError if there is a size mismatch between
    // 'values', 'mask' and `output_block_size`).
    static absl::StatusOr<AnalogOutputBlock> CreateAnalogOutputBlock(
        absl::string_view output_block_name, size_t output_block_size,
        FixedVector<double, DioBlock::kMaxValuesPerBlock> values,
        FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask);

    const std::string output_block_name_;
    const FixedVector<double, DioBlock::kMaxValuesPerBlock> values_;
    // The action ignores values where the respective mask entries are false.
    const FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask_;

   private:
    AnalogOutputBlock(absl::string_view output_block_name,
                      FixedVector<double, DioBlock::kMaxValuesPerBlock> values,
                      FixedVector<bool, DioBlock::kMaxValuesPerBlock> mask)
        : output_block_name_(output_block_name),
          values_(std::move(values)),
          mask_(std::move(mask)) {}
  };

  struct FixedParams {
    FixedVector<AnalogInputExpectation, ADIO::kMaxBlocks>
        analog_input_expectations;
    FixedVector<AnalogOutputBlock, ADIO::kMaxBlocks> analog_outputs;
    FixedVector<DigitalInputExpectation, ADIO::kMaxBlocks>
        digital_input_expectations;
    FixedVector<DigitalOutputBlock, ADIO::kMaxBlocks> digital_outputs;
  };

  // Parses `prams`, looks up IO blocks, gets IO references. Returns an error
  // for unknown blocks and invalid indices.
  static absl::StatusOr<std::unique_ptr<ADIOAction>> Create(
      const ADIOActionInfo::FixedParams& params, ActionFactoryContext& context);

  static intrinsic_proto::icon::v1::ActionSignature GetSignature();

  RealtimeStatus OnEnter(OnEnterParameters params) override;

  RealtimeStatus Sense(SenseParameters params) override;

  RealtimeStatus Control(ControlParameters params) override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override;

 private:
  explicit ADIOAction(RealtimeSlotId slot_id, FixedParams params);
  RealtimeSlotId slot_id_;
  FixedParams params_;

  // Internal variable corresponding to the 'ADIOActionInfo::kInputsMatch'
  // condition variable.
  bool all_inputs_match_ = false;
  // Internal variable corresponding to the
  // 'ADIOActionInfo::kOutputsSet' condition variable.
  bool outputs_set_ = false;
  // Saves the "outputs_set_" status of the Action between calls to Control()
  // and Sense().
  bool outputs_set_buffer_ = false;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_ADIO_ACTION_H_
