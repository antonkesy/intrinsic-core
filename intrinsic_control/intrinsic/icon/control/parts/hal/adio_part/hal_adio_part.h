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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_HAL_ADIO_PART_HAL_ADIO_PART_H_
#define INTRINSIC_ICON_CONTROL_PARTS_HAL_ADIO_PART_HAL_ADIO_PART_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/hal/adio_part/hal_adio_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/io_block.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/hardware_module_proxy.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// Provides access to a Analog and Digital Inputs and Outputs from HAL modules.
// The part will not send any commands if it is not enabled.
class HalADIOPart final : public HalRealtimePartBase, public ADIO {
 public:
  static constexpr char kPartTypeName[] = "HalADIOPart";

  // Builds a HalADIOPart from the given configuration proto as defined in
  // intrinsic/icon/control/parts/hal/adio_part/hal_adio_part_config.proto.
  //
  // It is possible to define multiple parts interacting with hardware modules.
  // This poses the risk of multiple parts controlling the same IO blocks!
  // Configuring the same output block multiple times with different names
  // results in undefined behavior.
  //
  // Returns an error if the configuration is invalid, or on parsing errors.
  static absl::StatusOr<std::unique_ptr<HalADIOPart>> FromProto(
      PartFactoryContext context,
      const intrinsic_proto::icon::HalADIOPartConfig& config);

  RealtimeStatus ReadStatus(ReadStatusParameters params) override;
  RealtimeStatus ApplyCommand(ApplyCommandParameters params) override;

  // Provides a stable pointer to the requested digital input block.
  // A nullptr is returned if the requested block doesn't exist.
  // The pointers remain stable for the lifetime of this part.
  // Note that the data in the blocks is not valid before `ReadStatus()` is
  // called the first time. This should never happen, as the RTCL runtime takes
  // care of this before making the interfaces available to any Actions.
  const DioBlock* DigitalInputBlock(absl::string_view name) const override;
  // Provides a stable pointer to the requested digital output block.
  DioBlock* MutableDigitalOutputBlock(absl::string_view name) override;
  // Provides a stable pointer to the requested analog input block.
  const AnalogBlock* AnalogInputBlock(absl::string_view name) const override;
  // Provides a stable pointer to the requested analog output block.
  AnalogBlock* MutableAnalogOutputBlock(absl::string_view name) override;

  // Names of the registered DigitalInputBlocks. Constant for the
  // lifetime of this part.
  absl::Span<const std::string> DigitalInputBlockNames() const override;
  absl::Span<const std::string> DigitalInputSignalNames(
      absl::string_view block_name) const override;

  // Names of the registered DigitalOutputBlocks. Constant for the
  // lifetime of this part.
  absl::Span<const std::string> DigitalOutputBlockNames() const override;
  absl::Span<const std::string> DigitalOutputSignalNames(
      absl::string_view block_name) const override;

  // Names of the registered AnalogInputBlocks. Constant for the
  // lifetime of this part.
  absl::Span<const std::string> AnalogInputBlockNames() const override;
  absl::Span<const std::string> AnalogInputSignalNames(
      absl::string_view block_name) const override;

  // Names of the registered AnalogOutputBlocks. Constant for the
  // lifetime of this part.
  absl::Span<const std::string> AnalogOutputBlockNames() const override;
  absl::Span<const std::string> AnalogOutputSignalNames(
      absl::string_view block_name) const override;

  // Realtime safe copy of the state of all IO blocks and an absl::Span for the
  // block names (they remain stable for the lifetime of this part).
  // WARNING: Accessing the return value of this after the Part it was obtained
  // from goes out of scope is an error!
  ADIO::ADIOState GetADIOState() const override;

 private:
  struct AnalogInputBlockData {
    // Local buffer of the block data.
    AnalogBlock block;
    intrinsic::icon::HardwareInterfaceHandle<intrinsic_fbs::AIOStatus>
        input_handle;
    std::vector<std::string> signal_names;
  };

  struct AnalogOutputBlockData {
    // Local buffer of the block data.
    AnalogBlock block;
    intrinsic::icon::MutableHardwareInterfaceHandle<intrinsic_fbs::AIOCommand>
        output_handle;
    std::vector<std::string> signal_names;
  };

  struct DigitalInputBlockData {
    // Local buffer of the block data.
    DioBlock block;
    intrinsic::icon::HardwareInterfaceHandle<intrinsic_fbs::DIOStatus>
        input_handle;
    std::vector<std::string> signal_names;
  };

  struct DigitalOutputBlockData {
    // Local buffer of the command.
    DioBlock block;
    intrinsic::icon::MutableHardwareInterfaceHandle<intrinsic_fbs::DIOCommand>
        output_handle;
    std::vector<std::string> signal_names;
  };

  // Internal PartConfig. Fully populated by `PopulateHalIOs`.
  struct PartConfig {
    // Names of the available blocks. Keys for the block maps.
    std::vector<std::string> digital_input_names;
    std::vector<std::string> digital_output_names;
    std::vector<std::string> analog_input_names;
    std::vector<std::string> analog_output_names;

    // Buffers the blocks read from and written to the respective HAL module.
    absl::flat_hash_map<std::string, DigitalInputBlockData> digital_inputs;
    absl::flat_hash_map<std::string, DigitalOutputBlockData> digital_outputs;
    absl::flat_hash_map<std::string, AnalogInputBlockData> analog_inputs;
    absl::flat_hash_map<std::string, AnalogOutputBlockData> analog_outputs;

    // Enables access to the hardware modules used by this module. e.g. used to
    // check their state.
    absl::flat_hash_set<const HardwareModuleProxy*> hardware_modules = {};
  };
  // Internal helper that creates a fully populated PartConfig  by parsing
  // `adio_part_config` and looking up all the required interfaces.
  //
  // Forwards lookup failures of HardwareInterfaceHandles.
  // Returns FailedPreconditionError if a block consists of too many handles.
  // Returns FailedPreconditionError on duplicate export names.
  static absl::StatusOr<HalADIOPart::PartConfig> PopulatePartConfig(
      const Context& context,
      const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config);
  static absl::Status AddAnalogInputsToConfig(
      const Context& context,
      const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config,
      const HardwareModuleManager* hardware_module_manager,
      HalADIOPart::PartConfig& config);
  static absl::Status AddAnalogOutputsToConfig(
      const Context& context,
      const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config,
      const HardwareModuleManager* hardware_module_manager,
      HalADIOPart::PartConfig& config);
  static absl::Status AddDigitalInputsToConfig(
      const Context& context,
      const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config,
      const HardwareModuleManager* hardware_module_manager,
      HalADIOPart::PartConfig& config);
  static absl::Status AddDigitalOutputsToConfig(
      const Context& context,
      const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config,
      const HardwareModuleManager* hardware_module_manager,
      HalADIOPart::PartConfig& config);
  // Constructs the HalADIOPart using a fully populated PartConfig.
  HalADIOPart(PartConfig part_config, HardwareModuleManager* manager);

  // Fully populated config.
  PartConfig config_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_HAL_ADIO_PART_HAL_ADIO_PART_H_
