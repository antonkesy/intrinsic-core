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

#include "intrinsic/icon/control/parts/hal/adio_part/hal_adio_part.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/hal/adio_part/hal_adio_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal/v1/hal_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/io_block.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/hardware_module_proxy.h"
#include "intrinsic/icon/hal/interfaces/adio.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
namespace {

// Returns the `export_name` if it is defined in the `block_config`, or
// generates the export name of a block using `interface_name` and
// `module_name`.
// Returns InvalidArgumentError if export name is defined, but empty.
absl::StatusOr<std::string> ExportName(
    absl::string_view module_name, absl::string_view interface_name,
    const intrinsic_proto::icon::HalADIOPartConfig::HalBlockConfig&
        block_config) {
  std::string export_name = "";
  if (!block_config.has_export_name()) {
    export_name = absl::StrCat(module_name, "_", interface_name);
  } else {
    export_name = block_config.export_name();
  }
  if (export_name.empty()) {
    return InvalidArgumentError(
        absl::StrCat("Got empty export name for interface[", interface_name,
                     "] of module[", module_name, "]."));
  }
  return export_name;
}

}  // namespace

// static
absl::StatusOr<std::unique_ptr<HalADIOPart>> HalADIOPart::FromProto(
    PartFactoryContext context,
    const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config) {
  INTRINSIC_ASSERT_NON_REALTIME();
  if (adio_part_config.analog_inputs().empty() &&
      adio_part_config.analog_outputs().empty() &&
      adio_part_config.digital_inputs().empty() &&
      adio_part_config.digital_outputs().empty()) {
    return FailedPreconditionError(
        "At least one input or output block needs to be defined.");
  }

  if (const size_t num_blocks = adio_part_config.analog_inputs().size();
      num_blocks > kMaxBlocks) {
    return absl::FailedPreconditionError(absl::StrCat(
        "The number of analog_inputs(", num_blocks,
        ") exceeds the maximum number of blocks(", kMaxBlocks, ")."));
  }
  if (const size_t num_blocks = adio_part_config.analog_outputs().size();
      num_blocks > kMaxBlocks) {
    return absl::FailedPreconditionError(absl::StrCat(
        "The number of analog_outputs(", num_blocks,
        ") exceeds the maximum number of blocks(", kMaxBlocks, ")."));
  }
  if (const size_t num_blocks = adio_part_config.digital_inputs().size();
      num_blocks > kMaxBlocks) {
    return absl::FailedPreconditionError(absl::StrCat(
        "The number of digital_inputs(", num_blocks,
        ") exceeds the maximum number of blocks(", kMaxBlocks, ")."));
  }
  if (const size_t num_blocks = adio_part_config.digital_outputs().size();
      num_blocks > kMaxBlocks) {
    return absl::FailedPreconditionError(absl::StrCat(
        "The number of digital_outputs(", num_blocks,
        ") exceeds the maximum number of blocks(", kMaxBlocks, ")."));
  }
  INTR_ASSIGN_OR_RETURN(HalADIOPart::PartConfig config,
                        PopulatePartConfig(context.context, adio_part_config));

  std::vector<std::string> hardware_module_names;
  for (const auto* proxy : config.hardware_modules) {
    hardware_module_names.push_back(std::string(proxy->Name()));
  }
  auto part = absl::WrapUnique(new HalADIOPart(
      std::move(config), context.context.GetHardwareModuleManager()));
  for (const auto& name : hardware_module_names) {
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(name));
  }
  return part;
}

HalADIOPart::HalADIOPart(PartConfig part_config, HardwareModuleManager* manager)
    : HalRealtimePartBase(manager), config_(std::move(part_config)) {
  QCHECK(interface_registry_.RegisterInterface<ADIO>(this).ok());
}

// static
absl::StatusOr<HalADIOPart::PartConfig> HalADIOPart::PopulatePartConfig(
    const Context& context,
    const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config) {
  HalADIOPart::PartConfig config;
  const HardwareModuleManager* hardware_module_manager =
      context.GetHardwareModuleManager();

  if (!hardware_module_manager) {
    return absl::FailedPreconditionError(
        "Could not access the HardwareModuleManager");
  }

  INTR_RETURN_IF_ERROR(AddAnalogInputsToConfig(
      context, adio_part_config, hardware_module_manager, config));

  INTR_RETURN_IF_ERROR(AddAnalogOutputsToConfig(
      context, adio_part_config, hardware_module_manager, config));

  INTR_RETURN_IF_ERROR(AddDigitalInputsToConfig(
      context, adio_part_config, hardware_module_manager, config));

  INTR_RETURN_IF_ERROR(AddDigitalOutputsToConfig(
      context, adio_part_config, hardware_module_manager, config));

  // Sort names in default order for status export and nicer access.
  std::sort(std::begin(config.analog_input_names),
            std::end(config.analog_input_names));
  std::sort(std::begin(config.analog_output_names),
            std::end(config.analog_output_names));
  std::sort(std::begin(config.digital_input_names),
            std::end(config.digital_input_names));
  std::sort(std::begin(config.digital_output_names),
            std::end(config.digital_output_names));

  return config;
}

// static
absl::Status HalADIOPart::AddAnalogInputsToConfig(
    const Context& context,
    const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config,
    const HardwareModuleManager* hardware_module_manager,
    HalADIOPart::PartConfig& config) {
  std::vector<std::string> analog_input_names;
  // Used to check for duplicate interfaces.
  absl::flat_hash_set<std::string> analog_input_raw_interface_names;
  // Configures all Analog inputs.
  for (const auto& block_config : adio_part_config.analog_inputs()) {
    const auto& module_name = block_config.interface().module_name();
    const auto& interface_name = block_config.interface().interface_name();
    std::string analog_input_raw_interface_name =
        absl::StrCat(module_name, "_", interface_name);
    if (analog_input_raw_interface_names.contains(
            analog_input_raw_interface_name)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Duplicate analog input: ", analog_input_raw_interface_name));
    }
    analog_input_raw_interface_names.insert(analog_input_raw_interface_name);
    INTR_ASSIGN_OR_RETURN(
        std::string export_name,
        ExportName(module_name, interface_name, block_config));
    if (absl::c_find(analog_input_names, export_name) !=
        analog_input_names.end()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Duplicate analog input export name: ", export_name));
    };
    analog_input_names.push_back(export_name);
    AnalogInputBlockData block_data;
    INTR_ASSIGN_OR_RETURN(
        block_data.input_handle,
        context.GetHardwareInterfaceHandle<intrinsic_fbs::AIOStatus>(
            module_name, interface_name));
    size_t size = block_data.input_handle->signals()->size();
    if (size > AnalogBlock::kMaxValuesPerBlock) {
      return absl::FailedPreconditionError(
          absl::StrCat("The AnalogInputBlock '", export_name, "' defines ",
                       size, " values. This part can only handle ",
                       AnalogBlock::kMaxValuesPerBlock, "."));
    }
    LOG(INFO) << "Configuring " << module_name << ":" << interface_name
              << " as " << export_name << " with " << size << " signals";
    std::vector<AnalogBlock::Unit> units;
    units.reserve(size);
    for (size_t i = 0; i < size; i++) {
      const auto* signal = block_data.input_handle->signals()->Get(i);
      if (signal == nullptr) {
        return absl::FailedPreconditionError(
            absl::StrCat("Invalid AnalogInputBlock '", export_name,
                         "' at index ", i, ". Block is nullptr."));
      }
      // `unit()` is an inline scalar so no nullptr check is required.
      units.emplace_back(static_cast<AnalogBlock::Unit>(signal->unit()));
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(block_data.block, AnalogBlock::Create(units));

    // Copy the signal names.
    for (size_t signal_idx = 0; signal_idx < size; ++signal_idx) {
      const auto* signal = block_data.input_handle->signals()->Get(signal_idx);
      if (signal == nullptr) {
        return absl::FailedPreconditionError(
            absl::StrCat("Invalid AnalogInputBlock '", export_name,
                         "' at index ", signal_idx, ". Block is nullptr."));
      }
      if (signal->name() == nullptr) {
        return absl::FailedPreconditionError(absl::StrCat(
            "Invalid AnalogInputBlock '", export_name, "' at index ",
            signal_idx, ". signal_name is nullptr."));
      }
      block_data.signal_names.push_back(
          std::string(signal->name()->string_view()));
    }
    // emplace should always succeed here since we checked for duplicates
    // before.
    config.analog_inputs.emplace(export_name, std::move(block_data));
    if (const HardwareModuleProxy* module_proxy =
            hardware_module_manager->GetHardwareModuleProxy(module_name);
        module_proxy == nullptr) {
      return absl::FailedPreconditionError(absl::StrCat(
          "Failed to get HardwareModuleProxy for interface: ", module_name));
    } else {
      // Only required once, fine to add multiple times.
      config.hardware_modules.emplace(std::move(module_proxy));
    }
  }
  config.analog_input_names = std::move(analog_input_names);

  return absl::OkStatus();
};

// static
absl::Status HalADIOPart::AddAnalogOutputsToConfig(
    const Context& context,
    const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config,
    const HardwareModuleManager* hardware_module_manager,
    HalADIOPart::PartConfig& config) {
  std::vector<std::string> analog_output_names;
  absl::flat_hash_set<std::string> analog_output_raw_interface_names;
  // Configures all Analog outputs.
  for (const auto& block_config : adio_part_config.analog_outputs()) {
    const auto& module_name = block_config.interface().module_name();
    const auto& interface_name = block_config.interface().interface_name();
    std::string analog_output_raw_interface_name =
        absl::StrCat(module_name, "_", interface_name);
    if (analog_output_raw_interface_names.contains(
            analog_output_raw_interface_name)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Duplicate analog output: ", analog_output_raw_interface_name));
    }
    analog_output_raw_interface_names.insert(analog_output_raw_interface_name);
    INTR_ASSIGN_OR_RETURN(
        std::string export_name,
        ExportName(module_name, interface_name, block_config));
    if (absl::c_find(analog_output_names, export_name) !=
        analog_output_names.end()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Duplicate analog output export name: ", export_name));
    };
    analog_output_names.push_back(export_name);
    AnalogOutputBlockData block_data;
    INTR_ASSIGN_OR_RETURN(
        block_data.output_handle,
        context.GetMutableHardwareInterfaceHandle<intrinsic_fbs::AIOCommand>(
            module_name, interface_name));
    size_t size = block_data.output_handle->signals()->size();
    if (size > AnalogBlock::kMaxValuesPerBlock) {
      return absl::FailedPreconditionError(
          absl::StrCat("The AnalogOutputBlock '", export_name, "' defines ",
                       size, " values. This part can only handle ",
                       AnalogBlock::kMaxValuesPerBlock, "."));
    }
    LOG(INFO) << "Configuring " << module_name << ":" << interface_name
              << " as " << export_name << " with " << size << " signals";
    std::vector<AnalogBlock::Unit> units;
    units.reserve(size);
    for (size_t i = 0; i < size; i++) {
      const auto* signal = block_data.output_handle->signals()->Get(i);
      if (signal == nullptr) {
        return absl::FailedPreconditionError(
            absl::StrCat("Invalid AnalogOutputBlock '", export_name,
                         "' at index ", i, ". Block is nullptr."));
      }
      // `unit()` is an inline scalar so no nullptr check is required.
      units.emplace_back(static_cast<AnalogBlock::Unit>(signal->unit()));
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(block_data.block, AnalogBlock::Create(units));

    // Copy the signal names.
    for (size_t signal_idx = 0; signal_idx < size; ++signal_idx) {
      const auto* signal = block_data.output_handle->signals()->Get(signal_idx);
      if (signal == nullptr) {
        return absl::FailedPreconditionError(
            absl::StrCat("Invalid AnalogOutputBlock '", export_name,
                         "' at index ", signal_idx, ". Block is nullptr."));
      }
      if (signal->name() == nullptr) {
        return absl::FailedPreconditionError(absl::StrCat(
            "Invalid AnalogOutputBlock '", export_name, "' at index ",
            signal_idx, ". signal_name is nullptr."));
      }
      block_data.signal_names.push_back(
          std::string(signal->name()->string_view()));
    }
    // emplace should always succeed here since we checked for duplicates
    // before.
    config.analog_outputs.emplace(export_name, std::move(block_data));
    if (const HardwareModuleProxy* module_proxy =
            hardware_module_manager->GetHardwareModuleProxy(module_name);
        module_proxy == nullptr) {
      return absl::FailedPreconditionError(absl::StrCat(
          "Failed to get HardwareModuleProxy for interface: ", module_name));
    } else {
      // Only required once, fine to add multiple times.
      config.hardware_modules.emplace(std::move(module_proxy));
    }
  }
  config.analog_output_names = std::move(analog_output_names);

  return absl::OkStatus();
};

// static
absl::Status HalADIOPart::AddDigitalInputsToConfig(
    const Context& context,
    const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config,
    const HardwareModuleManager* hardware_module_manager,
    HalADIOPart::PartConfig& config) {
  std::vector<std::string> digital_input_names;
  absl::flat_hash_set<std::string> digital_input_raw_interface_names;
  // Configures all Digital inputs.
  for (const auto& block_config : adio_part_config.digital_inputs()) {
    const auto& module_name = block_config.interface().module_name();
    const auto& interface_name = block_config.interface().interface_name();
    std::string digital_input_raw_interface_name =
        absl::StrCat(module_name, "_", interface_name);
    if (digital_input_raw_interface_names.contains(
            digital_input_raw_interface_name)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Duplicate digital input: ", digital_input_raw_interface_name));
    }
    digital_input_raw_interface_names.insert(digital_input_raw_interface_name);
    INTR_ASSIGN_OR_RETURN(
        std::string export_name,
        ExportName(module_name, interface_name, block_config));
    if (absl::c_find(digital_input_names, export_name) !=
        digital_input_names.end()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Duplicate digital input export name: ", export_name));
    };
    digital_input_names.push_back(export_name);
    DigitalInputBlockData block_data;
    INTR_ASSIGN_OR_RETURN(
        block_data.input_handle,
        context.GetHardwareInterfaceHandle<intrinsic_fbs::DIOStatus>(
            module_name, interface_name));
    size_t size = block_data.input_handle->signals()->size();
    if (size > DioBlock::kMaxValuesPerBlock) {
      return absl::FailedPreconditionError(
          absl::StrCat("The DigitalInputBlock '", export_name, "' defines ",
                       size, " values. This part can only handle ",
                       DioBlock::kMaxValuesPerBlock, "."));
    }
    LOG(INFO) << "Configuring " << module_name << ":" << interface_name
              << " as " << export_name << " with " << size << " signals";
    INTRINSIC_RT_ASSIGN_OR_RETURN(block_data.block, DioBlock::Create(size));

    // Copy the signal names.
    for (size_t signal_idx = 0; signal_idx < size; ++signal_idx) {
      const auto* signal = block_data.input_handle->signals()->Get(signal_idx);
      if (signal == nullptr) {
        return absl::FailedPreconditionError(
            absl::StrCat("Invalid DigitalInputBlock '", export_name,
                         "' at index ", signal_idx, ". Block is nullptr."));
      }
      if (signal->name() == nullptr) {
        return absl::FailedPreconditionError(absl::StrCat(
            "Invalid DigitalInputBlock '", export_name, "' at index ",
            signal_idx, ". signal_name is nullptr."));
      }
      block_data.signal_names.push_back(
          std::string(signal->name()->string_view()));
    }
    // emplace should always succeed here since we checked for duplicates
    // before.
    config.digital_inputs.emplace(export_name, std::move(block_data));
    if (const HardwareModuleProxy* module_proxy =
            hardware_module_manager->GetHardwareModuleProxy(module_name);
        module_proxy == nullptr) {
      return absl::FailedPreconditionError(absl::StrCat(
          "Failed to get HardwareModuleProxy for interface: ", module_name));
    } else {
      // Only required once, fine to add multiple times.
      config.hardware_modules.emplace(std::move(module_proxy));
    }
  }
  config.digital_input_names = std::move(digital_input_names);

  return absl::OkStatus();
};

// static
absl::Status HalADIOPart::AddDigitalOutputsToConfig(
    const Context& context,
    const intrinsic_proto::icon::HalADIOPartConfig& adio_part_config,
    const HardwareModuleManager* hardware_module_manager,
    HalADIOPart::PartConfig& config) {
  std::vector<std::string> digital_output_names;
  absl::flat_hash_set<std::string> digital_output_raw_interface_names;
  // Configures all Digital outputs.
  for (const auto& block_config : adio_part_config.digital_outputs()) {
    const auto& module_name = block_config.interface().module_name();
    const auto& interface_name = block_config.interface().interface_name();
    std::string digital_output_raw_interface_name =
        absl::StrCat(module_name, "_", interface_name);
    if (digital_output_raw_interface_names.contains(
            digital_output_raw_interface_name)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Duplicate digital output: ", digital_output_raw_interface_name));
    }
    digital_output_raw_interface_names.insert(
        digital_output_raw_interface_name);
    INTR_ASSIGN_OR_RETURN(
        std::string export_name,
        ExportName(module_name, interface_name, block_config));
    if (absl::c_find(digital_output_names, export_name) !=
        digital_output_names.end()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Duplicate digital output export name: ", export_name));
    };
    digital_output_names.push_back(export_name);
    DigitalOutputBlockData block_data;
    INTR_ASSIGN_OR_RETURN(
        block_data.output_handle,
        context.GetMutableHardwareInterfaceHandle<intrinsic_fbs::DIOCommand>(
            module_name, interface_name));

    size_t size = block_data.output_handle->signals()->size();
    if (size > DioBlock::kMaxValuesPerBlock) {
      return absl::FailedPreconditionError(
          absl::StrCat("The DigitalOutputBlock '", export_name, "' defines ",
                       size, " values. This part can only handle ",
                       DioBlock::kMaxValuesPerBlock, "."));
    }
    LOG(INFO) << "Configuring " << module_name << ":" << interface_name
              << " as " << export_name << " with " << size << " signals";
    INTRINSIC_RT_ASSIGN_OR_RETURN(block_data.block, DioBlock::Create(size));

    // Copy the signal names.
    for (size_t signal_idx = 0; signal_idx < size; ++signal_idx) {
      const auto* signal = block_data.output_handle->signals()->Get(signal_idx);
      if (signal == nullptr) {
        return absl::FailedPreconditionError(
            absl::StrCat("Invalid DigitalOutputBlock '", export_name,
                         "' at index ", signal_idx, ". Block is nullptr."));
      }
      if (signal->name() == nullptr) {
        return absl::FailedPreconditionError(absl::StrCat(
            "Invalid DigitalOutputBlock '", export_name, "' at index ",
            signal_idx, ". signal_name is nullptr."));
      }
      block_data.signal_names.push_back(
          std::string(signal->name()->string_view()));
    }
    // emplace should always succeed here since we checked for duplicates
    // before.
    config.digital_outputs.emplace(export_name, std::move(block_data));
    if (const HardwareModuleProxy* module_proxy =
            hardware_module_manager->GetHardwareModuleProxy(module_name);
        module_proxy == nullptr) {
      return absl::FailedPreconditionError(absl::StrCat(
          "Failed to get HardwareModuleProxy for interface: ", module_name));
    } else {
      // Only required once, fine to add multiple times.
      config.hardware_modules.emplace(std::move(module_proxy));
    }
  }
  config.digital_output_names = std::move(digital_output_names);
  return absl::OkStatus();
};

RealtimeStatus HalADIOPart::ApplyCommand(ApplyCommandParameters params) {
  // Writes the data from the local buffers into the HardwareInterfaces.
  for (auto& [name, block] : config_.digital_outputs) {
    for (size_t i = 0; i < block.output_handle->signals()->size(); ++i) {
      block.output_handle->mutable_signals()->GetMutableObject(i)->mutate_value(
          block.block.Values()[i]);
    }
    block.output_handle.UpdatedAt(Clock::now());
  }
  for (auto& [name, block] : config_.analog_outputs) {
    for (size_t i = 0; i < block.output_handle->signals()->size(); ++i) {
      block.output_handle->mutable_signals()->GetMutableObject(i)->mutate_value(
          block.block.Values()[i]);
    }
    block.output_handle.UpdatedAt(Clock::now());
  }
  return OkStatus();
}

RealtimeStatus HalADIOPart::ReadStatus(ReadStatusParameters params) {
  // Reads the data from the HardwareInterfaces into our buffers.
  for (auto& [name, block] : config_.analog_inputs) {
    for (size_t i = 0; i < block.input_handle->signals()->size(); ++i) {
      block.block.MutableValues()[i] =
          block.input_handle->signals()->Get(i)->value();
    }
  }
  for (auto& [name, block] : config_.digital_inputs) {
    for (size_t i = 0; i < block.input_handle->signals()->size(); ++i) {
      block.block.MutableValues()[i] =
          block.input_handle->signals()->Get(i)->value();
    }
  }
  // Not reading the digital_outputs, as they are only used as commands and
  // reading could lead to feedback loops (e.g. if the hardware only applies a
  // command after n-ticks).
  // A module can however represent the actual state of the outputs by defining
  // an additional input.

  return OkStatus();
}

ADIO::ADIOState HalADIOPart::GetADIOState() const {
  ADIO::ADIOState state;
  state.analog_input_block_names = config_.analog_input_names;
  state.analog_output_block_names = config_.analog_output_names;
  state.digital_input_block_names = config_.digital_input_names;
  state.digital_output_block_names = config_.digital_output_names;

  // Not explicitly checking against `kMaxBlocks`, as the number of block
  // names is verified on creation of the part.
  for (const auto& name : config_.analog_input_names) {
    auto iter = config_.analog_inputs.find(name);
    // The part configuration is broken if this lookup ever fails.
    if (iter != config_.analog_inputs.end()) {
      state.analog_inputs.emplace_back(iter->second.block);
    } else {
      INTRINSIC_RT_LOG(ERROR) << "Failed to find analog input block: " << name;
    }
  }
  for (const auto& name : config_.analog_output_names) {
    auto iter = config_.analog_outputs.find(name);
    // The part configuration is broken if this lookup ever fails.
    if (iter != config_.analog_outputs.end()) {
      state.analog_outputs.emplace_back(iter->second.block);
    } else {
      INTRINSIC_RT_LOG(ERROR) << "Failed to find analog output block: " << name;
    }
  }

  for (const auto& name : config_.digital_input_names) {
    auto iter = config_.digital_inputs.find(name);
    // The part configuration is broken if this lookup ever fails.
    if (iter != config_.digital_inputs.end()) {
      state.digital_inputs.emplace_back(iter->second.block);
    } else {
      INTRINSIC_RT_LOG(ERROR) << "Failed to find digital input block: " << name;
    }
  }

  for (const auto& name : config_.digital_output_names) {
    auto iter = config_.digital_outputs.find(name);
    // The part configuration is broken if this lookup ever fails.
    if (iter != config_.digital_outputs.end()) {
      state.digital_outputs.emplace_back(iter->second.block);
    } else {
      INTRINSIC_RT_LOG(ERROR) << "Failed to find analog input block: " << name;
    }
  }
  return state;
}

absl::Span<const std::string> HalADIOPart::DigitalInputBlockNames() const {
  return config_.digital_input_names;
}
absl::Span<const std::string> HalADIOPart::DigitalOutputBlockNames() const {
  return config_.digital_output_names;
}
absl::Span<const std::string> HalADIOPart::AnalogInputBlockNames() const {
  return config_.analog_input_names;
}
absl::Span<const std::string> HalADIOPart::AnalogOutputBlockNames() const {
  return config_.analog_output_names;
}

const DioBlock* HalADIOPart::DigitalInputBlock(absl::string_view name) const {
  auto it = config_.digital_inputs.find(name);
  if (it == config_.digital_inputs.end()) return nullptr;
  return &it->second.block;
}

absl::Span<const std::string> HalADIOPart::DigitalInputSignalNames(
    absl::string_view block_name) const {
  auto it = config_.digital_inputs.find(block_name);
  if (it == config_.digital_inputs.end()) return {};
  return it->second.signal_names;
}

DioBlock* HalADIOPart::MutableDigitalOutputBlock(absl::string_view name) {
  auto it = config_.digital_outputs.find(name);
  if (it == config_.digital_outputs.end()) return nullptr;
  return &it->second.block;
}

absl::Span<const std::string> HalADIOPart::DigitalOutputSignalNames(
    absl::string_view block_name) const {
  auto it = config_.digital_outputs.find(block_name);
  if (it == config_.digital_outputs.end()) return {};
  return it->second.signal_names;
}

const AnalogBlock* HalADIOPart::AnalogInputBlock(absl::string_view name) const {
  auto it = config_.analog_inputs.find(name);
  if (it == config_.analog_inputs.end()) return nullptr;
  return &it->second.block;
}

AnalogBlock* HalADIOPart::MutableAnalogOutputBlock(absl::string_view name) {
  auto it = config_.analog_outputs.find(name);
  if (it == config_.analog_outputs.end()) return nullptr;
  return &it->second.block;
}

absl::Span<const std::string> HalADIOPart::AnalogInputSignalNames(
    absl::string_view block_name) const {
  auto it = config_.analog_inputs.find(block_name);
  if (it == config_.analog_inputs.end()) return {};
  return it->second.signal_names;
}

absl::Span<const std::string> HalADIOPart::AnalogOutputSignalNames(
    absl::string_view block_name) const {
  auto it = config_.analog_outputs.find(block_name);
  if (it == config_.analog_outputs.end()) return {};
  return it->second.signal_names;
}

}  // namespace intrinsic::icon
