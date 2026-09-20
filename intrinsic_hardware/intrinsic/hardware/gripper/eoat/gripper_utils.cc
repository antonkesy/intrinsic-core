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

#include "intrinsic/hardware/gripper/eoat/gripper_utils.h"

#include <algorithm>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/gpio/v1/signal.pb.h"
#include "intrinsic/hardware/gripper/eoat/eoat_service.pb.h"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"

namespace intrinsic {
namespace gripper {

absl::Status GpioSrvEndpointIsValid(
    const intrinsic_proto::eoat::GpioServiceEndpoint& srv_endpoint) {
  if (srv_endpoint.grpc_address().empty()) {
    return absl::InvalidArgumentError("GPIO grpc address not set.");
  }
  if (srv_endpoint.gpio_service_name().empty()) {
    return absl::InvalidArgumentError("GPIO service name not set.");
  }
  return absl::OkStatus();
}

absl::flat_hash_set<std::string> SignalsToClaim(
    const intrinsic_proto::eoat::PinchGripperConfig& config) {
  absl::flat_hash_set<std::string> signals;
  for (const auto& [name, value] : config.grasp().value_set().values()) {
    signals.insert(name);
  }

  for (const auto& [name, value] : config.release().value_set().values()) {
    signals.insert(name);
  }

  return signals;
}

absl::flat_hash_set<std::string> SignalsToRead(
    const intrinsic_proto::eoat::PinchGripperConfig& config) {
  absl::flat_hash_set<std::string> signals;
  for (const auto& [name, value] :
       config.gripping_indicated().value_set().values()) {
    signals.insert(name);
  }

  return signals;
}

absl::Status GripperConfigIsValid(
    const intrinsic_proto::eoat::PinchGripperConfig& config) {
  if (config.grasp().value_set().values().empty()) {
    return absl::InvalidArgumentError("Grasp Config not set");
  }
  if (config.release().value_set().values().empty()) {
    return absl::InvalidArgumentError("Release Config not set");
  }
  if (config.gripping_indicated().value_set().values().empty()) {
    return absl::InvalidArgumentError("GrippingIndicated Config not set");
  }

  return absl::OkStatus();
}

absl::flat_hash_set<std::string> SignalsToClaim(
    const intrinsic_proto::eoat::SuctionGripperConfig& config) {
  absl::flat_hash_set<std::string> signals;
  for (const auto& [name, value] : config.grasp().value_set().values()) {
    signals.insert(name);
  }

  for (const auto& [name, value] : config.release().value_set().values()) {
    signals.insert(name);
  }

  for (const auto& [name, value] : config.blowoff_on().value_set().values()) {
    signals.insert(name);
  }

  for (const auto& [name, value] : config.blowoff_off().value_set().values()) {
    signals.insert(name);
  }

  return signals;
}

absl::flat_hash_set<std::string> SignalsToRead(
    const intrinsic_proto::eoat::SuctionGripperConfig& config) {
  absl::flat_hash_set<std::string> signals;
  for (const auto& [name, value] :
       config.gripping_indicated().value_set().values()) {
    signals.insert(name);
  }

  return signals;
}

absl::Status GripperConfigIsValid(
    const intrinsic_proto::eoat::SuctionGripperConfig& config) {
  if (config.grasp().value_set().values().empty()) {
    return absl::InvalidArgumentError("Grasp Config not set");
  }
  if (config.release().value_set().values().empty()) {
    return absl::InvalidArgumentError("Release Config not set");
  }
  if (config.blowoff_on().value_set().values().empty()) {
    return absl::InvalidArgumentError("blowoff_on Config not set");
  }
  if (config.blowoff_off().value_set().values().empty()) {
    return absl::InvalidArgumentError("blowoff_off Config not set");
  }
  if (config.gripping_indicated().value_set().values().empty()) {
    return absl::InvalidArgumentError("GrippingIndicated Config not set");
  }

  return absl::OkStatus();
}

template <class Config>
static absl::Status SignalsAreValid(
    const Config& config,
    const intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse&
        valid_signals) {
  // `set_difference` requires ordered containers, so use `std::set` to store
  // these signal names.
  std::set<std::string> valid_read_signals;
  std::set<std::string> valid_write_signals;

  for (const auto& signal_desc : valid_signals.signal_descriptions()) {
    if (signal_desc.can_read()) {
      valid_read_signals.insert(signal_desc.signal_name());
      for (const auto& alternate_signal_name :
           signal_desc.alternate_signal_names()) {
        valid_read_signals.insert(alternate_signal_name);
      }
    }
    if (signal_desc.can_write()) {
      valid_write_signals.insert(signal_desc.signal_name());
      for (const auto& alternate_signal_name :
           signal_desc.alternate_signal_names()) {
        valid_write_signals.insert(alternate_signal_name);
      }
    }
  }

  // Returns a vector of strings (signal names) that are present in
  // `unordered_subset` but not in `superset`.
  auto set_diff = [](const absl::flat_hash_set<std::string>& unordered_subset,
                     const std::set<std::string>& superset) {
    const std::set<std::string> subset(unordered_subset.begin(),
                                       unordered_subset.end());
    std::vector<std::string> diff;
    std::set_difference(subset.begin(), subset.end(), superset.begin(),
                        superset.end(), std::inserter(diff, diff.begin()));
    return diff;
  };

  // Read/write signals names that are only present in the `config` but not in
  // valid read/write signals.
  const auto diff_read = set_diff(SignalsToRead(config), valid_read_signals);
  const auto diff_write = set_diff(SignalsToClaim(config), valid_write_signals);

  if (diff_read.empty() && diff_write.empty()) {
    return absl::OkStatus();
  }

  auto make_message = [](absl::string_view signal_type,
                         const std::vector<std::string>& invalid_signals,
                         const std::set<std::string>& valid_signals) {
    return absl::StrCat(
        "Invalid ", signal_type,
        " signal name(s) provided: ", absl::StrJoin(invalid_signals, ", "),
        ". Valid signal names are: ", absl::StrJoin(valid_signals, ", "), ".");
  };

  std::string error_message;
  if (!diff_read.empty()) {
    absl::StrAppend(&error_message,
                    make_message("read", diff_read, valid_read_signals), " ");
  }

  if (!diff_write.empty()) {
    absl::StrAppend(&error_message,
                    make_message("write", diff_write, valid_write_signals));
  }

  return absl::InvalidArgumentError(error_message);
}

absl::Status SignalsAreValid(
    const intrinsic_proto::eoat::PinchGripperConfig& config,
    const intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse&
        valid_signals) {
  return SignalsAreValid<>(config, valid_signals);
}

absl::Status SignalsAreValid(
    const intrinsic_proto::eoat::SuctionGripperConfig& config,
    const intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse&
        valid_signals) {
  return SignalsAreValid<>(config, valid_signals);
}

}  // namespace gripper
}  // namespace intrinsic
