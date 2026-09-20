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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_FACTORY_COMMON_H_
#define INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_FACTORY_COMMON_H_

#include <memory>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/part_property_registry.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic::icon {

struct PartFactoryContext {
  absl::string_view part_name;
  const Context& context;
  PartPropertyRegistry& property_registry;
  double control_frequency_hz;
  // Name of the Intrinsic resource instance associated with this Part, if there
  // is one. If the part is not associated with a particular resource, then this
  // is the resource name of the ICON instance.
  absl::string_view hardware_resource_name;
};

struct PartPtrAndGenericConfig {
  std::unique_ptr<RealtimePartInterface> part_ptr;
  intrinsic_proto::icon::GenericPartConfig config;
};

struct PartPtrAndPartConfig {
  std::unique_ptr<RealtimePartInterface> part_ptr;
  intrinsic_proto::icon::v1::PartConfig config;
};

// Extracts generic part config information from `feature_interfaces`, where
// possible.
//
// Note that this function cannot generate fully valid PartConfig protos for all
// feature interfaces. For example, the config for the JointVelocity feature
// interface contains the number of joints, but the feature interface itself has
// no way of getting that information.
//
// In these cases, we attempt to fall back to information from the
// JointPositionSensor feature interface, on the assumption that any part that
// has joint based interfaces will probably offer that as well.
intrinsic_proto::icon::GenericPartConfig ExtractGenericConfig(
    const FeatureInterfaceRegistry& feature_interfaces);

absl::Status ValidateGenericConfig(
    absl::flat_hash_set<::intrinsic_proto::icon::v1::FeatureInterfaceTypes>
        feature_interfaces,
    const intrinsic_proto::icon::GenericPartConfig& generic_config);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_REALTIME_PART_FACTORY_COMMON_H_
