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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_CARTESIAN_LIMITS_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_CARTESIAN_LIMITS_H_

#include <vector>

#include "absl/container/fixed_array.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/proto/v1/mode_of_safe_operation_limits_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"

namespace intrinsic::icon {

// Implementation of the CartesianLimitsFeatureInterface.
class CartesianLimitsFeature : public HalFeatureInterfaceBase,
                               public CartesianLimitsInterface {
 public:
  // Creates a new CartesianLimitsFeature instance.
  // Initializes each of the cartesian limits individually and additionally sets
  // the safe operation limits.
  static absl::StatusOr<CartesianLimitsFeature> Create(
      const std::vector<double>& min_translational_position,
      const std::vector<double>& max_translational_position,
      const std::vector<double>& min_translational_velocity,
      const std::vector<double>& max_translational_velocity,
      const std::vector<double>& min_translational_acceleration,
      const std::vector<double>& max_translational_acceleration,
      const std::vector<double>& min_translational_jerk,
      const std::vector<double>& max_translational_jerk,
      double max_rotational_velocity, double max_rotational_acceleration,
      double max_rotational_jerk,
      const intrinsic_proto::icon::v1::ModeOfSafeOperationLimitsConfig& config);

  // Creates a new CartesianLimitsFeature instance.
  // Initializes cartesian limits with the given set of limits and additionally
  // sets the safe operation limits.
  static absl::StatusOr<CartesianLimitsFeature> Create(
      const intrinsic::CartesianLimits& limits,
      const intrinsic_proto::icon::v1::ModeOfSafeOperationLimitsConfig& config);

  // Creates a new CartesianLimitsFeature instance.
  // Initializes cartesian limits with the safe operation mode limits.
  static absl::StatusOr<CartesianLimitsFeature> Create(
      const intrinsic_proto::icon::v1::ModeOfSafeOperationLimitsConfig& config);

  CartesianLimits GetDefaultCartesianLimits() const override;
  // We save the latest safety status during this call.
  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  const CartesianLimits& GetLimitsForModeOfSafeOperation(
      intrinsic_fbs::ModeOfSafeOperation mode) const;

 private:
  // Takes the span of Cartesian limits for each ModeOfSafeOperation from this
  // enum: intrinsic/icon/control/safety/extern/safety_status.fbs
  explicit CartesianLimitsFeature(
      absl::Span<const CartesianLimits> limits_in_enum_order);

  SafetyStatus current_safety_status_;

  const absl::FixedArray<CartesianLimits> limits_in_enum_order_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_CARTESIAN_LIMITS_H_
