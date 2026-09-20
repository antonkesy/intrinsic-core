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

#include "intrinsic/icon/control/parts/feature_interfaces/cartesian_limits.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/parts/proto/v1/mode_of_safe_operation_limits_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/proto/safety_status.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {
using ::intrinsic_fbs::EnumNameModeOfSafeOperation;
using ::intrinsic_fbs::EnumValuesModeOfSafeOperation;
using ::intrinsic_fbs::ModeOfSafeOperation;

absl::Status FailForWrongVectorSize(const std::vector<double>& limit_vector) {
  if (limit_vector.size() != 3) {
    return absl::InvalidArgumentError(
        absl::StrCat("The limit vector provided is of size ",
                     limit_vector.size(), " and should be size 3."));
  }
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<CartesianLimitsFeature> CartesianLimitsFeature::Create(
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
    const intrinsic_proto::icon::v1::ModeOfSafeOperationLimitsConfig& config) {
  INTR_RETURN_IF_ERROR(FailForWrongVectorSize(min_translational_position));
  INTR_RETURN_IF_ERROR(FailForWrongVectorSize(max_translational_position));
  INTR_RETURN_IF_ERROR(FailForWrongVectorSize(min_translational_velocity));
  INTR_RETURN_IF_ERROR(FailForWrongVectorSize(max_translational_velocity));
  INTR_RETURN_IF_ERROR(FailForWrongVectorSize(min_translational_acceleration));
  INTR_RETURN_IF_ERROR(FailForWrongVectorSize(max_translational_acceleration));
  INTR_RETURN_IF_ERROR(FailForWrongVectorSize(min_translational_jerk));
  INTR_RETURN_IF_ERROR(FailForWrongVectorSize(max_translational_jerk));

  CartesianLimits limits;
  limits.min_translational_position =
      VectorToVectorNd(min_translational_position);
  limits.max_translational_position =
      VectorToVectorNd(max_translational_position);
  limits.min_translational_velocity =
      VectorToVectorNd(min_translational_velocity);
  limits.max_translational_velocity =
      VectorToVectorNd(max_translational_velocity);
  limits.min_translational_acceleration =
      VectorToVectorNd(min_translational_acceleration);
  limits.max_translational_acceleration =
      VectorToVectorNd(max_translational_acceleration);
  limits.min_translational_jerk = VectorToVectorNd(min_translational_jerk);
  limits.max_translational_jerk = VectorToVectorNd(max_translational_jerk);
  limits.max_rotational_velocity = max_rotational_velocity;
  limits.max_rotational_acceleration = max_rotational_acceleration;
  limits.max_rotational_jerk = max_rotational_jerk;

  return Create(limits, config);
}

absl::StatusOr<CartesianLimitsFeature> CartesianLimitsFeature::Create(
    const CartesianLimits& limits,
    const intrinsic_proto::icon::v1::ModeOfSafeOperationLimitsConfig& config) {
  if (!limits.IsValid()) {
    return absl::InvalidArgumentError(
        "Cannot initialize CartesianLimitFeature with an invalid limits.");
  }

  absl::flat_hash_map<ModeOfSafeOperation, CartesianLimits> limits_map = {
      {ModeOfSafeOperation::UNKNOWN, limits}};

  if (config.has_t1_limits()) {
    // The limits for ModeOfSafeOperation::UNKNOWN are the default set.
    CartesianLimits t1_limits = limits_map.at(ModeOfSafeOperation::UNKNOWN);
    // Assigns the minimal value of the max_velocity and the t1_limit for every
    // dof. Assumes that max_velocity is positive.
    t1_limits.max_translational_velocity =
        t1_limits.max_translational_velocity.cwiseMin(
            config.t1_limits().tcp_max_translational_velocity());
    t1_limits.min_translational_velocity =
        t1_limits.min_translational_velocity.cwiseMax(
            -config.t1_limits().tcp_max_translational_velocity());
    t1_limits.max_rotational_velocity =
        std::min(t1_limits.max_rotational_velocity,
                 config.t1_limits().tcp_max_rotational_velocity());
    limits_map[ModeOfSafeOperation::TEACH_PENDANT_1] = t1_limits;
  }
  CartesianLimits fallback_limits = limits_map.at(ModeOfSafeOperation::UNKNOWN);
  // Fill in the remaining limits for all 'ModeOfSafeOperation's in
  // intrinsic/icon/control/safety/safety_messages.fbs
  for (const auto mode : EnumValuesModeOfSafeOperation()) {
    auto it = limits_map.find(mode);
    if (it == limits_map.end()) {
      LOG(WARNING) << "No explicit limits defined for ModeOfSafeOperation::"
                   << EnumNameModeOfSafeOperation(mode)
                   << " Using fallback limits.";
      limits_map[mode] = fallback_limits;
    }
  }

  std::vector<CartesianLimits> limits_vector(limits_map.size());
  for (const auto mode : EnumValuesModeOfSafeOperation()) {
    limits_vector[static_cast<uint8_t>(mode)] = limits_map[mode];
  }

  // Ensures that all cases of ModeOfSafeOperation in
  // intrinsic/icon/control/safety/safety_messages.fbs are covered.
  QCHECK(limits_vector.size() ==
         static_cast<uint8_t>(ModeOfSafeOperation::MAX) + 1)
      << "There needs to be a LimitBundle for every ModeOfSafeOperation. And "
         "the enum values need to be sequential starting with zero.";

  return CartesianLimitsFeature(std::move(limits_vector));
}

absl::StatusOr<CartesianLimitsFeature> CartesianLimitsFeature::Create(
    const intrinsic_proto::icon::v1::ModeOfSafeOperationLimitsConfig& config) {
  if (!config.has_t1_limits()) {
    return absl::InvalidArgumentError(
        "Cannot initialize CartesianLimitFeature from safe operation limits "
        "without t1 limits.");
  }

  CartesianLimits cart_limits = CreateSimpleCartesianLimits(
      /*max_translational_position=*/std::numeric_limits<double>::max(),
      /*max_translational_velocity=*/
      config.t1_limits().tcp_max_translational_velocity(),
      /*max_translational_acceleration=*/std::numeric_limits<double>::max(),
      /*max_translational_jerk=*/std::numeric_limits<double>::max(),
      /*max_rotational_velocity=*/
      config.t1_limits().tcp_max_rotational_velocity(),
      /*max_rotational_acceleration=*/std::numeric_limits<double>::max(),
      /*max_rotational_jerk=*/std::numeric_limits<double>::max());

  return Create(std::move(cart_limits), config);
}

CartesianLimitsFeature::CartesianLimitsFeature(
    absl::Span<const CartesianLimits> limits_in_enum_order)
    : limits_in_enum_order_(
          {limits_in_enum_order.begin(), limits_in_enum_order.end()}) {}

CartesianLimits CartesianLimitsFeature::GetDefaultCartesianLimits() const {
  return GetLimitsForModeOfSafeOperation(
      current_safety_status_.mode_of_safe_operation);
}

RealtimeStatus CartesianLimitsFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  current_safety_status_ = params.safety_status;
  return OkStatus();
}

const CartesianLimits& CartesianLimitsFeature::GetLimitsForModeOfSafeOperation(
    intrinsic_fbs::ModeOfSafeOperation mode) const {
  // Construction ensures that there is a Cartesian Limit for every
  // ModeOfSafeOperation.
  return limits_in_enum_order_.at(static_cast<uint8_t>(mode));
}

}  // namespace intrinsic::icon
