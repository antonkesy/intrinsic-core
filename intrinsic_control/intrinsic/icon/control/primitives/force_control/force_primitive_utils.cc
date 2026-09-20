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

#include "intrinsic/icon/control/primitives/force_control/force_primitive_utils.h"

#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/force_primitive.pb.h"
#include "intrinsic/icon/actions/force_primitive_info.h"
#include "intrinsic/icon/control/primitives/force_control/proto/force_primitives.pb.h"
#include "intrinsic/icon/equipment/force_control_settings.pb.h"
#include "intrinsic/manipulation/skills/force/contact_stiffness.pb.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon::force_primitive {

// Same values as in
// intrinsic/manipulation/skills/force/contact_stiffness_params.py
inline constexpr double kDefaultLowStiffness = 5000.0;
inline constexpr double kDefaultMediumStiffness = 60000.0;
inline constexpr double kDefaultHighStiffness = 100000.0;

using ::intrinsic_proto::force::Direction;
using ::intrinsic_proto::icon::ForceControlSettings;
using ::intrinsic_proto::manipulation::skills::ContactStiffnessParams;
using ::intrinsic_proto::manipulation::skills::ContactStiffnessValues;

absl::StatusOr<eigenmath::Vector3d> ToVector(
    const Direction::TranslationalAxis& axis) {
  switch (axis) {
    case Direction::X:
      return eigenmath::Vector3d::UnitX();
    case Direction::Y:
      return eigenmath::Vector3d::UnitY();
    case Direction::Z:
      return eigenmath::Vector3d::UnitZ();
    case Direction::MINUS_X:
      return -eigenmath::Vector3d::UnitX();
    case Direction::MINUS_Y:
      return -eigenmath::Vector3d::UnitY();
    case Direction::MINUS_Z:
      return -eigenmath::Vector3d::UnitZ();
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Invalid translational axis: ", axis));
  }
}

absl::StatusOr<eigenmath::Vector3d> UnitDirectionFromProto(
    const intrinsic_proto::force::Direction& direction) {
  eigenmath::Vector3d motion_direction;

  switch (direction.direction_case()) {
    case Direction::kAxis: {
      INTR_ASSIGN_OR_RETURN(motion_direction, ToVector(direction.axis()));
      break;
    }
    case Direction::kVector:
      motion_direction = FromProto(direction.vector());
      break;
    default:
      return absl::InvalidArgumentError(
          absl::StrCat("Invalid direction: ", direction.direction_case()));
  }

  if (AlmostEquals(motion_direction.norm(), 0.0)) {
    return absl::InvalidArgumentError(
        absl::StrCat("Motion direction cannot have a zero norm. Direction: ",
                     absl::StrJoin(motion_direction, ", ")));
  }
  return motion_direction.normalized();
}

eigenmath::Matrix6d VirtualInertiaFromSettings(
    const ForceControlSettings& settings) {
  eigenmath::Matrix6d virtual_inertia = eigenmath::Matrix6d::Zero();
  virtual_inertia.diagonal().head<3>().setConstant(
      settings.virtual_translational_inertia());
  virtual_inertia.diagonal().tail<3>().setConstant(
      settings.virtual_rotational_inertia());
  return virtual_inertia;
}

absl::StatusOr<double> GetEnvironmentStiffness(
    const ContactStiffnessParams& params) {
  switch (params.contact_stiffness_case()) {
    case ContactStiffnessParams::kMechanicalStiffnessAtContact:
      return params.mechanical_stiffness_at_contact();
    case ContactStiffnessParams::kDefaultContactStiffness: {
      switch (params.default_contact_stiffness()) {
        case ContactStiffnessValues::CONTACT_STIFFNESS_LOW:
          return kDefaultLowStiffness;
        case ContactStiffnessValues::CONTACT_STIFFNESS_MEDIUM:
          return kDefaultMediumStiffness;
        case ContactStiffnessValues::CONTACT_STIFFNESS_HIGH:
          return kDefaultHighStiffness;
        default:
          return absl::InvalidArgumentError(
              "No default contact stiffness found in the parameters.");
      }
    }
    case ContactStiffnessParams::CONTACT_STIFFNESS_NOT_SET:
      return absl::InvalidArgumentError(
          "No environment stiffness set in the parameters.");
  }
  return absl::InternalError(
      absl::StrCat("contact_stiffness contains an invalid value: ",
                   params.contact_stiffness_case()));
}

absl::StatusOr<std::string> GetActionTypeName(
    const intrinsic_proto::force::AllForcePrimitives& primitive) {
  if (primitive.has_apply_force() ||
      primitive.has_align_rotation_with_frame() ||
      primitive.has_make_contact() || primitive.has_hold()) {
    return ForcePrimitiveInfo::kActionTypeName;
  }
  return absl::InvalidArgumentError(
      absl::StrCat("Force primitive is not supported: ", primitive));
}

absl::Status SetForcePrimitive(
    const intrinsic_proto::force::AllForcePrimitives& primitive,
    intrinsic_proto::icon::actions::proto::ForcePrimitiveFixedParams&
        action_params) {
  switch (primitive.primitive_case()) {
    case intrinsic_proto::force::AllForcePrimitives::kMakeContact:
      *action_params.mutable_force_primitive()->mutable_make_contact() =
          primitive.make_contact();
      break;
    case intrinsic_proto::force::AllForcePrimitives::kApplyForce:
      *action_params.mutable_force_primitive()->mutable_apply_force() =
          primitive.apply_force();
      break;
    case intrinsic_proto::force::AllForcePrimitives::kAlignRotationWithFrame:
      *action_params.mutable_force_primitive()
           ->mutable_align_rotation_with_frame() =
          primitive.align_rotation_with_frame();
      break;
    case intrinsic_proto::force::AllForcePrimitives::kHold:
      *action_params.mutable_force_primitive()->mutable_hold() =
          primitive.hold();
      break;
    default:
      return absl::InvalidArgumentError(absl::StrCat(
          "Force primitive is not supported: ", primitive.primitive_case()));
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::icon::force_primitive
