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

#ifndef INTRINSIC_ICON_CONTROL_PRIMITIVES_FORCE_CONTROL_FORCE_PRIMITIVE_UTILS_H_
#define INTRINSIC_ICON_CONTROL_PRIMITIVES_FORCE_CONTROL_FORCE_PRIMITIVE_UTILS_H_

#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/force_primitive.pb.h"
#include "intrinsic/icon/control/primitives/force_control/proto/force_primitives.pb.h"
#include "intrinsic/icon/equipment/force_control_settings.pb.h"
#include "intrinsic/manipulation/skills/force/contact_stiffness.pb.h"

namespace intrinsic::icon::force_primitive {

// Converts a Direction proto to a unit direction vector.
//
// Returns an error if the proto does not contain a valid TranslationalAxis or a
// vector with a zero norm.
absl::StatusOr<eigenmath::Vector3d> UnitDirectionFromProto(
    const intrinsic_proto::force::Direction& direction);

// Returns a virtual inertia matrix from the given ForceControlSettings.
eigenmath::Matrix6d VirtualInertiaFromSettings(
    const intrinsic_proto::icon::ForceControlSettings& settings);

// Returns the environment stiffness from the given ContactStiffnessParams.
//
// Returns an error if the params do not contain a valid contact stiffness.
absl::StatusOr<double> GetEnvironmentStiffness(
    const intrinsic_proto::manipulation::skills::ContactStiffnessParams&
        params);

// Returns the action type name for the given ForcePrimitive. Either the
// force_primitive or the force_trajectory_primitive action are used, depending
// on the primitive type.
//
// Returns an error if the primitive is not supported.
absl::StatusOr<std::string> GetActionTypeName(
    const intrinsic_proto::force::AllForcePrimitives& primitive);

absl::Status SetForcePrimitive(
    const intrinsic_proto::force::AllForcePrimitives& primitive,
    intrinsic_proto::icon::actions::proto::ForcePrimitiveFixedParams&
        action_params);

}  // namespace intrinsic::icon::force_primitive

#endif  // INTRINSIC_ICON_CONTROL_PRIMITIVES_FORCE_CONTROL_FORCE_PRIMITIVE_UTILS_H_
