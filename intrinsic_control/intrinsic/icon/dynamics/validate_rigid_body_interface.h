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

#ifndef INTRINSIC_ICON_DYNAMICS_VALIDATE_RIGID_BODY_INTERFACE_H_
#define INTRINSIC_ICON_DYNAMICS_VALIDATE_RIGID_BODY_INTERFACE_H_

#include "absl/status/status.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"

namespace intrinsic::icon {

// Validates that the `rigid_body_interface` is of the appropriate size as the
// input `sample`  and can be used to construct dynamic constraints such as
// torque limits.
absl::Status ValidateRigidBodyInterface(
    icon::RigidBodyInterface* rigid_body_interface,
    const eigenmath::VectorNd& q, const eigenmath::VectorNd& dq);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_DYNAMICS_VALIDATE_RIGID_BODY_INTERFACE_H_
