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

#ifndef INTRINSIC_ICON_DYNAMICS_VALIDATE_INERTIAL_PARAMETERS_H_
#define INTRINSIC_ICON_DYNAMICS_VALIDATE_INERTIAL_PARAMETERS_H_

#include "absl/status/status.h"
#include "intrinsic/kinematics/model_interface.h"

namespace intrinsic::icon {

// Validates that the inertial parameters defined in the `model` interface for
// all links are physically consistent. This routine checks that:
// 1) the link mass to be positive.
// 2) the link inertia expressed at the center of gravity to be positive
//    definite (symmetric and with positive eigenvalues) and that its
//    eigenvalues fulfill the triangle inequalities.
absl::Status ValidateInertialParameters(
    const kinematics::ModelInterface* model);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_DYNAMICS_VALIDATE_INERTIAL_PARAMETERS_H_
