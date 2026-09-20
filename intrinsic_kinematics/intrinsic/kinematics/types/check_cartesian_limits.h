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

#ifndef INTRINSIC_KINEMATICS_TYPES_CHECK_CARTESIAN_LIMITS_H_
#define INTRINSIC_KINEMATICS_TYPES_CHECK_CARTESIAN_LIMITS_H_

#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"

namespace intrinsic {

bool IsWithinLimits(const CartStateP& state, const CartesianLimits& limits);

bool IsWithinLimits(const CartStateV& state, const CartesianLimits& limits);

bool IsWithinAccLimits(const CartStateA& state, const CartesianLimits& limits);

bool IsWithinJerkLimits(const CartStateJ& state, const CartesianLimits& limits);

bool IsWithinLimits(const CartStatePV& state, const CartesianLimits& limits);

bool IsWithinLimits(const CartStatePVA& state, const CartesianLimits& limits);

bool IsWithinLimits(const CartStateVAJ& state, const CartesianLimits& limits);

bool IsWithinLimits(const CartesianLimits& limits_to_check,
                    const CartesianLimits& limits);

}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_TYPES_CHECK_CARTESIAN_LIMITS_H_
