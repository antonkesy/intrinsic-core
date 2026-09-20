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

#ifndef INTRINSIC_WORLD_CONFIGURATION_VALIDATION_H_
#define INTRINSIC_WORLD_CONFIGURATION_VALIDATION_H_

#include <functional>

#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Validator bit-flag options. Options can be set independently, by using the
// bitwise OR operator. For example: (kUseVariableLimits | kUseRcsLimits)
enum ConfigurationValidatorOptions {
  kAlwaysValid = 0x00,
  kUseVariableLimits = 0x01,
  kUseSimplifiedPathPlanningLimits = 0x02,
  kUseRcsLimits = 0x04,
};

// For a given dof configuration, returns if it is considered valid.
using ConfigurationValidator = std::function<bool(const eigenmath::VectorXd&)>;

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_CONFIGURATION_VALIDATION_H_
