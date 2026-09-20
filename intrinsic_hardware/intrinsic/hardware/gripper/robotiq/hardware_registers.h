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

#ifndef INTRINSIC_HARDWARE_GRIPPER_ROBOTIQ_HARDWARE_REGISTERS_H_
#define INTRINSIC_HARDWARE_GRIPPER_ROBOTIQ_HARDWARE_REGISTERS_H_

#include <cstdint>

namespace intrinsic::gripper::robotiq {

// Returns true if the gripper indicates that an object is detected given the
// value of the Status hardware register.
bool ObjectDetected(uint8_t status);

}  // namespace intrinsic::gripper::robotiq

#endif  // INTRINSIC_HARDWARE_GRIPPER_ROBOTIQ_HARDWARE_REGISTERS_H_
