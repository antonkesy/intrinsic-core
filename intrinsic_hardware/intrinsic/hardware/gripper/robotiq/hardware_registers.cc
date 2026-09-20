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

#include "intrinsic/hardware/gripper/robotiq/hardware_registers.h"

#include <cassert>
#include <cstdint>

#include "absl/log/log.h"

namespace intrinsic::gripper::robotiq {

namespace {

/*
Reference:
https://assets.robotiq.com/website-assets/support_documents/document/online/Hand-E_Instruction_Manual_e-Series_Web_20190924.zip/Hand-E_Instruction_Manual_e-Series_Web/Content/4.%20Control.htm

Status register byte:
|------+--------------------------------------------------------------|
| Bits | Status fields                                                |
|------+--------------------------------------------------------------|
| 0    | Activation status (gACT):                                    |
|      | 0x0: Gripper reset                                           |
|      | 0x1: Gripper activation                                      |
|------+--------------------------------------------------------------|
| 1,2  | Reserved                                                     |
|------+--------------------------------------------------------------|
| 3    | Action status (gGTO):                                        |
|      | 0x0: Stopped (or performing activation / automatic release)  |
|      | 0x1: Go to position request                                  |
|------+--------------------------------------------------------------|
| 4, 5 | Gripper Status (gSTA):                                       |
|      | 0x00: Gripper is in reset (or automatic release) state       |
|      | 0x01: Activation in progress                                 |
|      | 0x02: Not used                                               |
|      | 0x03: Activation is complete                                 |
|------+--------------------------------------------------------------|
| 6, 7 | Object detection status (gOBJ), ignore if ActionStatus == 0  |
|      | 0x00: No object detected. Fingers are in motion towards      |
|      |       requested position.                                    |
|      | 0x01: Object detected. Fingers have stopped due to contact   |
|      |       while opening before requested position.               |
|      | 0x02: Object detected. Fingers have stopped due to contact   |
|      |       while closing before requested position.               |
|      | 0x03: No object detected. Fingers are at requested position. |
|------+--------------------------------------------------------------|

*/

// Number of left shift bits for the different status fields.
constexpr int kActivationStatusBitShift = 0;
constexpr int kActionStatusBitShift = 3;
constexpr int kGripperStatusBitShift = 4;
constexpr int kObjectDetectionStatusBitShift = 6;

// Bit masks for different status fields in the Status byte register.
constexpr uint8_t kActivationStatusMask = 0x01 << kActivationStatusBitShift;
constexpr uint8_t kActionStatusMask = 0x01 << kActionStatusBitShift;
constexpr uint8_t kGripperStatusMask = 0x03 << kGripperStatusBitShift;
constexpr uint8_t kObjectDetectionMask = 0x03 << kObjectDetectionStatusBitShift;

// Sanity checks.
static_assert(kActivationStatusMask == 0x01);
static_assert(kActionStatusMask == 0x08);
static_assert(kGripperStatusMask == 0x30);
static_assert(kObjectDetectionMask == 0xc0);

// Zeros out all the bits except for the lowest two bits.
static uint8_t LowestTwoBits(const uint8_t val) { return val & 0x03; }

// Returns true if the action status is stopped (i.e. there's no active request
// to move the gripper fingers to a desired position).
static bool ActionStatusIsStopped(const uint8_t status) {
  return (status & kActionStatusMask) == 0;
}

// Returns the value (within range [0, 3]) of the object detection status.
static uint8_t ObjectDetectionStatusValue(const uint8_t status) {
  return LowestTwoBits((status & kObjectDetectionMask) >>
                       kObjectDetectionStatusBitShift);
}

}  // namespace

bool ObjectDetected(const uint8_t status) {
  // Object detection is valid only if Action status is not stopped.
  if (ActionStatusIsStopped(status)) {
    return false;
  }

  switch (ObjectDetectionStatusValue(status)) {
    case 0x00:
      // No object detected. Fingers are in motion.
      return false;
    case 0x01:
      // Object detected while opening before requested position.
      return true;
    case 0x02:
      // Object detected while closing before requested position.
      return true;
    case 0x03:
      // No object detected. Fingers have reached requested position.
      return false;
    default:
      LOG(FATAL) << "Status value should be <= 3";
  }
}

}  // namespace intrinsic::gripper::robotiq
