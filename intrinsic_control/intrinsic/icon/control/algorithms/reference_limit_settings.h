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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_REFERENCE_LIMIT_SETTINGS_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_REFERENCE_LIMIT_SETTINGS_H_

namespace intrinsic::icon {

struct ReferenceLimitSettings {
  // Absolute maximum distance between reference pose and sensed pose
  // translation.
  double max_translation;
  // Absolute maximum distance between reference pose and sensed pose
  // rotation.
  double max_rotation;
  // Must be in [0, 1].
  // When the distance between sensed and reference pose translation crosses
  // activation_ratio * max_translation, the reference state is smoothly scaled
  // to stay within max_translation from the sensed state and reference
  // velocities are smoothly scaled towards zero.
  double activation_ratio;
};

struct ReferenceJointLimitSettings {
  // Absolute maximum distance between reference joint angle and sensed joint
  // angle (in radians).
  double max_distance;
  // Must be in [0, 1].
  // When the distance between sensed and reference joint angle crosses
  // activation_ratio * max_distance, the reference state is smoothly scaled to
  // stay within max_distance from the sensed state and reference velocities are
  // smoothly scaled towards zero.
  double activation_ratio;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_REFERENCE_LIMIT_SETTINGS_H_
