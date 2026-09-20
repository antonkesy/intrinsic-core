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

#ifndef INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_ERROR_H_
#define INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_ERROR_H_

namespace intrinsic {
namespace collision {

// Enumeration for the error messages in collision library
enum class CollisionError {
  NO_ERROR = 0,
  INVALID_PLANE_GEOMETRY_INDEX = -100,
  INVALID_SPHERE_GEOMETRY_INDEX = -101,
  INVALID_CAPSULE_GEOMETRY_INDEX = -102,
  INVALID_COLLISION_OBJECT_INDEX = -103,
  INVALID_RADIUS_FOR_COLLISION_GEOMETRY = -104,
  INVALID_NUM_COLLISION_OBJECTS = -105
};

}  // namespace collision
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_CONTROL_COLLISION_COLLISION_ERROR_H_
