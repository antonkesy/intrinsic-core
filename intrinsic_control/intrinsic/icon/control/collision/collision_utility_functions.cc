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

#include "intrinsic/icon/control/collision/collision_utility_functions.h"

#include <cmath>

#include "intrinsic/icon/control/collision/collision_data_types.h"

namespace intrinsic {
namespace collision {

RotationMatrix GetRPYRotationMatrix(double roll, double pitch, double yaw) {
  RotationMatrix R_z = RotationMatrix::Identity();
  R_z(0, 0) = cos(yaw);
  R_z(0, 1) = -sin(yaw);
  R_z(1, 0) = sin(yaw);
  R_z(1, 1) = cos(yaw);
  RotationMatrix R_y = RotationMatrix::Identity();
  R_y(0, 0) = cos(pitch);
  R_y(0, 2) = sin(pitch);
  R_y(2, 0) = -sin(pitch);
  R_y(2, 2) = cos(pitch);
  RotationMatrix R_x = RotationMatrix::Identity();
  R_x(1, 1) = cos(roll);
  R_x(1, 2) = -sin(roll);
  R_x(2, 1) = sin(roll);
  R_x(2, 2) = cos(roll);
  return R_z * R_y * R_x;
}

}  // namespace collision
}  // namespace intrinsic
