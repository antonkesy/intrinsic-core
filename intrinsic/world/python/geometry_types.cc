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


#include "intrinsic/world/geometry_types.h"

#include <pybind11/pybind11.h>

namespace intrinsic {

PYBIND11_MODULE(geometry_types, m) {
  m.attr("KIND_VISUAL_GEOMETRY") = pybind11::str(kKindVisualGeometry);
  m.attr("KIND_COLLISION_GEOMETRY") = pybind11::str(kKindCollisionGeometry);
}

}  // namespace intrinsic
