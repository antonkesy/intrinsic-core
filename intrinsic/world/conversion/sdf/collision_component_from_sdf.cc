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

#include "intrinsic/world/conversion/sdf/collision_component_from_sdf.h"

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/collision_component.h"
#include "intrinsic/world/conversion/sdf/collision_surface_conversion.h"
#include "sdf/Link.hh"
#include "sdf/Surface.hh"

namespace intrinsic {
namespace sdf {

absl::StatusOr<std::unique_ptr<CollisionComponent>>
CollisionComponentFromSdfLink(const ::sdf::Link& sdf_link) {
  std::unique_ptr<CollisionComponent> collision_component =
      CollisionComponent::Create();

  INTR_ASSIGN_OR_RETURN(const ::sdf::Surface* surface,
                        GetSingleSurface(sdf_link));
  if (surface != nullptr) {
    INTR_ASSIGN_OR_RETURN(ParseSurfaceResult parse_surface_result,
                          ParseSurface(*surface));
    collision_component->SetCollisionResponse(
        !parse_surface_result.contact.collide_without_contact.value_or(false));
  }
  return collision_component;
}

}  // namespace sdf
}  // namespace intrinsic
