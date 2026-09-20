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

#ifndef INTRINSIC_WORLD_OBJECTS_DEFAULTING_WORLD_OBJECT_VISITOR_H_
#define INTRINSIC_WORLD_OBJECTS_DEFAULTING_WORLD_OBJECT_VISITOR_H_

#include "absl/status/status.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/physical_object.h"
#include "intrinsic/world/objects/root_object.h"
#include "intrinsic/world/objects/world_object_internal.h"

namespace intrinsic {
namespace object_world {

// Base class for implementations of WorldObjectVisitor that share a common
// default behavior for more than one type of WorldObject.
//
// CAUTION: Only use this class for use cases where it is acceptable that your
// implementation will continue to compile should new WorldObject types be
// added.
class DefaultingWorldObjectVisitor : public WorldObjectVisitor {
 public:
  // Visit action to be executed as a default for all WorldObject types for
  // which the corresponding visit method is not overwritten explicitly.
  virtual absl::Status DefaultVisit(WorldObject& object) = 0;

  absl::Status Visit(RootObject& root_object) override {
    return DefaultVisit(root_object);
  }

  absl::Status Visit(PhysicalObject& physical_object) override {
    return DefaultVisit(physical_object);
  }

  absl::Status Visit(KinematicObject& kinematic_object) override {
    return DefaultVisit(kinematic_object);
  }
};

// Const variant of DefaultingWorldObjectVisitor.
class DefaultingWorldObjectConstVisitor : public WorldObjectConstVisitor {
 public:
  // Visit action to be executed as a default for all WorldObject types for
  // which the corresponding visit method is not overwritten explicitly.
  virtual absl::Status DefaultVisit(const WorldObject& object) = 0;

  absl::Status Visit(const RootObject& root_object) override {
    return DefaultVisit(root_object);
  }

  absl::Status Visit(const PhysicalObject& physical_object) override {
    return DefaultVisit(physical_object);
  }

  absl::Status Visit(const KinematicObject& kinematic_object) override {
    return DefaultVisit(kinematic_object);
  }
};

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_DEFAULTING_WORLD_OBJECT_VISITOR_H_
