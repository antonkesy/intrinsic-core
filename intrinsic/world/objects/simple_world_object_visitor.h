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

#ifndef INTRINSIC_WORLD_OBJECTS_SIMPLE_WORLD_OBJECT_VISITOR_H_
#define INTRINSIC_WORLD_OBJECTS_SIMPLE_WORLD_OBJECT_VISITOR_H_

#include <functional>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/physical_object.h"
#include "intrinsic/world/objects/root_object.h"
#include "intrinsic/world/objects/world_object_internal.h"

namespace intrinsic {
namespace object_world {

// Implementation of TransformNodeVisitor that is based on std::functions and
// thus can, e.g., be used with inline lambdas.
//
// Example:
//   WorldObject* object = ...;
//   SimpleWorldObjectVisitor visitor(
//       [](RootObject& root) {
//         LOG(INFO) << "Look, it's root!"
//         return absl::OkStatus();
//       },
//       [](PhysicalObject& object) {
//         LOG(INFO) << "Look, it's an object!"
//         return absl::OkStatus();
//       },
//       [](KinematicObject& object) {
//         LOG(INFO) << "Look, it's a robot!"
//         return absl::OkStatus();
//       });
//   INTR_RETURN_IF_ERROR(object->Accept(visitor));
class SimpleWorldObjectVisitor : public WorldObjectVisitor {
 public:
  SimpleWorldObjectVisitor(
      std::function<absl::Status(RootObject&)> root_object_function,
      std::function<absl::Status(PhysicalObject&)> physical_object_function,
      std::function<absl::Status(KinematicObject&)> kinematic_object_function)
      : root_object_function_(std::move(root_object_function)),
        physical_object_function_(std::move(physical_object_function)),
        kinematic_object_function_(std::move(kinematic_object_function)) {}

  absl::Status Visit(RootObject& root_object) override {
    return root_object_function_(root_object);
  }

  absl::Status Visit(PhysicalObject& physical_object) override {
    return physical_object_function_(physical_object);
  }

  absl::Status Visit(KinematicObject& kinematic_object) override {
    return kinematic_object_function_(kinematic_object);
  }

 private:
  std::function<absl::Status(RootObject&)> root_object_function_;
  std::function<absl::Status(PhysicalObject&)> physical_object_function_;
  std::function<absl::Status(KinematicObject&)> kinematic_object_function_;
};

// Const variant of SimpleWorldObjectVisitor above.
class SimpleWorldObjectConstVisitor : public WorldObjectConstVisitor {
 public:
  SimpleWorldObjectConstVisitor(
      std::function<absl::Status(const RootObject&)> root_object_function,
      std::function<absl::Status(const PhysicalObject&)>
          physical_object_function,
      std::function<absl::Status(const KinematicObject&)>
          kinematic_object_function)
      : root_object_function_(std::move(root_object_function)),
        physical_object_function_(std::move(physical_object_function)),
        kinematic_object_function_(std::move(kinematic_object_function)) {}

  absl::Status Visit(const RootObject& root_object) override {
    return root_object_function_(root_object);
  }

  absl::Status Visit(const PhysicalObject& physical_object) override {
    return physical_object_function_(physical_object);
  }

  absl::Status Visit(const KinematicObject& kinematic_object) override {
    return kinematic_object_function_(kinematic_object);
  }

 private:
  std::function<absl::Status(const RootObject&)> root_object_function_;
  std::function<absl::Status(const PhysicalObject&)> physical_object_function_;
  std::function<absl::Status(const KinematicObject&)>
      kinematic_object_function_;
};

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_SIMPLE_WORLD_OBJECT_VISITOR_H_
