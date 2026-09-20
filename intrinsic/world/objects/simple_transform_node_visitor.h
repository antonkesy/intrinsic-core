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

#ifndef INTRINSIC_WORLD_OBJECTS_SIMPLE_TRANSFORM_NODE_VISITOR_H_
#define INTRINSIC_WORLD_OBJECTS_SIMPLE_TRANSFORM_NODE_VISITOR_H_

#include <functional>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/transform_node_internal.h"
#include "intrinsic/world/objects/world_object_internal.h"

namespace intrinsic {
namespace object_world {

// Implementation of TransformNodeVisitor that is based on std::functions and
// thus can, e.g., be used with inline lambdas.
//
// Example:
//   TransformNode* node = ...;
//   SimpleTransformNodeVisitor visitor(
//       [](Frame& frame) {
//         LOG(INFO) << "Look, it's a frame!"
//         return absl::OkStatus();
//       },
//       [](WorldObject& object) {
//         LOG(INFO) << "Look, it's an object!"
//         return absl::OkStatus();
//       });
//   INTR_RETURN_IF_ERROR(node->Accept(visitor));
class SimpleTransformNodeVisitor : public TransformNodeVisitor {
 public:
  SimpleTransformNodeVisitor(
      std::function<absl::Status(Frame& frame)> frame_function,
      std::function<absl::Status(WorldObject& frame)> object_function)
      : frame_function_(std::move(frame_function)),
        object_function_(std::move(object_function)) {}

  absl::Status Visit(Frame& frame) override { return frame_function_(frame); }
  absl::Status Visit(WorldObject& object) override {
    return object_function_(object);
  }

 private:
  std::function<absl::Status(Frame& frame)> frame_function_;
  std::function<absl::Status(WorldObject& object)> object_function_;
};

// Const variant of SimpleTransformNodeVisitor above.
class SimpleTransformNodeConstVisitor : public TransformNodeConstVisitor {
 public:
  SimpleTransformNodeConstVisitor(
      std::function<absl::Status(const Frame& frame)> frame_function,
      std::function<absl::Status(const WorldObject& frame)> object_function)
      : frame_function_(std::move(frame_function)),
        object_function_(std::move(object_function)) {}

  absl::Status Visit(const Frame& frame) override {
    return frame_function_(frame);
  }
  absl::Status Visit(const WorldObject& object) override {
    return object_function_(object);
  }

 private:
  std::function<absl::Status(const Frame& frame)> frame_function_;
  std::function<absl::Status(const WorldObject& object)> object_function_;
};

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_SIMPLE_TRANSFORM_NODE_VISITOR_H_
