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

#ifndef INTRINSIC_KINEMATICS_PROTO_KINEMATICS_CONVERSION_H_
#define INTRINSIC_KINEMATICS_PROTO_KINEMATICS_CONVERSION_H_

#include <memory>
#include <string>

#include "absl/container/btree_set.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/proto/kinematics.pb.h"
#include "intrinsic/kinematics/proto/skeleton.pb.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// Serialize a skeleton to proto.
absl::StatusOr<intrinsic_proto::Skeleton> ToProto(const Skeleton& skeleton)
    INTRINSIC_NON_REALTIME_ONLY;

// Deserialize a skeleton from proto.
absl::StatusOr<std::unique_ptr<Skeleton>> FromProto(
    const intrinsic_proto::Skeleton& skeleton_proto)
    INTRINSIC_NON_REALTIME_ONLY;

// Serialize a `linear_dependency` to proto.
intrinsic_proto::Joint::LinearDependency ToProto(
    const Joint::LinearDependency& linear_dependency)
    INTRINSIC_NON_REALTIME_ONLY;

// Deserialize a `linear_dependency` from proto.
Joint::LinearDependency FromProto(
    const intrinsic_proto::Joint::LinearDependency& linear_dependency_proto)
    INTRINSIC_NON_REALTIME_ONLY;

namespace details {

// This is an intermediate representation of an element intended to be used
// during serialization and deserialization until all elements are used and
// pointer to in-memory elements can replace the ids of parent and children.
struct DeferredElement {
  // In order to avoid const_cast, we use an inbound and outbound element
  // pointer

  // Element owned by a skeleton
  const Element* outbound_element = nullptr;
  // Element created while reading the proto and which ownership should be
  // transferred to produced skeleton.
  Element* inbound_element = nullptr;

  ElementId id;
  ElementId parent_id;
  Pose3d parent_t_this;
  absl::btree_set<ElementId> children_ids;
};

struct DeferredJoint {
  DeferredElement deferred_element;
  std::string value_function;
  absl::flat_hash_map<std::string, ElementId> value_function_to_joint;
};

struct DeferredLink {
  // Mostly for consistency with joints and overload of ToProto.
  DeferredElement deferred_element;
};

intrinsic_proto::Element ToProto(const DeferredElement& element);
absl::StatusOr<DeferredElement> FromProto(
    const intrinsic_proto::Element& element_proto);

absl::StatusOr<intrinsic_proto::Link> ToProto(const DeferredLink& link_element);
absl::StatusOr<DeferredLink> FromProto(const intrinsic_proto::Link& link_proto);

absl::StatusOr<intrinsic_proto::Joint> ToProto(
    const DeferredJoint& joint_element);
absl::StatusOr<DeferredJoint> FromProto(
    const intrinsic_proto::Joint& joint_proto);

}  // namespace details
}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_PROTO_KINEMATICS_CONVERSION_H_
