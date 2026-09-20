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

#include "intrinsic/perception/proto_conversion/vector.h"

#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/proto/vector.pb.h"

namespace intrinsic_proto::perception {

intrinsic_proto::perception::Vector3f ToProto(
    const intrinsic::perception::Vector3f& k) {
  intrinsic_proto::perception::Vector3f proto;
  proto.set_x(k.x());
  proto.set_y(k.y());
  proto.set_z(k.z());
  return proto;
}

intrinsic::perception::Vector3f FromProto(
    const intrinsic_proto::perception::Vector3f& k) {
  return {k.x(), k.y(), k.z()};
}

intrinsic_proto::perception::Vector2f ToProto(
    const intrinsic::perception::Vector2f& k) {
  intrinsic_proto::perception::Vector2f proto;
  proto.set_x(k.x());
  proto.set_y(k.y());
  return proto;
}

intrinsic::perception::Vector2f FromProto(
    const intrinsic_proto::perception::Vector2f& k) {
  return {k.x(), k.y()};
}

}  // namespace intrinsic_proto::perception
