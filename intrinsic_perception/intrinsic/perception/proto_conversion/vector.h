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

#ifndef INTRINSIC_PERCEPTION_PROTO_CONVERSION_VECTOR_H_
#define INTRINSIC_PERCEPTION_PROTO_CONVERSION_VECTOR_H_

#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/proto/vector.pb.h"

namespace intrinsic_proto::perception {

intrinsic_proto::perception::Vector3f ToProto(
    const intrinsic::perception::Vector3f& k);

intrinsic::perception::Vector3f FromProto(
    const intrinsic_proto::perception::Vector3f& k);

intrinsic_proto::perception::Vector2f ToProto(
    const intrinsic::perception::Vector2f& k);

intrinsic::perception::Vector2f FromProto(
    const intrinsic_proto::perception::Vector2f& k);

}  // namespace intrinsic_proto::perception

// Convenience ToProto aliases to allow argument-dependent lookup. Directly use
// the versioned namespace alternatives above instead when dealing with
// different versions at the same time.
namespace intrinsic::perception {

using ::intrinsic_proto::perception::FromProto;
using ::intrinsic_proto::perception::ToProto;

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_PROTO_CONVERSION_VECTOR_H_
