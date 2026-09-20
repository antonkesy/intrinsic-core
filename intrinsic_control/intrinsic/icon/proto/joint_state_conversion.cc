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

#include "intrinsic/icon/proto/joint_state_conversion.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/repeated_field.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace {

void ToProto(const eigenmath::VectorNd& v,
             ::google::protobuf::RepeatedField<double>* proto) {
  proto->Assign(v.begin(), v.end());
}

}  // namespace

intrinsic_proto::icon::JointState ToProto(const JointStateP& joint_state) {
  intrinsic_proto::icon::JointState proto;
  ToProto(joint_state.position, proto.mutable_position());
  return proto;
}

intrinsic_proto::icon::JointStatePVA ToProto(const JointStatePVA& joint_state) {
  intrinsic_proto::icon::JointStatePVA proto;
  ToProto(joint_state.position, proto.mutable_position());
  ToProto(joint_state.velocity, proto.mutable_velocity());
  ToProto(joint_state.acceleration, proto.mutable_acceleration());
  return proto;
}

absl::StatusOr<JointStatePVA> FromProto(
    const intrinsic_proto::icon::JointStatePVA& proto) {
  if (proto.position().size() != proto.velocity().size()) {
    return absl::FailedPreconditionError(
        absl::StrCat("Position dimension, which is ", proto.position().size(),
                     " does not match velocity dimension, which is ",
                     proto.velocity().size()));
  }

  if (proto.position().size() != proto.acceleration().size()) {
    return absl::FailedPreconditionError(
        absl::StrCat("Position dimension, which is ", proto.position().size(),
                     " does not match acceleration dimension, which is ",
                     proto.acceleration().size()));
  }

  JointStatePVA joint_state_out;
  INTR_RETURN_IF_ERROR(joint_state_out.SetSize(proto.position().size()));
  INTR_ASSIGN_OR_RETURN(joint_state_out.position,
                        icon::RepeatedDoubleToVectorNd(proto.position()));
  INTR_ASSIGN_OR_RETURN(joint_state_out.velocity,
                        icon::RepeatedDoubleToVectorNd(proto.velocity()));
  INTR_ASSIGN_OR_RETURN(joint_state_out.acceleration,
                        icon::RepeatedDoubleToVectorNd(proto.acceleration()));

  return joint_state_out;
}

intrinsic_proto::icon::JointStatePV ToProto(const JointStatePV& joint_state) {
  intrinsic_proto::icon::JointStatePV proto;
  ToProto(joint_state.position, proto.mutable_position());
  ToProto(joint_state.velocity, proto.mutable_velocity());
  return proto;
}

absl::StatusOr<JointStatePV> FromProto(
    const intrinsic_proto::icon::JointStatePV& proto) {
  if (proto.position().size() != proto.velocity().size()) {
    return absl::FailedPreconditionError(
        absl::StrCat("Position dimension, which is ", proto.position().size(),
                     " does not match velocity dimension, which is ",
                     proto.velocity().size()));
  }

  JointStatePV joint_state_out;
  INTR_RETURN_IF_ERROR(joint_state_out.SetSize(proto.position().size()));
  INTR_ASSIGN_OR_RETURN(joint_state_out.position,
                        icon::RepeatedDoubleToVectorNd(proto.position()));
  INTR_ASSIGN_OR_RETURN(joint_state_out.velocity,
                        icon::RepeatedDoubleToVectorNd(proto.velocity()));

  return joint_state_out;
}

}  // namespace intrinsic
