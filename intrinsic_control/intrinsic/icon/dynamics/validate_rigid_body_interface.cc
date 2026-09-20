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

#include "intrinsic/icon/dynamics/validate_rigid_body_interface.h"

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

absl::Status ValidateRigidBodyInterface(
    icon::RigidBodyInterface* rigid_body_interface,
    const eigenmath::VectorNd& q, const eigenmath::VectorNd& dq) {
  if (!rigid_body_interface) {
    return absl::InvalidArgumentError(
        "The rigid body interface pointer cannot be a nullptr.");
  }
  if (rigid_body_interface->GetNumDof() != q.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The number of degrees of freedom of the rigid body interface does not "
        "match the expected value ",
        q.size(), ", but got ", rigid_body_interface->GetNumDof(), "."));
  }
  if (dq.size() != q.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("The number of degrees of freedom of joint position and "
                     "velocity vector do not match got ",
                     q.size(), ", vs ", dq.size(), "."));
  }

  icon::RealtimeStatusOr<eigenmath::VectorNd> generalized_gravity_status =
      rigid_body_interface->ComputeGeneralizedGravityVector(q);
  if (!generalized_gravity_status.ok()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Rigid body interface is invalid, failed at the "
                     "generalized gravity computation with error: ",
                     generalized_gravity_status.status().message(), "."));
  }
  icon::RealtimeStatusOr<eigenmath::VectorNd> coriolis_times_qp_status =
      rigid_body_interface->ComputeCoriolisVector(q, dq);
  if (!coriolis_times_qp_status.ok()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Rigid body interface is invalid, failed at the Coriolis "
                     "vector computation with error: ",
                     coriolis_times_qp_status.status().message(), "."));
  }
  icon::RealtimeStatusOr<eigenmath::MatrixNd>
      joint_space_inertia_matrix_status =
          rigid_body_interface->ComputeJointSpaceInertiaMatrix(q);
  if (!joint_space_inertia_matrix_status.ok()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Rigid body interface is invalid, failed at the joint space inertia "
        "matrix computation with error: ",
        joint_space_inertia_matrix_status.status().message(), "."));
  }

  return absl::OkStatus();
}

}  // namespace intrinsic::icon
