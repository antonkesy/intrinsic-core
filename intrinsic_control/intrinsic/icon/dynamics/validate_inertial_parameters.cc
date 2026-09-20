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

#include "intrinsic/icon/dynamics/validate_inertial_parameters.h"

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/validate_link_parameters.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::Status ValidateInertialParameters(
    const intrinsic::kinematics::ModelInterface* model) {
  if (model == nullptr) {
    return absl::InvalidArgumentError(
        "The model cannot be a nullptr. Validation of inertial parameters "
        "failed.");
  }

  // Iterate through all joints in the kinematic chain.
  for (const kinematics::ElementId joint_element_id : model->GetAllJointIds()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const kinematics::Joint* current_joint_ptr,
                                  model->GetJoint(joint_element_id));

    // Check if there is a child that is a link (a joint might _also_ have
    // children that are frames)
    const kinematics::Link* child_link_ptr = nullptr;
    for (const kinematics::Element* child :
         current_joint_ptr->GetChildElements()) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(kinematics::ElementId child_id,
                                    model->GetElementId(child));
      icon::RealtimeStatusOr<const kinematics::Link*> maybe_child_link =
          model->GetLink(child_id);
      if (!maybe_child_link.ok()) {
        continue;
      }
      // Check if we've already identified one of the joint's child elements as
      // a link in a previous iteration
      if (child_link_ptr != nullptr) {
        return absl::FailedPreconditionError(
            absl::StrCat("Joint '", current_joint_ptr->GetName(),
                         "' has more than one child link, this case is "
                         "currently not handled"));
      }
      child_link_ptr = maybe_child_link.value();
    }
    if (child_link_ptr == nullptr) {
      return absl::FailedPreconditionError(
          absl::StrCat("Joint '", current_joint_ptr->GetName(),
                       "' does not have any child that is a link"));
    }
    const kinematics::Link::Parameters& link_parameters =
        child_link_ptr->GetParameters();

    INTR_RETURN_IF_ERROR(
        intrinsic::kinematics::ValidateMass(link_parameters.mass))
        << "The mass of link " << child_link_ptr->GetName() << " is not valid.";

    INTR_RETURN_IF_ERROR(
        intrinsic::kinematics::ValidateInertia(link_parameters.inertia))
        << "Inertia tensor of link " << child_link_ptr->GetName()
        << " is not valid.";
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::icon
