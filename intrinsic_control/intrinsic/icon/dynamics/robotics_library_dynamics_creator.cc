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

#include "intrinsic/icon/dynamics/robotics_library_dynamics_creator.h"

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/dynamics/robotics_library_dynamics.h"
#include "intrinsic/icon/dynamics/validate_inertial_parameters.h"
#include "intrinsic/icon/utils/constants.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"
#include "rl/mdl/Body.h"
#include "rl/mdl/Fixed.h"
#include "rl/mdl/Model.h"
#include "rl/mdl/Prismatic.h"
#include "rl/mdl/Revolute.h"
#include "rl/mdl/World.h"

namespace intrinsic::icon {

absl::StatusOr<std::unique_ptr<RigidBodyInterface>>
CreateRoboticsLibraryDynamics(
    std::unique_ptr<intrinsic::kinematics::Skeleton> model) {
  INTR_RETURN_IF_ERROR(ValidateInertialParameters(model.get()));

  auto dynamic = std::make_unique<rl::mdl::Dynamic>();
  dynamic->setName(std::string(model->GetName()));

  absl::flat_hash_map<kinematics::ElementId, rl::mdl::Frame*>
      element_id_to_frame_map;

  // Transfer the link dynamics properties to the rl Model.
  for (const auto& link_id : model->GetAllLinkIds()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* link, model->GetLink(link_id));
    rl::mdl::Body* body = new rl::mdl::Body();
    dynamic->add(body);
    body->setName(std::string(link->GetName()));

    // Update the map from the current link ID to the body.
    element_id_to_frame_map[link_id] = body;

    // The model records the link transformations in the frame stated by the
    // original URDF.
    // https://github.com/intrinsic-ai/intrinsic-core/blob/main/google3/intrinsic/kinematics/config_node_assembly.cc#L133
    // The RoboticsLibrary URDF appears to desire links reported directly from
    // the URDF format as well.
    // https://github.com/intrinsic-ai/intrinsic-core/blob/main/google3/third_party/roboticslibrary/src/rl/mdl/UrdfFactory.cpp#L132
    kinematics::Link::Parameters link_parameters = link->GetParameters();
    Pose3d body_frame_t_CoM = link_parameters.center_of_gravity;
    body->setCenterOfMass(body_frame_t_CoM.translation().x(),
                          body_frame_t_CoM.translation().y(),
                          body_frame_t_CoM.translation().z());
    body->setMass(link_parameters.mass);
    // Rotate the inertia tensor before setting body inerita.
    // The 'RoboticsLibrary' assumes that the provided inertia tensor is
    // expressed at the body's center of mass, but aligned with the body
    // frame.
    eigenmath::Matrix3d body_frame_R_CoM = body_frame_t_CoM.rotationMatrix();
    eigenmath::Matrix3d inertia_body_frame_aligned =
        body_frame_R_CoM * link_parameters.inertia *
        body_frame_R_CoM.transpose();
    body->setInertia(
        /* Ixx */ inertia_body_frame_aligned(0, 0),
        /* Iyy */ inertia_body_frame_aligned(1, 1),
        /* Izz */ inertia_body_frame_aligned(2, 2),
        /* Iyz */ inertia_body_frame_aligned(1, 2),
        /* Ixz */ inertia_body_frame_aligned(0, 2),
        /* Ixy */ inertia_body_frame_aligned(0, 1));
  }

  for (const auto& joint_id : model->GetAllJointIds()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint, model->GetJoint(joint_id));
    // Check if parent is a link
    std::optional<const kinematics::Element*> parent_element =
        joint->GetParentElement();
    if (parent_element.value_or(nullptr) == nullptr) {
      return absl::FailedPreconditionError(
          absl::StrCat("No parent link found for joint ", joint->GetName(),
                       ". If you're certain that this joint *does* have a "
                       "parent, check if the parent is a frame. This function "
                       "does not support joints whose parent is a frame"));
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(kinematics::ElementId parent_link_element_id,
                                  model->GetElementId(parent_element.value()));
    // We don't care about the link pointer that this returns. We only care
    // about whether or not the parent element actually *is* a link.
    if (!model->GetLink(parent_link_element_id).status().ok()) {
      return absl::FailedPreconditionError(
          absl::StrCat("Parent element (ID ", parent_link_element_id.value(),
                       ") for joint ", joint->GetName(), " is not a link!"));
    }

    // Check if there is a child that is a link (a joint might _also_ have
    // children that are frames)
    const kinematics::Link* child_link = nullptr;
    for (const kinematics::Element* child : joint->GetChildElements()) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(kinematics::ElementId child_id,
                                    model->GetElementId(child));
      icon::RealtimeStatusOr<const kinematics::Link*> maybe_child_link =
          model->GetLink(child_id);
      if (!maybe_child_link.ok()) {
        continue;
      }
      // Check if we've already identified one of the joint's child elements as
      // a link in a previous iteration
      if (child_link != nullptr) {
        return absl::FailedPreconditionError(
            absl::StrCat("Joint '", joint->GetName(),
                         "' has more than one child link, which is currently "
                         "not supported"));
      }
      child_link = maybe_child_link.value();
    }

    auto itr = element_id_to_frame_map.find(parent_link_element_id);
    if (itr == element_id_to_frame_map.end()) {
      return absl::FailedPreconditionError(
          absl::StrCat("No parent link found for joint ", joint->GetName()));
    }
    rl::mdl::Frame* parent = itr->second;

    INTRINSIC_RT_ASSIGN_OR_RETURN(auto child_link_element_id,
                                  model->GetElementId(child_link));
    itr = element_id_to_frame_map.find(child_link_element_id);
    if (itr == element_id_to_frame_map.end()) {
      return absl::FailedPreconditionError(
          absl::StrCat("No child link found for joint ", joint->GetName()));
    }
    rl::mdl::Frame* child = itr->second;

    rl::mdl::Frame* frame = new rl::mdl::Frame();
    dynamic->add(frame);

    rl::mdl::Fixed* fixed = new rl::mdl::Fixed();

    // The model records the joint transformations from parent to child as
    // stated by the original URDF.
    // https://github.com/intrinsic-ai/intrinsic-core/blob/main/google3/intrinsic/kinematics/config_node_utils.cc#L101
    // The Robotics library implementation appears to desire frames reported as
    // parent to child driectly from the URDF format as well.
    // https://github.com/intrinsic-ai/intrinsic-core/blob/main/google3/third_party/roboticslibrary/src/rl/mdl/UrdfFactory.cpp#L191
    Pose3d parent_t_joint = joint->GetParentTThis();
    fixed->x.linear() = parent_t_joint.rotationMatrix();
    fixed->x.translation().x() = parent_t_joint.translation().x();
    fixed->x.translation().y() = parent_t_joint.translation().y();
    fixed->x.translation().z() = parent_t_joint.translation().z();

    switch (joint->GetParameters().type) {
      case intrinsic::kinematics::Joint::REVOLUTE: {
        rl::mdl::Revolute* revolute = new rl::mdl::Revolute();

        dynamic->add(fixed, parent, frame);
        dynamic->add(revolute, frame, child);

        revolute->speed(0) = joint->GetParameters().system_limits.velocity;
        revolute->offset(0) = 0;

        // Taken from
        // https://github.com/intrinsic-ai/intrinsic-core/blob/main/google3/third_party/roboticslibrary/src/rl/mdl/UrdfFactory.cpp#L310
        revolute->S(0, 0) = joint->GetParameters().axis(0);
        revolute->S(1, 0) = joint->GetParameters().axis(1);
        revolute->S(2, 0) = joint->GetParameters().axis(2);

        revolute->setName(std::string(joint->GetName()));

        const auto& position_limits =
            joint->GetParameters().system_limits.position;
        if (isfinite(position_limits.lower) &&
            isfinite(position_limits.upper)) {
          revolute->max(0) = position_limits.upper;
          revolute->min(0) = position_limits.lower;
          revolute->wraparound(0) = false;
        } else {
          if (isfinite(position_limits.lower) ||
              isfinite(position_limits.upper)) {
            return icon::InternalError("Both limit should be unlimited");
          }
          revolute->max(0) = M_PI;
          revolute->min(0) = -M_PI;
          revolute->wraparound(0) = true;
        }
        break;
      }
      case intrinsic::kinematics::Joint::PRISMATIC: {
        rl::mdl::Prismatic* prismatic = new rl::mdl::Prismatic();
        dynamic->add(fixed, parent, child);
        dynamic->add(prismatic, frame, child);

        prismatic->max(0) = joint->GetParameters().system_limits.position.upper;
        prismatic->min(0) = joint->GetParameters().system_limits.position.lower;
        prismatic->speed(0) = joint->GetParameters().system_limits.velocity;
        prismatic->wraparound(0) = false;
        prismatic->offset(0) = 0;

        // Taken from
        // https://github.com/intrinsic-ai/intrinsic-core/blob/main/google3/third_party/roboticslibrary/src/rl/mdl/UrdfFactory.cpp#L263
        prismatic->S(3, 0) = joint->GetParameters().axis(0);
        prismatic->S(4, 0) = joint->GetParameters().axis(1);
        prismatic->S(5, 0) = joint->GetParameters().axis(2);

        prismatic->setName(std::string(joint->GetName()));
        break;
      }
      case intrinsic::kinematics::Joint::FIXED: {
        dynamic->add(fixed, parent, child);
        fixed->setName(std::string(joint->GetName()));
        break;
      }
    }
  }

  // Add the World and Root links.
  rl::mdl::World* world = new rl::mdl::World();
  dynamic->add(world);
  world->setGravity(0, 0, intrinsic::kDefaultGravity);

  if (model->GetBaseId() == kinematics::kInvalidElementId) {
    return absl::FailedPreconditionError("Root link shouldn't be invalid.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* base_link,
                                model->GetLink(model->GetBaseId()));

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto base_link_element_id,
                                model->GetElementId(base_link));
  auto itr = element_id_to_frame_map.find(base_link_element_id);
  if (itr == element_id_to_frame_map.end()) {
    return absl::FailedPreconditionError(
        absl::StrCat("No root link found for skeleton ", model->GetName()));
  }
  rl::mdl::Frame* root = itr->second;

  rl::mdl::Fixed* fixed = new rl::mdl::Fixed();
  dynamic->add(fixed, world, root);

  dynamic->update();
  return std::make_unique<RoboticsLibraryDynamics>(std::move(dynamic),
                                                   std::move(model));
}

}  // namespace intrinsic::icon
