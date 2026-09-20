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

#include "intrinsic/icon/control/collision/robot_collision_checker.h"

#include <cstddef>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/collision/collision_world.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace collision {

icon::RealtimeStatus RobotCollisionChecker::Init(
    std::unique_ptr<intrinsic::kinematics::Skeleton> skeleton,
    collision::CollisionWorld collision_world,
    std::vector<std::string> link_names_in_index_order)
    INTRINSIC_NON_REALTIME_ONLY {
  if (skeleton == nullptr) {
    return icon::InvalidArgumentError("Skeleton must not be nullptr");
  }
  Reset();
  collision_world_ = std::move(collision_world);
  link_names_ = link_names_in_index_order;
  skeleton_ = std::move(skeleton);

  if (!collision_world_.IsValid()) {
    status_ = icon::InternalError("Could not create collision world.");
    return status_;
  }

  for (size_t link_ind = 0; link_ind < link_names_.size(); link_ind++) {
    // add to the hash map if unique else report error
    if (bool inserted = link_name_to_collision_world_link_index_
                            .try_emplace(link_names_.at(link_ind), link_ind)
                            .second;
        !inserted) {
      status_ = icon::AlreadyExistsError(icon::RealtimeStatus::StrCat(
          "Link name ", link_names_.at(link_ind), " used twice."));
      return status_;
    }
  }
  if (link_names_.size() != collision_world_.GetNumberOfCollisionObjects()) {
    status_ = icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
        "Collision info has ", link_names_.size(),
        " links, collision world has ",
        collision_world_.GetNumberOfCollisionObjects(),
        " collision objects. Those numbers need to be equal!"));
    return status_;
  }
  if ((skeleton_->GetAllLinkIds().size() > link_names_.size()) ||
      (skeleton_->GetAllLinkIds().size() >
       collision_world_.GetNumberOfCollisionObjects())) {
    status_ = icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
        "Link number mismatch. Skeleton: ", skeleton_->GetAllLinkIds().size(),
        " links, collision info: ", link_names_.size(),
        " links, collision world: ",
        collision_world_.GetNumberOfCollisionObjects(), " collision objects."));
    return status_;
  }

  // assign number of dofs
  num_dofs_ = skeleton_->GetNumberDegreesOfFreedom();
  // assign number of links
  num_links_ = skeleton_->GetAllLinkIds().size();
  // get the indices for the links corresponding to their names
  // also get the indices of their parent joints
  fk_link_indices_.resize(num_links_);
  for (size_t link_ind = 0; link_ind < num_links_; link_ind++) {
    const auto fk_link_id_or =
        skeleton_->FindElementIdByName(link_names_[link_ind]);
    if (!fk_link_id_or.ok()) {
      status_ = icon::NotFoundError(icon::RealtimeStatus::StrCat(
          "Failed to find link named ", link_names_[link_ind], ": ",
          fk_link_id_or.status().message()));
      return status_;
    }
    const auto fk_link_or = skeleton_->GetLink(fk_link_id_or.value());
    if (!fk_link_or.ok()) {
      status_ = icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
          "Element named ", link_names_[link_ind],
          " is not a link: ", fk_link_or.status().message()));
      return status_;
    }
    const kinematics::Link* fk_link = fk_link_or.value();

    auto link_id_or = skeleton_->GetElementId(fk_link);
    if (!link_id_or.ok()) {
      status_ = icon::NotFoundError(icon::RealtimeStatus::StrCat(
          "Failed to get link ID of ", fk_link->GetName(), ": ",
          link_id_or.status().message()));
      return status_;
    }
    fk_link_indices_[link_ind] = link_id_or.value();
  }

  // no error
  status_ = icon::OkStatus();

  // Clear and Resize collision data
  collision_result_.clear();
  collision_result_.resize(GetNumLinks());

  return status_;
}

icon::RealtimeStatusOr<std::vector<std::size_t>>
RobotCollisionChecker::GetLinkIndices(
    const std::vector<std::string>& link_names) {
  if (!IsValid()) {
    return status_;
  }
  std::vector<std::size_t> link_indices;
  for (auto& link : link_names) {
    // check if link name is available
    if (auto link_and_index =
            link_name_to_collision_world_link_index_.find(link);
        link_and_index == link_name_to_collision_world_link_index_.end()) {
      // If not, return an error
      status_ = icon::NotFoundError(
          icon::RealtimeStatus::StrCat("Cannot find link name'", link, "'"));
      return status_;
    } else {
      // If yes, add link index
      link_indices.push_back(link_and_index->second);
    }
  }
  return link_indices;
}

icon::RealtimeStatus RobotCollisionChecker::UpdateCollisionState(
    const eigenmath::VectorNd& joint_angles) {
  // check if robot collision checker has been configured correctly
  if (!IsValid()) {
    return status_;
  }
  if (joint_angles.size() != GetNumDofs()) {
    status_ = icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Collision checker has ", GetNumDofs(),
        " degrees of freedom, but got joint angles with size ",
        joint_angles.size()));
    return status_;
  }
  // update transformations of collision objects in links
  UpdateLinkTransformations(joint_angles);
  // No errors, actuallly check the collision
  status_ = icon::OkStatus();
  has_collision_ = collision_world_.CheckCollision(&collision_result_);
  return status_;
}

void RobotCollisionChecker::UpdateLinkTransformations(
    const eigenmath::VectorNd& joint_angles) {
  // check if robot collision checker has been configured correctly
  if (!IsValid()) {
    INTRINSIC_RT_LOG(ERROR) << "ERROR: RobotCollisionChecker is INVALID!";
    return;
  }
  // compute FK
  intrinsic::kinematics::State state(skeleton_.get());
  CHECK_EQ(state.SetDofPositions(joint_angles), intrinsic::icon::OkStatus());

  // update transformations of their associated collision objects
  // collision planes are ignored
  for (size_t link_ind = 0; link_ind < GetNumLinks(); link_ind++) {
    INTRINSIC_RT_ASSIGN_OR_DIE(Pose3d fk_root_T_link,
                               state.GetTransform(fk_link_indices_[link_ind]));
    collision_world_.SetWorldObjectTransformation(
        link_ind, fk_root_T_link.so3().matrix(), fk_root_T_link.translation());
  }
}

icon::RealtimeStatusOr<bool> RobotCollisionChecker::IsThereCollision(
    const std::vector<std::size_t>& link_indices) {
  bool collision_occurs = false;
  for (auto& link_ind : link_indices) {
    // check if link index is valid
    if (link_ind >= num_links_) {
      status_ = icon::OutOfRangeError(icon::RealtimeStatus::StrCat(
          "Link index ", link_ind, " is outside of the valid range (0-",
          num_links_, ")"));
      return status_;
    }
    // check the collision status of the link
    if (collision_result_[link_ind].collision_occurs) {
      collision_occurs = true;
    }
  }
  return collision_occurs;
}

std::ostream& RobotCollisionChecker::operator<<(std::ostream& out) {
  collision_world_ << out;
  return out;
}

}  // namespace collision

}  // namespace intrinsic
