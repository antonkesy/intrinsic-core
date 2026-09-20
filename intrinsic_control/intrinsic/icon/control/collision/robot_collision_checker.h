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

#ifndef INTRINSIC_ICON_CONTROL_COLLISION_ROBOT_COLLISION_CHECKER_H_
#define INTRINSIC_ICON_CONTROL_COLLISION_ROBOT_COLLISION_CHECKER_H_

#include <cstddef>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/collision/collision_object.h"
#include "intrinsic/icon/control/collision/collision_world.h"
#include "intrinsic/icon/control/collision/collision_world_loader.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/skeleton.h"

namespace intrinsic {
namespace collision {

//  Collision Checking Class for a generic n-DOF Robot
//  This class is a wrapper class on CollisionWorld and ForwardPositionKernel.
class RobotCollisionChecker {
 public:
  RobotCollisionChecker() = default;

  RobotCollisionChecker(const RobotCollisionChecker& robot_collision_checker) =
      delete;

  ~RobotCollisionChecker() = default;

  // Initializes collision checker from `skeleton`, `collision_world` and
  // `link_names_in_index_order`.
  //
  // Takes ownership of `skeleton`.
  // We need `link_names_in_index_order` because `collision_world` does not have
  // link names, but the API of RobotCollisionChecker allows looking up a link's
  // ID by name.
  //
  // Returns InternalError if `collision_world` is not valid.
  // Returns AlreadyExistsError if `link_names_in_index_order` contains any
  //   duplicates.
  // Returns FailedPreconditionError if the following three are not equal:
  //   * number of links in `skeleton`
  //   * number of collision objects in `collision_world`
  //   * number of items in `link_names_in_index_order`
  // Returns NotFoundError if any of the link names in
  //   `link_names_in_index_order` do not name an element in `skeleton`.
  // Returns FailedPreconditionError if any of the link names in
  //   `link_names_in_index_order` name a non-link element in `skeleton`.
  // Returns FailedPreconditionError if `skeleton` has a degree of freedom with
  //   no child link.
  icon::RealtimeStatus Init(
      std::unique_ptr<intrinsic::kinematics::Skeleton> skeleton,
      collision::CollisionWorld collision_world,
      std::vector<std::string> link_names_in_index_order)
      INTRINSIC_NON_REALTIME_ONLY;

  // Writes the indices for each link in `link_names` into `link_indices` (in
  // the same order).
  //
  // Link indices are the same as those used by Skeleton. I.e., if you have a
  // Skeleton built from the same model that this RobotCollisionChecker was
  // initialized with, you can use DoF indices from that Skeleton with this
  // checker.
  //
  // Returns an error without doing any work if the collision checker is in an
  // error state.
  // Returns NotFoundError and saves the error state for future calls if any of
  // the `link_names` does not name a link in the collision model.
  icon::RealtimeStatusOr<std::vector<std::size_t>> GetLinkIndices(
      const std::vector<std::string>& link_names) INTRINSIC_NON_REALTIME_ONLY;

  // Dumps the underlying CollisionWorld to `out`.
  std::ostream& operator<<(std::ostream& out);

  // Updates the internal kinematic model to the given `joint_angles` and
  // computes collision results at those angles. The order of joint angles is
  // the same as it would be for a Skeleton built from the same model that this
  // RobotCollisionChecker was initialized with.
  //
  // Inspect the collision results using the other methods.
  //
  // Returns an error without doing any work if the collision checker is in an
  // error state.
  // Returns InvalidArgumentError and saves the error state for future calls if
  // the size of `joint_angles` does not match the number of joints in the
  // collision model.
  icon::RealtimeStatus UpdateCollisionState(
      const eigenmath::VectorNd& joint_angles) INTRINSIC_CHECK_REALTIME_SAFE;

  std::vector<CollisionResult> GetCollisionResults() const
      INTRINSIC_NON_REALTIME_ONLY {
    return collision_result_;
  }

  // Check whether RobotCollisionChecker is valid or not
  bool IsValid() const INTRINSIC_CHECK_REALTIME_SAFE { return status_.ok(); }

  // Checks for collisions among any links.
  bool IsThereCollision() const INTRINSIC_CHECK_REALTIME_SAFE {
    return has_collision_;
  }

  // Checks for collision among the links with the given `link_indices`.
  //
  // You can find link indices for a link with a given name using
  // `GetLinkIndices()` (above, non-realtime only).
  //
  // Returns an error without doing any work if the collision checker is in an
  // error state.
  // Returns OutOfRangeError and saves the error state for future calls if
  // any member of `link_indices` is invalid (i.e. larger than the highest valid
  // link index).
  // Otherwise, returns true if one or more of the listed links are
  // in collision.
  icon::RealtimeStatusOr<bool> IsThereCollision(
      const std::vector<std::size_t>& link_indices)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns the minimum collision distance among all links.
  double GetMinCollisionDistance() const INTRINSIC_CHECK_REALTIME_SAFE {
    return collision_world_.GetMinCollisionDistance();
  }

  // Returns the number of degrees of freedom of the robot.
  size_t GetNumDofs() const INTRINSIC_CHECK_REALTIME_SAFE { return num_dofs_; }

  // Returns the number of links defined for the robot
  size_t GetNumLinks() const INTRINSIC_CHECK_REALTIME_SAFE {
    return num_links_;
  }

 private:
  // Reset function used to re-initialize
  void Reset() INTRINSIC_NON_REALTIME_ONLY {
    num_dofs_ = 0;
    num_links_ = 0;
    link_names_.clear();
    link_name_to_collision_world_link_index_.clear();
    fk_link_indices_.clear();
    collision_result_.clear();
    status_ = icon::FailedPreconditionError("Uninitialized");
  }

  // Computes forward kinematics and updates the link reference frames'
  // positions and rotations in world frame at the given `joint_angles`.
  //
  // `joint_angles` must be ordered the same way as the DoFs of `skeleton_`.
  void UpdateLinkTransformations(const eigenmath::VectorNd& joint_angles)
      INTRINSIC_CHECK_REALTIME_SAFE;
  size_t num_dofs_ = 0;
  size_t num_links_ = 0;
  // Vector of link names
  std::vector<std::string> link_names_;
  // Maps from link name to link index (in `collision_world_` and
  // `collision_world_`).
  absl::flat_hash_map<std::string, size_t>
      link_name_to_collision_world_link_index_;
  // For each `i`, the `i`th element of this holds the ElementId that identifies
  // the `i`th link in `collision_world_` in `skeleton_`.
  std::vector<intrinsic::kinematics::ElementId> fk_link_indices_;

  // Current status
  icon::RealtimeStatus status_ = icon::FailedPreconditionError("Uninitialized");
  bool has_collision_ = false;
  std::vector<CollisionResult> collision_result_;
  CollisionWorld collision_world_;
  CollisionCheckInfo collision_check_info_;
  std::unique_ptr<intrinsic::kinematics::Skeleton> skeleton_;
};

}  // namespace collision
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_CONTROL_COLLISION_ROBOT_COLLISION_CHECKER_H_
