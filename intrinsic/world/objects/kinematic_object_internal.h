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

#ifndef INTRINSIC_WORLD_OBJECTS_KINEMATIC_OBJECT_INTERNAL_H_
#define INTRINSIC_WORLD_OBJECTS_KINEMATIC_OBJECT_INTERNAL_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.pb.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/frame_internal.h"
#include "intrinsic/world/objects/object_entity_filter.h"
#include "intrinsic/world/objects/object_world_data.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/physical_object.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"
#include "intrinsic/world/robot_payload/robot_payload.h"

namespace intrinsic {
namespace object_world {

// A physical object in the object-based view onto a world (see ObjectWorld)
// that has moveable joints.
//
// Corresponds to a robot collection in the underlying World (see, e.g.,
// RobotCollectionsEntityId in intrinsic/world/entity_id.h).
class KinematicObject : public PhysicalObject {
 public:
  // Creates an instance. You shouldn't need to call this directly, use
  // ObjectWorld::CreateView to create an entire ObjectWorld instance instead.
  KinematicObject(ObjectWorldResourceId id, WorldObjectName name,
                  RobotCollectionsEntityId robot_entity_id,
                  WorldHashSet<AttachmentEntityId> entity_ids,
                  ObjectWorldData& data);

  // Returns the joint positions in radians (for revolute joints) or meters (for
  // prismatic joints).
  absl::StatusOr<eigenmath::VectorXd> GetJointPositions() const;

  // Returns the joint system limits. Note that JointLimitsXd.max_torque is
  // not supported, it will always contain zeroes.
  absl::StatusOr<JointLimitsXd> GetJointSystemLimits() const;

  // Returns the joint application limits. Note that JointLimitsXd.max_torque is
  // not supported, it will always contain zeroes.
  absl::StatusOr<JointLimitsXd> GetJointApplicationLimits() const;

  // Returns the ids of the joint entities in the same joint order as
  // GetJointPositions() and GetJointLimits().
  absl::StatusOr<std::vector<JointEntityId>> GetJointEntityIds() const;

  // Returns the types of the individual joints.
  absl::StatusOr<
      std::vector<intrinsic_proto::world::KinematicsComponent::MotionType>>
  GetJointTypes() const;

  // Returns the named joint configurations stored on this kinematic object.
  absl::StatusOr<WorldHashMap<std::string, eigenmath::VectorXd>>
  GetNamedJointConfigurations() const;

  // Returns the named joint configuration stored on this kinematic object.
  absl::StatusOr<eigenmath::VectorXd> GetNamedJointConfiguration(
      absl::string_view name) const;

  // Returns the frames on this kinematic object which mark flanges according to
  // the ISO 9787 standard. Not every kinematic object has flange frames, but
  // callers can expect this method to return one flange frame for every "robot
  // arm" contained in the kinematic object.
  std::vector<Frame*> GetIsoFlangeFrames();
  std::vector<const Frame*> GetIsoFlangeFrames() const;

  // If GetIsoFlangeFrames() returns exactly one flange frame, returns this
  // flange frame. Otherwise returns an error.
  absl::StatusOr<Frame*> GetSingleIsoFlangeFrame();
  absl::StatusOr<const Frame*> GetSingleIsoFlangeFrame() const;

  // Sets the joint positions to the given values. Expects radians (for revolute
  // joints) or meters (for prismatic joints).
  absl::Status SetJointPositions(
      const eigenmath::VectorXd& joint_positions,
      std::optional<absl::Time> timestamp = std::nullopt,
      bool enforce_monotonic_time = false);

  // Sets the joint positions to the given values. Expects radians (for revolute
  // joints) or meters (for prismatic joints). Will ignore joint limits.
  absl::Status SetJointPositions(
      const eigenmath::VectorXd& joint_positions, bool enforce_limits,
      std::optional<absl::Time> timestamp = std::nullopt,
      bool enforce_monotonic_time = false);

  // Sets the joint system limits.
  absl::Status SetJointSystemLimits(const JointLimitsXd& joint_limits,
                                    bool enforce_limits);

  // Sets the joint application limits.
  absl::Status SetJointApplicationLimits(const JointLimitsXd& joint_limits,
                                         bool enforce_limits);

  // Updates the joint system limits.
  absl::Status UpdateJointSystemLimits(
      const intrinsic_proto::JointLimitsUpdate& joint_limits,
      bool enforce_limits);

  // Updates the joint application limits.
  absl::Status UpdateJointApplicationLimits(
      const intrinsic_proto::JointLimitsUpdate& joint_limits,
      bool enforce_limits);

  // Gets the cartesian limits for this kinematic object.
  absl::StatusOr<CartesianLimits> GetCartesianLimits() const;

  // Sets the cartesian limits for this kinematic object.
  absl::Status SetCartesianLimits(const CartesianLimits& cart_limits);

  // Stores the given joint position under the given name on this kinematic
  // object as a named joint configuration, overwriting any existing
  // configuration with this name. Returns an error if the number of DoFs does
  // not match this kinematic object.
  absl::Status SetNamedJointConfiguration(
      absl::string_view name, const eigenmath::VectorXd& joint_position);

  // Removes the joint configuration with the given name from this kinematic
  // object. Returns an error if a named configuration with this name cannot be
  // found.
  absl::Status RemoveNamedJointConfiguration(absl::string_view name);

  // Gets the mounted payload for the kinematic object.
  absl::StatusOr<std::optional<RobotPayload>> GetMountedPayload() const;

  // Gets the mutable mounted payload for the kinematic object.
  absl::StatusOr<RobotPayload*> GetMutableMountedPayload();

  // Sets or clears the mounted payload for the kinematic object.
  absl::Status SetMountedPayload(std::optional<RobotPayload> payload);

  // Adds the IK solver key for the base and final tip frame of this kinematic
  // object, only works if this kinematic object has a single tip frame at the
  // end (aka it's a kinematic chain)
  absl::Status AddIkSolverKey(
      absl::string_view ik_solver,
      std::optional<std::string> tip_link_name = std::nullopt);

  // Sets the IK solver key for all existing solvable frame pairs to use
  // `ik_solver_key`.
  absl::Status SetRobotIKSolverKey(absl::string_view ik_solver_key);

  // Updates the kinematic chain for this kinematic object. The transformation
  // between the entity specified by ObjectEntityFilter to its parent is set.
  // All entities specified must be valid JointEntityId or LinkEntityId
  // entities. If the entity is a joint entity it's kinematic component
  // parent_t_inboard is set. For link entities the attachment component
  // parent_t_this is set.
  absl::Status UpdateRobotKinematics(
      const std::vector<std::pair<world::ObjectEntityFilter, Pose3d>>&
          kinematics_update);

  // Returns the kinematics chain data for this kinematic object. The data comes
  // in the form of an entity, specified via the ObjectEntityFilter, and a pose
  // which specifies the transformation of that entity to its parent.
  absl::StatusOr<std::vector<std::pair<world::ObjectEntityFilter, Pose3d>>>
  ExtractRobotKinematicsProperties() const;

  // Returns whether the kinematic chain has been updated.
  absl::StatusOr<bool> AreRobotKinematicsUpdated() const;

  absl::Status Accept(WorldObjectVisitor& visitor) override {
    return visitor.Visit(*this);
  }

  absl::Status Accept(WorldObjectConstVisitor& visitor) const override {
    return visitor.Visit(*this);
  }

  // Direct access to the underlying RobotEntityId. Before using this, consider
  // whether it makes more sense to add a new function that does not expose
  // entity level information.
  RobotCollectionsEntityId GetRobotEntityId() const { return robot_entity_id_; }

  absl::StatusOr<AttachmentEntityId>
  FinalEntityIfKinematicObjectOrElseRootEntity() const override;

  // Returns a Map of IK solver keys, by base link, then tip link.
  absl::StatusOr<WorldHashMap<AttachmentEntityId,
                              WorldHashMap<AttachmentEntityId, std::string>>>
  GetIkSolvers() const;

 private:
  absl::StatusOr<std::unique_ptr<DofKinematicView>> GetKinematicView();
  absl::StatusOr<std::unique_ptr<const DofKinematicView>> GetKinematicView()
      const;

  RobotCollectionsEntityId robot_entity_id_;
};

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_KINEMATIC_OBJECT_INTERNAL_H_
