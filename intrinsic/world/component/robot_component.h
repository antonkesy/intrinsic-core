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

#ifndef INTRINSIC_WORLD_COMPONENT_ROBOT_COMPONENT_H_
#define INTRINSIC_WORLD_COMPONENT_ROBOT_COMPONENT_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/robot_component.pb.h"
#include "intrinsic/world/robot_payload/robot_payload.h"

namespace intrinsic {

// A component to hold information about a kinematic system with movable dofs
// and ik/fk solvers.
class RobotComponent {
 public:
  virtual ~RobotComponent() = default;

  // Returns a new RobotComponent instance.
  static std::unique_ptr<RobotComponent> Create();

  // Returns a new RobotComponent instance derived from the given proto.
  static absl::StatusOr<std::unique_ptr<RobotComponent>> FromProto(
      const intrinsic_proto::world::RobotComponent& proto);

  // Returns a new RobotComponent instance that is a copy of this one.
  virtual std::unique_ptr<RobotComponent> Clone() const = 0;

  // Returns a proto representation of this component.
  virtual absl::StatusOr<intrinsic_proto::world::RobotComponent> ToProto()
      const = 0;

  // Returns the pairs of EntityIds that the robot can solve for. For example a
  // standard robotics arm will have one pair represented by the base and tip of
  // the arm.
  virtual WorldHashSet<std::pair<AttachmentEntityId, AttachmentEntityId>>
  GetSolvableFrames() const = 0;

  // Returns the solver key that can solve between the two given frames. If no
  // solver key is setup for the pair it will return a not found error.
  virtual absl::StatusOr<std::string> GetSolverKeyForFrames(
      AttachmentEntityId base, AttachmentEntityId tip) const = 0;

  // Adds a new entity pair that the robot can solve using the given solver key.
  // The order of the frames is important as the poses given to the solver key
  // will use base_t_tip. The frames must be part of this robot. If the pair
  // already exists we will override the existing solver key with the given key.
  //
  // Note: This function can also operate in a compatibility mode, which
  // EntityRobots uses to replicate behaviors of the original Robots aspect
  // implementation. This mode is enabled by setting base to kInvalidEntityId,
  // at which that point, the frame will be added without any other checks.
  virtual absl::Status AddSolvableFrames(AttachmentEntityId base,
                                         AttachmentEntityId tip,
                                         absl::string_view solver_key) = 0;

  // Removes an entity pair that the robot can solve.
  virtual absl::Status RemoveSolvableFrames(AttachmentEntityId base,
                                            AttachmentEntityId tip) = 0;

  // Gets the named configuration if one exists with the given name.
  virtual absl::StatusOr<eigenmath::VectorXd> GetNamedConfiguration(
      absl::string_view named_configuration) const = 0;

  // Gets the set of named configurations available in the component.
  virtual WorldHashMap<std::string, eigenmath::VectorXd>
  GetNamedConfigurations() const = 0;

  // Overrides all of the named configurations for this component.
  virtual void SetNamedConfigurations(
      WorldHashMap<std::string, eigenmath::VectorXd>&& named_configuration) = 0;

  // Adds or overrides a single named configuration for this component.
  virtual void SetNamedConfiguration(absl::string_view name,
                                     eigenmath::VectorXd configuration) = 0;

  // Removes a named configuration if one exists with the given name.
  virtual void RemoveNamedConfiguration(absl::string_view name) = 0;

  // Gets specifications for ICON sim devices.
  //
  // NOTE: The number of devices returned could be 0.
  virtual absl::StatusOr<
      std::vector<intrinsic_proto::world::RobotComponent::IconSimDevice>>
  GetIconSimDevices() const = 0;

  // Gets the current set of CartesianLimits for the robot.
  virtual const CartesianLimits& GetCartesianLimits() const = 0;

  // Sets the current set of CartesianLimits for the robot.
  virtual absl::Status SetCartesianLimits(
      const CartesianLimits& cart_limits) = 0;

  // Sets specifications for ICON sim devices.
  //
  // NOTE: The number of devices set could be 0.
  virtual void SetIconSimDevices(
      const std::vector<intrinsic_proto::world::RobotComponent::IconSimDevice>&
          devices) = 0;

  // Gets the specification for ICON sim plugin.
  virtual absl::StatusOr<
      std::optional<intrinsic_proto::world::RobotComponent::IconSimPluginSpec>>
  GetIconSimPluginSpec() const = 0;

  // Sets the specification for ICON sim plugin.
  virtual void SetIconSimPluginSpec(
      const intrinsic_proto::world::RobotComponent::IconSimPluginSpec&
          spec) = 0;

  // Removes the specification for ICON sim plugin.
  virtual void RemoveIconSimPluginSpec() = 0;

  // Gets the specification for multi camera plugin.
  virtual absl::StatusOr<std::optional<
      intrinsic_proto::world::RobotComponent::MultiCameraPluginSpec>>
  GetMultiCameraPluginSpec() const = 0;

  // Sets the specification for multi camera plugin.
  virtual void SetMultiCameraPluginSpec(
      const intrinsic_proto::world::RobotComponent::MultiCameraPluginSpec&
          spec) = 0;

  // Removes the specification for multi camera plugin.
  virtual void RemoveMultiCameraPluginSpec() = 0;

  // Gets the specification for generic action plugin.
  virtual absl::StatusOr<std::optional<
      intrinsic_proto::world::generic_action::GenericActionPluginSpec>>
  GetGenericActionPluginSpec() const = 0;

  // Sets the specification for generic action plugin.
  virtual void SetGenericActionPluginSpec(
      std::optional<
          intrinsic_proto::world::generic_action::GenericActionPluginSpec>
          spec) = 0;

  // Updates the component based on the given proto. This will do a full
  // override, if something is missing from this proto it will override any
  // existing values with the defaults.
  virtual absl::Status UpdateFromProto(
      const intrinsic_proto::world::RobotComponent& proto) = 0;

  // Updates all ids within the collection using the mapping. Fails if the
  // mapping doesn't include all entities referenced.
  virtual absl::Status RekeyIds(
      const WorldHashMap<EntityId, EntityId>& id_mapping) = 0;

  // Gets the mutable mounted payload if it exists.
  virtual std::optional<RobotPayload>& GetMountedPayload() = 0;

  // Gets the mounted payload if it exists.
  virtual const std::optional<RobotPayload>& GetMountedPayload() const = 0;

  // Sets or clears the mounted payload.
  virtual absl::Status SetMountedPayload(
      std::optional<RobotPayload> payload) = 0;

  // Returns true if the robot kinematics are updated.
  virtual bool AreKinematicsUpdated() const = 0;

  // Sets whether the robot kinematics are updated.
  virtual void SetAreKinematicsUpdated(bool are_kinematics_updated) = 0;

  // TODO(stoyang): private means not yet implemented.
 private:
  // Gets the DofKinematicView of the robot.
  virtual absl::StatusOr<std::unique_ptr<DofKinematicView>>
  CreateDofKinematicView() = 0;
  virtual absl::StatusOr<std::unique_ptr<const DofKinematicView>>
  CreateDofKinematicView() const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_COMPONENT_ROBOT_COMPONENT_H_
