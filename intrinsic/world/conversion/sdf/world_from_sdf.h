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

// API for reading SDF content (see sdformat.org) into the intrinsic::World
// library representation.
//
// We do not attempt to support the entire SDF specification, since it contains
// a lot of features that cannot be represented by the intrinsic::World data
// model and/or are not useful for our current purposes.
//
// -----------------------------------------------------------------------------
// Supported elements
// -----------------------------------------------------------------------------
//
// * <pose> elements and "name" attributes in general.
// * <world>
//   - Context: top-level element only
//   - Children: <model>
// * <model>
//   - Context: top-level element or child of <world> or <model>
//   - Children: <model>, <link>, <joint>, <static>, <plugin>
// * <link>
//   - Context: child of <model>
//   - Children: <collision>, <visual>, <inertial>, <sensor>
// * <joint>
//   - Context: child of <model>
//   - Attributes: "type"
//   - Children: <parent>, <child>, <axis>, <physics>, <sensor>
// * <inertial>
//   - Context: child of <link>
//   - Children: <mass>, <pose>, <inertia>
// * <collision>
//   - Context: child of <link>
//   - Children: <surface>, <geometry>
// * <visual>
//   - Context: child of <link>
//   - Children: <geometry>, <material>
// * <geometry>
//   - Context: child of <collision> or <visual>
//   - Children: <box>, <cylinder>, <sphere>, <mesh>
// * <sensor>
//   - Context: child of <link> or <joint>
//   - Attributes: "type"
//   - Children: <always_on>, <update_rate>, <camera>, <force_torque>, <plugin>
// * <axis>
//   - Context: child of <joint>
//   - Children: <use_parent_model_frame>, <xyz>, <limit>, <dynamics>,
//               <initial_position>
// * <limit>
//   - Context: child of <axis>
//   - Children: <upper>, <lower>, <velocity>
// * <dynamics>
//   - Context: child of <axis>
//   - Children: <damping>, <friction>
//
// -----------------------------------------------------------------------------
// Cross-model link references from joints
// -----------------------------------------------------------------------------
//
// The SDF specification is very vague about the rules for resolving <parent>
// and <child> link references from joints. SDF 1.7 introduced pose
// frame semantics that specified relative_to pose are searched from the parent
// <model>/<world> scope. We follow the same rule for joints, <parent> and
// <child> references are searched in joint's parent model scope
//
// For example, the following SDF is an acceptable (however confusing) way to
// create a chain of links from link0 to link3, connected by revolute joints:
//   <model name="outer_model">
//     <link name="link3"/>
//     <model name="middle_model">
//       <link name="link0"/>
//       <link name="link1"/>
//       <model name="inner_model">
//         <link name="link2"/>
//       </model>
//       <joint name="internal_references_only" type="revolute">
//         <parent>link0</parent>
//         <child>link1</child>
//         <axis><xyz>0 0 1</xyz></axis>
//       </joint>
//       <joint name="nested_link_reference" type="revolute">
//         <parent>link1</parent>
//         <child>inner_model::link2</child>
//         <axis><xyz>0 0 1</xyz></axis>
//       </joint>
//     </model>
//     <joint name="doubly_nested_link_reference" type="revolute">
//       <parent>middle_model::inner_model::link2</parent>
//       <child>link3</child>
//       <axis><xyz>0 0 1</xyz></axis>
//     </joint>
//   </model>

#ifndef INTRINSIC_WORLD_CONVERSION_SDF_WORLD_FROM_SDF_H_
#define INTRINSIC_WORLD_CONVERSION_SDF_WORLD_FROM_SDF_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/scene/sdf/sdf_path_resolver.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/proto/sensor_component.pb.h"
#include "intrinsic/world/world.h"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Projector.hh"
#include "sdf/Root.hh"
#include "sdf/Sensor.hh"
#include "sdf/World.hh"

namespace intrinsic {
namespace sdf {

class WorldFromSdf {
 public:
  static constexpr char kNestedModelSeparator[] = "::";
  static constexpr char kDefaultKinematicSolverKey[] = "kinematic_chain";

  // Uses SDF path resolver (see intrinsic/scene/sdf/sdf_path_resolver.h) by
  // default.
  WorldFromSdf();

  // Sets the URI resolver.
  WorldFromSdf& SetUriResolver(UriResolver uri_resolver);

  // Disable or enable the parsing of geometry. This shouldn't be common outside
  // of tests and direct kinematic model loading.
  WorldFromSdf& SetGeometryParsing(bool parse_geometry);

  // Disable or enable the creation of GroupIds for the models.
  WorldFromSdf& SetGroupIdGeneration(bool group_id_generation);

  // Parses an SDF in sdformat (https://sdformat.org) format
  // into a intrinsic::World, which can be accessed afterward through
  // GetWorld().
  absl::Status Parse(const ::sdf::Root& sdf_root);

  // Takes the contents of an SDF, parses it through sdformat, and then parses
  // that into a intrinsic::World(), which can be accessed afterward through
  // GetWorld().
  absl::Status Parse(const std::string& sdf_text);

  // Returns the World produced by the last call to Parse(). WorldFromSdf does
  // not retain a copy of the World after this function is called, so if this
  // function is called before Parse() or multiple times after Parse(), it will
  // return an error.
  absl::StatusOr<std::unique_ptr<World>> GetWorld();

  // Returns the map from fully-qualified SDF link name (i.e. a link in a nested
  // model would be named "outer_model::inner_model::link_name") to entity ID.
  //
  // Note that the entity IDs are not LinkEntityIds because SDF links often
  // represent static geometry, whereas Intrinsic LinkEntityIds are intended for
  // links within a robot / kinematic structure.
  const WorldHashMap<std::string, LinkEntityId>& GetLinkNameToEntityIdMap()
      const;

 private:
  UriResolver uri_resolver_;
  bool skip_geometry_parsing_ = false;
  bool generate_group_ids_ = false;
  std::unique_ptr<::sdf::Root> sdf_root_;
  std::unique_ptr<World> world_;
  WorldHashMap<std::string, LinkEntityId> link_name_to_entity_id_;

  // This map is used to cache each link's original world-relative pose.
  //
  // The  links in the SDF are positioned as if all <joint>s have 0
  // <initial_position>s. The corresponding link entities in the Intrinsic World
  // are moved to the correct initial poses in ParseAxis() (which is called from
  // ParseJoint()), but we need to preserve each link's original world-relative
  // transform in order to allow <joint>s to be processed in any order (see
  // b/180115665).
  WorldHashMap<AttachmentEntityId, Pose3d> link_id_to_original_world_t_link_;

  // ---------------------------------------------------------------------------
  // Static helper functions
  // ---------------------------------------------------------------------------

  // Helper function generally used after calling ParseModel() to merge a single
  // model's map into a broader one.
  static absl::Status MergeNestedLinkNameToIdMap(
      const WorldHashMap<std::string, LinkEntityId>& nested_model_map,
      const std::string& outer_model_name,
      WorldHashMap<std::string, LinkEntityId>* destination_map);

  // Helper function for ParseJoint(). Handles the fact that a <model>'s
  // links can be addressed with or without the model's name. That is, the
  // following is valid SDF:
  //
  // <model name="model_name">
  //   <link name="link0" />
  //   <link name="link1" />
  //   <joint name="joint0" type="fixed">
  //     <parent>link0</parent>
  //     <child>model_name::link1</child>
  //   </joint>
  // </model>
  static absl::StatusOr<LinkEntityId> FindLinkId(
      const std::string& model_name,
      const WorldHashMap<std::string, LinkEntityId>& link_name_to_id_map,
      const std::string& link_name);

  // Similar to FindLinkId but allows for a frame name to be used as the parent
  // name. Useful for finding the parent of a frame using 'attached_to'.
  static absl::StatusOr<AttachmentEntityId> FindParentId(
      const std::string& model_name,
      const WorldHashMap<std::string, LinkEntityId>& link_name_to_id_map,
      const WorldHashMap<std::string, AttachmentEntityId>& frame_name_to_id_map,
      const std::string& parent_name);

  // ---------------------------------------------------------------------------
  // Element-specific parsing functions.
  // ---------------------------------------------------------------------------

  // Parses a <world> element.
  absl::Status ParseWorld(const ::sdf::World& world);

  // Parses a <model> element, returns a map from <link> element names to their
  // corresponding EntityIds.
  absl::StatusOr<WorldHashMap<std::string, LinkEntityId>> ParseModel(
      const ::sdf::Model& model, const LabelId& parent_model_label,
      const Pose3d& world_t_parent_model);

  // Parses a <link> element, returning the newly created entity's ID on
  // success.
  struct ParseLinkResult {
    LinkEntityId link_id;
    std::vector<SensorEntityId> sensor_ids;
    std::vector<ProjectorEntityId> projector_ids;
  };
  absl::StatusOr<ParseLinkResult> ParseLink(const ::sdf::Link& link,
                                            const Pose3d& world_t_parent_model,
                                            const LabelId& model_label,
                                            CollectionsEntityId collections_id);

  // Parses a <joint> element, returning the newly created entity's ID on
  // success.
  struct CreateJointEntitiesResult {
    JointEntityId joint_id;
    std::vector<SensorEntityId> sensor_ids;
  };
  absl::StatusOr<CreateJointEntitiesResult> CreateJointEntities(
      const ::sdf::Joint& joint, const Pose3d& world_t_parent_model,
      const std::string& model_name, const LabelId& model_label,
      const WorldHashMap<std::string, LinkEntityId>& link_name_to_id_map,
      std::optional<CollectionsEntityId> collections_id);

  // Parses a <frame> element. If the element is marked with the attribute
  // "intrinsic::create_entity='true'", creates a single attachment entity that
  // is not part of the collection corresponding to the parent model (if there
  // is a parent model). Note that the resulting entity will be recognized as a
  // "Frame" in the object-world view.
  absl::StatusOr<AttachmentEntityId> ParseFrame(
      const ::sdf::Frame& frame, const std::string& model_name,
      const Pose3d& world_t_parent_model, const LabelId& model_label,
      WorldHashMap<std::string, AttachmentEntityId>& frame_name_to_id_map,
      const WorldHashMap<std::string, LinkEntityId>& link_name_to_id_map);

  // Parses a <sensor> element and returns the ID of a newly created sensor
  // entity (attached to parent_id).
  absl::StatusOr<SensorEntityId> CreateSensorEntity(
      const ::sdf::Sensor& sensor, AttachmentEntityId parent_id,
      std::optional<CollectionsEntityId> collections_id);

  // Parses a <projector> element stored in link, returns the ID of a newly
  // created entity (attached to parent_id)
  absl::StatusOr<ProjectorEntityId> ParseProjector(
      const ::sdf::Projector& projector, AttachmentEntityId parent_id,
      std::optional<CollectionsEntityId> collections_id);

  // ---------------------------------------------------------------------------
  // Other helper functions
  // ---------------------------------------------------------------------------
  absl::Status BypassFixedCrossModelJoint(JointEntityId joint_id);
  absl::Status BypassFixedInModelJoint(JointEntityId joint_id);
};

}  // namespace sdf
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_CONVERSION_SDF_WORLD_FROM_SDF_H_
