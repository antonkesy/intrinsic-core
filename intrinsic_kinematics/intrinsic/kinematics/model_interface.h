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

#ifndef INTRINSIC_KINEMATICS_MODEL_INTERFACE_H_
#define INTRINSIC_KINEMATICS_MODEL_INTERFACE_H_

#include <map>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/node_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/state_values.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic {
namespace kinematics {

// An abstract interface for a kinematic structure that is input to the Inverse
// Kinematics solver. It guarantees access to the relevant components.
// For each implementation of a model interface, we currently assume:
// - A joint does not necessarily represent a degree of freedom (dof), i.e., we
// can have joints that are fully dependent on the values of other joints or
// fixed.
// - Each joint can at most represent one dof.
// kinematic model represents a kinematic tree, i.e., it is not a chain where we
// can easily numerate each degree of freedom (dof). The mapping between the ids
// and element id will be provided by the following two functions:
// - icon::RealtimeStatusOr<int> GetDofIndexForElementId(const ElementId& id)
// - icon::RealtimeStatusOr<ElementId> GetElementIdForDofIndex(int dof_index)
//      const
class ModelInterface {
 public:
  explicit ModelInterface(absl::string_view name,
                          const Pose3d& offset = Pose3d::Identity());
  virtual ~ModelInterface() = default;

  // Returns the display name of the kinematic model.
  absl::string_view GetName() const;

  // Returns system joint limits of the kinematic model, i.e. position,
  // velocity, acceleration, and jerk limits. The returned limits are the limits
  // on the "input" joint values, i.e. the limits on the user-facing
  // coordinate, not the derived values.
  JointLimits GetDofSystemLimits() const;

  // Returns the kinematic element that is a common ancestor of two kinematic
  // elements specified as input.
  icon::RealtimeStatusOr<ElementId> GetCommonAncestor(
      const ElementId& element_a, const ElementId& element_b) const;

  // Returns the kinematic element that is a common ancestor of all given
  // kinematic elements
  icon::RealtimeStatusOr<ElementId> GetCommonAncestor(
      absl::Span<const ElementId> element_ids) const;

  // Returns the element id within the model for the element identified that has
  // the name provided. Returns a NotFound if no such element exist. Returns an
  // internal error if multiple elements with this name exist.
  // TODO(kmuelling): Handle multiple elements with the same name (b/173144824).
  icon::RealtimeStatusOr<ElementId> FindElementIdByName(
      absl::string_view name) const;

  // Returns the element ids within the model for the element identified that
  // has a name that contains the provide name.
  std::vector<ElementId> FindElementIdByNameContains(
      absl::string_view name) const;

  // Get a kinematic element from id. Returns an invalid argument if the
  // kinematic element does not exist.
  // Does not transfer ownership of the kinematic element. Pointer only valid as
  // long this object continues to exist.
  // TODO(kmuelling): Convert into something like in the world with typedresult.
  virtual icon::RealtimeStatusOr<const Element*> GetElement(
      const ElementId& id) const = 0;

  // Get the element id of an element.
  virtual icon::RealtimeStatusOr<ElementId> GetElementId(
      const Element* element) const = 0;

  // Get the element name. If the id is not valid, an error name is returned.
  std::string GetElementName(const ElementId& id) const;

  // Returns the Joint with the specified index. Returns an invalid argument if
  // the kinematic element does not exist.
  icon::RealtimeStatusOr<const Joint*> GetJoint(ElementId id) const;

  // Returns the Link for the specified index. Returns an invalid argument if
  // the kinematic element does not exist.
  icon::RealtimeStatusOr<const Link*> GetLink(ElementId id) const;

  // Returns the dof index for an element id. The dof_index is the index of the
  // element id all StateP and Limit vectors provided in the interface.
  // Please note that this only includes element ids that are counted as degrees
  // of freedom and are not fixed joints or fully dependent joints.
  virtual icon::RealtimeStatusOr<int> GetDofIndexForElementId(
      const ElementId& id) const = 0;

  // Returns the element id for the dof index. The dof_index is the index of the
  // element id of all state and limit vectors provided by this interface.
  // Please note that this only includes element ids that are counted as degrees
  // of freedom and are not fixed joints or fully dependent joints.
  virtual icon::RealtimeStatusOr<ElementId> GetElementIdForDofIndex(
      int dof_index) const = 0;

  // Returns ids of all kinematic components to iterate through them. he
  // lifetime of the returned span is limited to the lifetime of the kinematic
  // model.
  absl::Span<const ElementId> GetAllElementIds() const;

  // Returns ids of all joints in the model. This function returns all 'true'
  // joints that are counted as degree of freedom and not as a fixed joint. he
  // lifetime of the returned span is limited to the lifetime of the kinematic
  // model.
  //
  // The order of the ElementIds is the model's DoF index order (i.e. the same
  // one that joints appear in, for example, the output of
  // `GetDofSystemLimits()`)
  absl::Span<const ElementId> GetDofIds() const;

  // Returns ids of all elements of type joint. This also includes fixed joints.
  // The lifetime of the returned span is limited to the lifetime of the
  // kinematic model.
  absl::Span<const ElementId> GetAllJointIds() const;

  // Returns ids of all elements of type link.
  // The lifetime of the returned span is limited to the lifetime of the
  // kinematic model.
  absl::Span<const ElementId> GetAllLinkIds() const;

  // Returns ids of all elements of type coordinate frame.
  // The lifetime of the returned span is limited to the lifetime of the
  // kinematic model.
  absl::Span<const ElementId> GetAllCoordinateFrameIds() const;

  // Returns all kinematic IK solver keys defined for the kinematic model. The
  // lifetime of the returned span is limited to the lifetime of the kinematic
  // model.
  absl::Span<const std::string> GetAllSolverKeys(
      const ElementId& start_frame, const ElementId& end_frame) const;

  // Sets the IK solver keys for the two model elements.
  absl::Status SetSolverKey(
      const ElementId& start_frame, const ElementId& end_frame,
      absl::string_view solver_key) INTRINSIC_NON_REALTIME_ONLY;

  // Gets all the IK solver keys
  absl::node_hash_map<ElementId,
                      absl::node_hash_map<ElementId, std::vector<std::string>>>
  GetSolverKeys() const {
    return ik_solver_keys_;
  }

  // Gets all ids of the kinematic elements that are considered a 'tip', i.e.,
  // the leaves within the kinematic structure.
  absl::Span<const ElementId> GetTipIds() const;

  // Gets the id of the base of the kinematic structure, i.e., the root.
  ElementId GetBaseId() const;

  // Returns a vector of ids corresponding to the DoF in the chain from base to
  // a provided tip, identified by `element_id`. Tip should be one of the
  // element returned by GetTipIds.
  // TODO(jeanfrancoisd): We could also accept a dof as a tip.
  icon::RealtimeStatusOr<absl::Span<const ElementId>> GetDofChainForTip(
      const ElementId& tip_element_id) const;

  // Returns a vector of ids corresponding to the DoF in the chain from the base
  // to a provided element, identified by `element_id`.
  icon::RealtimeStatusOr<
      FixedVector<ElementId, eigenmath::VectorNd::MaxSizeAtCompileTime>>
  GetDofChainForElement(const ElementId& element_id) const;

  // Returns the number of degrees of the kinematic object.
  int GetNumberDegreesOfFreedom() const;

  // Returns true if the model contained is kinematic chain with one tip.
  bool HasOneTip() const;

  // Returns true if the model is a non-branching kinematic chain.
  // This method will not count attached frames as branches in the model.
  bool IsNonBranchingKinematicChain() const;

  // Finds the tip of a non-branching kinematic chain in the same sense as
  // `IsNonBranchingKinematicChain()` above. That is, this method *does not*
  // consider attached frames, but only joints and links.
  //
  // Because this method ignores attached frames, the return value might not be
  // one of the tip IDs reported by `GetTipIds()` (because those might be
  // attached frames themselves).
  //
  // Returns FailedPreconditionError if there are multiple branches in the
  // kinematic chain.
  icon::RealtimeStatusOr<kinematics::ElementId>
  FindNonBranchingKinematicChainTip() const;

  // Returns the default state of the kinematic system.
  icon::RealtimeStatusOr<StateValues> GetDefaultStateValues() const;

  // Clamps the joint values to the range provided by the system position limits
  // for the kinematic structure. The function takes the joint position state as
  // input and modifies it such that the joint limits will be within the system
  // position limits. If a joint value exceeds the limits, it will be set to the
  // limit value.
  icon::RealtimeStatus ClampToSystemLimits(JointStateP& joint_values) const;

  // Clamps the joint values to the range provided by the soft position limits
  // for the kinematic structure. The function takes the joint position state as
  // input and modifies it such that the joint limits will be within the soft
  // position limits. If a joint value exceeds the limits, it will be set to the
  // limit value.
  icon::RealtimeStatus ClampToSoftLimits(JointStateP& joint_values) const;

  // Computes a joint with `joint_id`'s derived (output) value based upon the
  // provided `dof_input_values` for the model. The derived value is the value
  // that is used to compute the joint's outbound transform and needs to follow
  // a minimal parameterization. Joint dependencies get resolved in this
  // function, therefore the derived value may be vastly different from the
  // input value for a dependent joint, and should not be checked against the
  // joint limits applying to the input value.
  // TODO(b/425672686): develop concept for limits handling on the derived
  // value.
  icon::RealtimeStatusOr<double> ComputeJointsDerivedValue(
      const ElementId& joint_id, const JointStateP& dof_input_values) const;

  // Returns the offset of the anchor point of this model. This is not the
  // offset of the base element to the anchor point.
  Pose3d GetAnchorPointOffset() const;

  // If the joint with id `joint_id` is dependent, this function validates the
  // set of leading joints in the linear dependency. This includes checking that
  // the leading joints are DoFs, that they are "preceding" the following
  // joint with `joint_id`, and that they are not dependent joints themselves.
  absl::Status ValidateLinearDependency(ElementId joint_id);

  // Returns true if the model has dependent joints, and false otherwise.
  icon::RealtimeStatusOr<bool> HasDependentJoints() const;

  // Returns the joint dependency matrix for the model, which maps DoF input
  // values to DoF derived values. The joint dependency matrix is
  // square and its dimension matches the number of degrees of freedom of the
  // kinematic model. Because joints are only allowed to depend on "preceding"
  // joints in a kinematic chain, it is a lower-triangular matrix.
  // Note: this is a computationally efficient real-time function, which only
  // performs several map lookup operations.
  icon::RealtimeStatusOr<eigenmath::MatrixNd> GetJointDependencyMatrix() const;

  // Returns the inverse of the joint dependency matrix, mapping DoF derived
  // values to DoF input values. Because joints are only allowed to depend on
  // "preceding" joints in a kinematic chain, it is a lower-triangular matrix.
  // Note: this is a computationally expensive real-time function. It
  // performs a matrix inversion. Please prefer caching the result rather than
  // calling this function repeatedly.
  icon::RealtimeStatusOr<eigenmath::MatrixNd> GetJointDependencyMatrixInverse()
      const;

  // Returns true if `ancestor_to_check` is an ancestor of `child` or equal to
  // it. Returns false otherwise.
  icon::RealtimeStatusOr<bool> IsAncestorOf(ElementId child,
                                            ElementId ancestor_to_check) const;

 protected:
  // These should be accessed through the default state.
  JointLimits GetDofSoftPositionLimits() const;
  JointStateP GetDefaultConfiguration() const;

  // Name of the model. Currently this does not need to be unique.
  std::string name_;

  // Offset of the base element with respect to some coordinate origin.
  Pose3d skeleton_base_offset_;

  // Mapping from the element_id to the degree of freedom index that will be
  // used to store the joint state values and limits in a linear vector.
  // TODO(b/421367043): clarify real-time compatibility.
  absl::flat_hash_map<ElementId, int> element_id_to_dof_index_;

  // Id of the base element of the kinematic model. This implies that a
  // model always has one unique base element;
  ElementId base_element_id_ = kInvalidElementId;

  // Data structure that gives quick access to the element ids that represents
  // the tips, i.e., the leaves of the tree structure.
  std::vector<ElementId> tip_ids_;

  // Data structure that stores which element ids are of type LINK.
  std::vector<ElementId> links_;

  // Data structure that stores which element ids within the skeleton are of
  // type JOINT. This includes fixed and dependent joints, i.e., joints that are
  // not necessarily considered a degree of freedom.
  std::vector<ElementId> joints_;

  // Data structure that stores which element ids are of type CF.
  std::vector<ElementId> coordinate_frames_;

  // HELPER DATA STRUCTURES

  // Helper data structure to provide quick access to map from dof index to
  // element id of the respective joint.
  std::vector<ElementId> dof_index_to_joint_id_;

  // Helper data structure to provide easy access to all elements ids.
  // Includes links, joints, and coordinate frames.
  std::vector<ElementId> element_ids_;

  // The chain of dof from base to tip for each tips in the skeleton.
  // The key to the map are all the tips returned by GetTipIds().
  std::map<ElementId, std::vector<ElementId>> tip_dof_chains_;

  // 2D map from [base_id][tip_id] to the set of ik solver keys available.
  absl::node_hash_map<ElementId,
                      absl::node_hash_map<ElementId, std::vector<std::string>>>
      ik_solver_keys_;
};

// Returns ok if `child` is rigidly attached to `ancestor`.
//
// If `ancestor` is a joint, its own dof is not considered (i.e., we consider
// its outboard frame). For example:
//
//   parent1 --> joint1 (movable) --> joint2 (fixed) --> child1
//
// In this example:
//   * child1 is rigidly attached to joint2,
//   * child1 is rigidly attached joint1, because it's rigidly attached to
//     joint1 outboard frame, and joint2 is fixed
//   * child1 is NOT rigidly attached to parent1, because the chain goes through
//     joint1's dof.
//
// Returns an error with the name of a non-fixed joint if one is found. Also
// returns an error if `child` is not a descendant of `ancestor`.
icon::RealtimeStatus IsAncestorRigidlyAttached(const ModelInterface& model,
                                               const ElementId& child,
                                               const ElementId& ancestor);

// Returns ok if the elements are rigidly attached, and otherwise an error that
// includes the name of one of the moveable joints in between them.
icon::RealtimeStatus AreElementsRigidlyAttached(const ModelInterface& model,
                                                const ElementId& e1,
                                                const ElementId& e2);

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_MODEL_INTERFACE_H_
