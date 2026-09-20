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

#ifndef INTRINSIC_KINEMATICS_SKELETON_H_
#define INTRINSIC_KINEMATICS_SKELETON_H_

#include <memory>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/coordinate_frame.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// A kinematic skeleton that contains a set of joints, links, and coordinate
// frames that belong to one control unit such as a robot.
//
// Joints and links are listed in this skeleton in the order that they are
// added. Because joints can only be added to existing parent links, and
// likewise for links and their parent joints, all joints and links are
// topologically sorted starting from the root element. This affects the
// `GetAllJointIds` and `GetAllLinkIds` methods.
class Skeleton : public ModelInterface {
 public:
  // Create an empty kinematic skeleton. This can be used as a basis to manually
  // construct a kinematic skeleton.
  explicit Skeleton(absl::string_view name,
                    const Pose3d& offset = Pose3d::Identity());

  std::unique_ptr<Skeleton> Clone() const INTRINSIC_NON_REALTIME_ONLY;

  //*********************
  // Functions to create a kinematic skeleton.
  //*********************

  // Adds the kinematic component to the kinematic skeleton and attaches it to
  // the parent component. Returns an InvalidArgument error if parent element
  // not found.
  absl::StatusOr<ElementId> AddJoint(
      std::unique_ptr<Joint> element,
      const ElementId& parent_id = kInvalidElementId,
      const Pose3d& parent_t_this = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Adds the kinematic component of type joint with a predefined ElementId to
  // the kinematic skeleton and attaches it to the parent component. Returns an
  // InvalidArgumentError if parent element not found or the specified
  // ElementId already exist in the skeleton.
  absl::Status AddJointWithId(std::unique_ptr<Joint> element,
                              const ElementId& new_element_id,
                              const ElementId& parent_id = kInvalidElementId,
                              const Pose3d& parent_t_this = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Adds the kinematic component to the kinematic skeleton and attaches it to
  // the parent component. Returns an InvalidArgument error if parent element
  // not found.
  absl::StatusOr<ElementId> AddLink(
      std::unique_ptr<Link> element,
      const ElementId& parent_id = kInvalidElementId,
      const Pose3d& parent_t_this = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Adds the kinematic component of type link with a predefined ElementId to
  // the kinematic skeleton and attaches it to the parent component. Returns an
  // InvalidArgumentError if parent element not found or the specified
  // ElementId already exist in the skeleton.
  absl::Status AddLinkWithId(std::unique_ptr<Link> element,
                             const ElementId& new_element_id,
                             const ElementId& parent_id = kInvalidElementId,
                             const Pose3d& parent_t_this = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Adds the kinematic component to the kinematic skeleton and attaches it to
  // the parent component. Returns an InvalidArgument error if parent element
  // not found.
  absl::StatusOr<ElementId> AddCoordinateFrame(
      std::unique_ptr<CoordinateFrame> element,
      const ElementId& parent_id = kInvalidElementId,
      const Pose3d& parent_t_this = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Adds the kinematic component of type CoordinateFrame with a predefined
  // ElementId to the kinematic skeleton and attaches it to the parent
  // component. Returns an InvalidArgumentError if parent element not found or
  // the specified ElementId already exist in the skeleton.
  absl::Status AddCoordinateFrameWithId(
      std::unique_ptr<CoordinateFrame> element, const ElementId& new_element_id,
      const ElementId& parent_id = kInvalidElementId,
      const Pose3d& parent_t_this = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Creates a joint with the specified parameters and attaches it to the parent
  // component specified.
  absl::StatusOr<ElementId> CreateJoint(
      absl::string_view name, const Joint::Parameters& parameters,
      const ElementId& parent_id = kInvalidElementId,
      const Pose3d& parent_t_joint = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Creates a joint with the specified `parameters` and a `linear_dependency`,
  // and attaches it to the parent component specified.
  absl::StatusOr<ElementId> CreateJoint(
      absl::string_view name, const Joint::Parameters& parameters,
      const Joint::LinearDependency& linear_dependency,
      const ElementId& parent_id = kInvalidElementId,
      const Pose3d& parent_t_joint = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Creates a joint with the specified joint parameters and new_element_id. It
  // will be attached to the specified parent. Returns an InvalidArgumentError
  // if parent element not found or the specified ElementId already exist in the
  // skeleton.
  absl::Status CreateJointWithId(
      absl::string_view name, const Joint::Parameters& parameters,
      const ElementId& new_element_id,
      const ElementId& parent_id = kInvalidElementId,
      const Pose3d& parent_t_joint = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Creates a link with the specified parameters and attaches is to the parent
  // component specified.
  absl::StatusOr<ElementId> CreateLink(
      absl::string_view name, const Link::Parameters& parameters,
      const ElementId& parent = kInvalidElementId,
      const Pose3d& parent_t_link = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Creates a link with the specified link parameters and new_element_id. It
  // will be attached to the specified parent. Returns an InvalidArgumentError
  // if parent element not found or the specified ElementId already exist in the
  // skeleton.
  absl::Status CreateLinkWithId(
      absl::string_view name, const Link::Parameters& parameters,
      const ElementId& new_element_id,
      const ElementId& parent = kInvalidElementId,
      const Pose3d& parent_t_link = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Creates a coordinate frame with the specified parameters and attaches it
  // within the kinematic tree at the specified parent node.
  absl::StatusOr<ElementId> CreateCoordinateFrame(
      absl::string_view name, const ElementId& parent_id = kInvalidElementId,
      const Pose3d& parent_t_cf = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  // Creates a coordinate frame with the specified new_element_id. It will
  // be attached to the specified parent. Returns an InvalidArgumentError if
  // parent element not found or the specified ElementId already exist in the
  // skeleton.
  absl::Status CreateCoordinateFrameWithId(
      absl::string_view name, const ElementId& new_element_id,
      const ElementId& parent_id = kInvalidElementId,
      const Pose3d& parent_t_cf = Pose3d::Identity())
      INTRINSIC_NON_REALTIME_ONLY;

  //*********************
  // Functionality to generate and request kinematic sub-groups.
  //*********************

  //*********************
  // Functions to query the skeleton joint properties
  //*********************

  icon::RealtimeStatusOr<int> GetDofIndexForElementId(
      const ElementId& id) const override;

  icon::RealtimeStatusOr<ElementId> GetElementIdForDofIndex(
      int dof_index) const override;

  icon::RealtimeStatusOr<const Element*> GetElement(
      const ElementId& id) const override;

  icon::RealtimeStatusOr<ElementId> GetElementId(
      const Element* element) const override;

 private:
  // Adds the kinematic component to the kinematic skeleton and attaches it to
  // the parent component. Returns an InvalidArgument error if parent element
  // not found.
  // This method is not real time safe.
  absl::Status AddElement(
      std::unique_ptr<Element> element, const ElementId& new_element_id,
      const ElementId& parent_id,
      const Pose3d& parent_t_this) INTRINSIC_NON_REALTIME_ONLY;

  ElementId GenerateElementId();

  // Mapping from the element id and the elements of the kinematic model.
  absl::flat_hash_map<ElementId, std::unique_ptr<Element>>
      element_id_to_element_;

  // Helper data structure to store the last id that was used in a continuous
  // manner.
  ElementId last_id_ = ElementId(0);
};

}  // namespace kinematics
}  // namespace intrinsic
#endif  // INTRINSIC_KINEMATICS_SKELETON_H_
