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

#ifndef INTRINSIC_KINEMATICS_ELEMENTS_H_
#define INTRINSIC_KINEMATICS_ELEMENTS_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/pose3.h"
#include "ortools/base/strong_int.h"

namespace intrinsic {
namespace kinematics {

// Strongly typed int that holds the kinematic element id
DEFINE_STRONG_INT_TYPE(ElementId, uint32_t);

// ElementId == 0 reserved for invalid id
inline constexpr ElementId kInvalidElementId = ElementId(0);

// Abstract definition of an element of a kinematic model such as a joint, link,
// or coordinate frame. All elements have in commen that they are defined
// through a coordinate frame transform. Furthermore, each element has a parent
// and potentially one or more child elements. Parent can be a nullptr and
// children an empty set if the element is a leaf element.
class Element {
 public:
  virtual ~Element() = default;

  // Returns the transform between parent and this element, or the identity if
  // no parent exists.
  virtual Pose3d GetParentTThis() const;

  // Returns the parent element of this kinematic element. Returns an
  // absl::nullopt if no parent exist.
  virtual std::optional<const Element*> GetParentElement() const;

  // Returns the set of child elements of the kinematic element. Returns empty
  // set if no children exist.
  virtual absl::Span<const Element* const> GetChildElements() const;

  // Returns the number of child elements.
  virtual int GetNumberOfChildren() const;

  // Returns a specific child element defined by the index id of the vector of
  // child elements.This index is 0 to GetNumberOfChildren()-1. These two
  // functions provide a realtime interface to the child elements.
  virtual icon::RealtimeStatusOr<const Element*> GetChildByIndex(
      int index) const;

  // Returns display name of kinematic element.
  virtual absl::string_view GetName() const;

  // Adds a child element to this kinematic element. This method is not real
  // time safe. The method will return an InvalidArgument error if a nullptr is
  // added, or if child was already part of the element's children. This method
  // does not update the other side of the parent/child relation. Does not take
  // ownership of child, which must refer to a valid Element that outlives this
  // object.
  virtual absl::Status AddChildElement(const Element* child)
      INTRINSIC_NON_REALTIME_ONLY;

  // Removes the child element from the set of children. Returns InvalidError if
  // child does not exist.
  // Thid method does not update the other side of the parent/child relation.
  virtual absl::Status RemoveChildElement(const Element* child)
      INTRINSIC_NON_REALTIME_ONLY;

  // Sets the parent element of the kinematic element and removes the old
  // parent. Parent cannot be nullptr, will return an InvalidArgument error. Use
  // RemoveParent instead.
  // Thid method does not update the other side of the parent/child relation.
  // Does not take ownership of parent, which must refer
  // to a valid Element that outlives this object.
  virtual icon::RealtimeStatus SetParentElement(const Element* parent,
                                                const Pose3d& parent_t_this);

  // Removes the parent of the kinematic element and sets this one root.
  virtual void RemoveParent();

  // Returns true if the kinematic element has a static transform with respect
  // to its parent element. For example links and coordinate frames are static
  // frames. Joints that are not of type fixed are not static and would return
  // false;
  virtual bool IsStaticFrame() const = 0;

 protected:
  explicit Element(absl::string_view name) INTRINSIC_NON_REALTIME_ONLY;

  Element(absl::string_view name, const Element* parent,
          const Pose3d& parent_t_this,
          bool is_static_frame = true) INTRINSIC_NON_REALTIME_ONLY;

  const Element* parent_ = nullptr;
  std::vector<const Element*> children_;
  Pose3d parent_t_this_;

  const std::string name_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_ELEMENTS_H_
