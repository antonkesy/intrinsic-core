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

#ifndef INTRINSIC_KINEMATICS_LINK_H_
#define INTRINSIC_KINEMATICS_LINK_H_

#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {
// Representation of a link in a kinematic structure. It holds the following
// information:
// - a name of the link
// - a pose relative to the parent frame
// - the parent frame
// - physics properties of the element
class Link : public Element {
 public:
  struct Parameters {
    // Transform from this link to the center of gravity.
    Pose3d center_of_gravity = Pose3d::Identity();

    // 3x3 inertia matrix represented in the inertial frame, i.e.,
    // the center of gravity. Expressed in kilogram metres squared
    eigenmath::Matrix3d inertia = eigenmath::Matrix3d::Zero();

    // The mass of the link in kg.
    double mass = 0.0;
  };

  // Construct a link as kinematic element with minimal
  // information: Display name. Parent and child set to nullptr.
  Link(absl::string_view name, const Parameters& params);

  // Disallow copy and move due to problematic implications to tree structure.
  Link(const Link&) = delete;
  Link(Link&&) = delete;
  Link& operator=(const Link&) = delete;
  Link& operator=(Link&&) = delete;

  // Get the set of parameters
  const Parameters& GetParameters() const;

  // Always true for link.
  bool IsStaticFrame() const override;

 protected:
  // Link parameters. Includes mass, center of gravity and inertia.
  Parameters params_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_LINK_H_
