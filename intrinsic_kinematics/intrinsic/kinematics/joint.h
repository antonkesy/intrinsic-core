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

#ifndef INTRINSIC_KINEMATICS_JOINT_H_
#define INTRINSIC_KINEMATICS_JOINT_H_

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {

// The class joints models the joint in a kinematic chain/tree. It contains the
// information of a specific degree of freedom.
class Joint : public Element {
 public:
  enum Type : uint8_t {
    REVOLUTE,   // a hinge joint that rotates along a single axis. The range of
                // motion can be limited by an upper or lower limit, or have
                // infinite limits in which case a wrap around is defined.
    PRISMATIC,  // a joint that slides along a single axis and has a limited
                // range defined by an upper and lower limit.
    FIXED  // a joint that cannot move and that does not represent a degree of
           // freedom
  };

  struct PositionLimits {
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
  };

  struct Limits {
    PositionLimits position;
    double velocity = std::numeric_limits<double>::max();
    double acceleration = std::numeric_limits<double>::max();
    double jerk = std::numeric_limits<double>::max();
    double effort = std::numeric_limits<double>::max();
  };

  // Defines a joint dependency, which is used to compute a joint's 'derived'
  // value from a weighted (linear) combination of 'input' joint values.
  // Dependencies are defined as general form q_derived[i] =
  // alpha_self*q_input[i] + alpha[i-1]*q_input[i-1]+...+
  // alpha[i-k]*q_input[i-k]. By definition 'q_derived' is a minimal joint
  // coordinate (e.g. DH parameterization) usable in standard rigid body
  // kinematics algorithms. `alpha_self` is the scaling factor for the input
  // value of the joint's own DoF. Preceding joints, that the given joint
  // depends on, are termed 'leading' joints. Non-zero scaling factors for
  // leading joints, `alpha[i-k], are defined in the map `alpha_leading`, where
  // leading joints are keyed by their kinematics::ElementId.
  struct LinearDependency {
    // Actuation of a joint's own DoF. Defaults to 1.0, must be non-zero.
    double alpha_self = 1.0;
    // Identifiers and scaling factors for the leading joints, on which this
    // joint depends.
    absl::flat_hash_map<ElementId, double> alpha_leading;
  };

  struct Parameters {
    struct Dynamics {
      // The physical static friction value of the joint.
      double static_friction = 0.0;

      // The physical damping value of the joint.
      double damping = 0.0;
    };

    // Axis of rotation/translation of the joint. Specified in the joint frame.
    eigenmath::Vector3d axis = eigenmath::Vector3d{0, 0, 1};
    // Joint type. Can be one of the following: continuous, prismatic, revolute,
    // or fixed.
    Type type = REVOLUTE;

    // Hard system limits of the joints. As defined in the urdf.
    Limits system_limits;

    // Soft motion limits for planning purposes.
    Limits soft_limits;

    Dynamics dynamics;

    // Default 'input' value for this DoF (note, this is not the derived value).
    double default_configuration = 0.0;
  };

  // Returns a joint with display 'name' and joint specific `parameters`. Parent
  // is set to nullptr and no children are defined. Linear joint dependencies
  // can be specified as optional argument `linear_dependency`. If unset, the
  // joint is not dependent.
  // For linear dependencies, an error status is returned if the set of leading
  // joints is empty (not a dependent joint), or if `alpha_self` is zero (which
  // would result in a mimicked joint, which is not supported yet).
  static absl::StatusOr<std::unique_ptr<Joint>> Create(
      absl::string_view name, const Parameters& parameters,
      std::optional<LinearDependency> linear_dependency = std::nullopt);

  // Disallow copy and move due to problematic implications to tree structure.
  Joint(const Joint&) = delete;
  Joint(Joint&&) = delete;
  Joint& operator=(const Joint&) = delete;
  Joint& operator=(Joint&&) = delete;

  // Returns the system joint position limits of the kinematic model, i.e.,
  // the position joint limits as defined in the urdf.
  const PositionLimits& GetSystemPositionLimit() const;

  // Returns the soft joint position limits of the kinematic tree used for
  // planning. By default 5% margin of the system limits if not otherwise
  // specified.
  const PositionLimits& GetSoftPositionLimit() const;

  // Returns system joint limits of the kinematic model, i.e., position,
  // velocity, acceleration, and jerk limits.
  const Limits& GetSystemLimits() const;

  // Returns soft joint limits of the kinematic model, i.e., position,
  // velocity, acceleration, and jerk limits.
  const Limits& GetSoftLimits() const;

  // Returns the type of this joint.
  Type GetType() const;

  // Returns the actuation axis of the joint
  eigenmath::Vector3d GetAxis() const;

  // Returns the full set of parameters.
  const Parameters& GetParameters() const;

  // Returns the default configuration of the joint. Default is zero.
  double GetDefaultConfiguration() const;

  bool IsStaticFrame() const override;

  // Return true if the joint has a degree of freedom.
  bool IsDof() const;

  // Compute the outbound transform of the joint, i.e., parent_t_this *
  // this_t_outbound(q_derived). This function operates on the derived
  // (=minimal) joint coordinate, where dependency information is already
  // resolved. The reason is that this function is parent-centric, and captures
  // solely a joint's outbound transform, that is the relative positioning of
  // the child link to the parent.
  Pose3d GetJointOutboundTransform(double q_derived) const;

  // Returns `true` if the joint is linearly dependent on other joints, and
  // `false` otherwise.
  bool IsDependent() const;

  // Returns a pointer to the linear dependency of the joint. Returns an error
  // status if the joint is not dependent. It is recommended to check for
  // existence of a linear dependency with `IsDependent()` before calling this
  // function.
  icon::RealtimeStatusOr<const LinearDependency*> GetLinearDependency() const;

  icon::RealtimeStatus CheckJointParameters() const;

 protected:
  // Construct a joint with display 'name' and a set of joint specific
  // `params`. Parent is set to nullptr and no children are defined. Optionally,
  // a linear dependency can be specified.
  Joint(absl::string_view name, const Parameters& params,
        std::optional<LinearDependency> linear_dependency);

  // Joint specific parameters.
  Parameters params_;
  std::optional<const LinearDependency> linear_dependency_;
};

}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_JOINT_H_
