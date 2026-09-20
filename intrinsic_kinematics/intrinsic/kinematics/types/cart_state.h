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

#ifndef INTRINSIC_KINEMATICS_TYPES_CART_STATE_H_
#define INTRINSIC_KINEMATICS_TYPES_CART_STATE_H_

#include <cstdint>
#include <tuple>
#include <type_traits>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/aggregate_type.h"

namespace intrinsic {

namespace cart_state_details {

struct CartStateBaseP {
  Pose3d pose;
};
struct CartStateBaseV {
  eigenmath::Vector6d velocity = eigenmath::Vector6d::Zero();
};
struct CartStateBaseA {
  eigenmath::Vector6d acceleration = eigenmath::Vector6d::Zero();
};
struct CartStateBaseJ {
  eigenmath::Vector6d jerk = eigenmath::Vector6d::Zero();
};
struct CartStateBaseT {
  uint64_t time_nsec = 0;
};

}  // namespace cart_state_details

template <typename... Bases>
struct CartState : public AggregateType<Bases...> {
  using AggregateType<Bases...>::AggregateType;

  // Constructs a partially initialized AggregateType from all viable
  // types in other State
  template <typename... OtherBases,
            typename = std::enable_if_t<aggregate_type_details::IsSubsetOf<
                std::tuple<OtherBases...>, std::tuple<Bases...>>::value>>
  static CartState<Bases...> From(const CartState<OtherBases...>& other) {
    CartState<Bases...> retv;
    ((static_cast<OtherBases&>(retv) = static_cast<OtherBases>(other)), ...);
    return retv;
  }
};

using CartStateP = CartState<cart_state_details::CartStateBaseP>;
using CartStateV = CartState<cart_state_details::CartStateBaseV>;
using CartStateA = CartState<cart_state_details::CartStateBaseA>;
using CartStateJ = CartState<cart_state_details::CartStateBaseJ>;
using CartStateVA = CartState<cart_state_details::CartStateBaseV,
                              cart_state_details::CartStateBaseA>;
using CartStatePV = CartState<cart_state_details::CartStateBaseP,
                              cart_state_details::CartStateBaseV>;
using CartStatePVA = CartState<cart_state_details::CartStateBaseP,
                               cart_state_details::CartStateBaseV,
                               cart_state_details::CartStateBaseA>;
using CartStatePVAJ = CartState<
    cart_state_details::CartStateBaseP, cart_state_details::CartStateBaseV,
    cart_state_details::CartStateBaseA, cart_state_details::CartStateBaseJ>;
using CartStateVAJ = CartState<cart_state_details::CartStateBaseV,
                               cart_state_details::CartStateBaseA,
                               cart_state_details::CartStateBaseJ>;

/// The translational and angular distance to a Cartesian pose
struct CartesianDistance {
  CartesianDistance() = default;
  CartesianDistance(const double lin, const double ang)
      : linear(lin), angular(ang) {}
  /// Euclidean norm of translational distance to target
  double linear = 0;
  /// absolute angle of rotation to target orientation
  double angular = 0;
};

/// The translational and angular velocity relative to a Cartesian twist
struct RelativeCartesianVelocity {
  RelativeCartesianVelocity() = default;
  RelativeCartesianVelocity(const double lin, const double ang)
      : linear(lin), angular(ang) {}
  /// Euclidean norm of relative translational velocity to target.
  double linear = 0;
  /// Euclidean norm of relative angular velocity to target.
  double angular = 0;
};

// A composite state of 3D orientation representation in `quaternion`,
// together with the corresponding `angular_velocity` and
// `angular_acceleration`. The `angular_velocity` lies in the Lie Algebra or
// the tangent space of the Special Orthogonal Group SO(3) manifold at
// `quaternion`. Hence, `angular_velocity` and `angular_acceleration` are
// local w.r.t. `quaternion`.
struct QuaternionStateQVA {
  QuaternionStateQVA() = default;
  QuaternionStateQVA(const eigenmath::Quaterniond& quaternion,
                     const eigenmath::Vector3d& angular_velocity,
                     const eigenmath::Vector3d& angular_acceleration)
      : quaternion(quaternion),
        angular_velocity(angular_velocity),
        angular_acceleration(angular_acceleration) {}
  eigenmath::Quaterniond quaternion;
  eigenmath::Vector3d angular_velocity;
  eigenmath::Vector3d angular_acceleration;
};

// A composite state of 3D orientation representation in `quaternion`,
// together with the corresponding `angular_velocity`. The `angular_velocity`
// lies in the Lie Algebra or the tangent space of the Special Orthogonal
// Group SO(3) manifold at `quaternion`. Hence, `angular_velocity` is local
// w.r.t. `quaternion`.
struct QuaternionStateQV {
  QuaternionStateQV() = default;
  QuaternionStateQV(const eigenmath::Quaterniond& quaternion,
                    const eigenmath::Vector3d& angular_velocity)
      : quaternion(quaternion), angular_velocity(angular_velocity) {}
  eigenmath::Quaterniond quaternion;
  eigenmath::Vector3d angular_velocity;
};

// A state of the 3D angular velocity representation. The `angular_velocity`
// lies in the Lie Algebra or the tangent space of the Special Orthogonal
// Group SO(3) manifold of the associated quaternion. Hence,
// `angular_velocity` is local w.r.t. the associated quaternion.
struct QuaternionStateV {
  QuaternionStateV() = default;
  explicit QuaternionStateV(const eigenmath::Vector3d& angular_velocity)
      : angular_velocity(angular_velocity) {}
  eigenmath::Vector3d angular_velocity;
};

// A state of 3D orientation representation in `quaternion` only, to be
// consistent with the QuaternionStateQV and QuaternionStateQVA states.
struct QuaternionStateQ {
  QuaternionStateQ() = default;
  explicit QuaternionStateQ(const eigenmath::Quaterniond& quaternion)
      : quaternion(quaternion) {}
  eigenmath::Quaterniond quaternion;
};

}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_TYPES_CART_STATE_H_
