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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SET_MODEL_STATE_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SET_MODEL_STATE_H_

#include <memory>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Types.hh"
#include "sdf/Element.hh"

namespace intrinsic::simulation {

// This system should be attached to a model and sets a specified
// model state during Configure and Reset using the syntax shown below.
// Currently joint positions and velocities can be set for named joints.
// By default, the units for position are radians for rotational joints
// and meters for translational joints. Likewise the units for velocity
// are radians / second for rotational joints and meters / second
// for translational joints. If the `degrees` attribute is set to "true"
// for a given scalar value, the value will be scaled by (π / 180)
// before being interpreted using default units.
// TODO(scpeters): warn if degrees==true for a translational joint.
/* \code{.xml}
  <model_state>
    <joint_state name="joint_0">
      <axis_state>
        <position degrees="true">60</position>
        <velocity degrees="true">-30</velocity>
      </axis_state>
    </joint_state>
    <joint_state name="joint_1">
      <axis_state>
        <position>1.0471975512</position>
        <velocity>-0.5235987756</velocity>
      </axis_state>
    </joint_state>
  </model_state>
\endcode */
//
// ## Components
//
// This system uses the following components:
//
// - ::gz::sim::components::JointPositionReset
// - ::gz::sim::components::JointVelocityReset
// TODO(scpeters@): add components to reset link pose and velocity
class SetModelState final : public ::gz::sim::System,
                            public ::gz::sim::ISystemConfigure,
                            public ::gz::sim::ISystemReset {
 public:
  void Configure(const ::gz::sim::Entity& entity,
                 const std::shared_ptr<const ::sdf::Element>& sdf,
                 ::gz::sim::EntityComponentManager& ecm,
                 ::gz::sim::EventManager& eventMgr) override;

  void Reset(const ::gz::sim::UpdateInfo& info,
             ::gz::sim::EntityComponentManager& ecm) override;

 private:
  ::gz::sim::Model model_{::gz::sim::kNullEntity};
};
}  // namespace intrinsic::simulation
#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SET_MODEL_STATE_H_
