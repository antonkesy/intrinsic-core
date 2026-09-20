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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_PID_JOINT_UTIL_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_PID_JOINT_UTIL_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "gz/math/PID.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "sdf/Element.hh"

namespace intrinsic {
namespace simulation {

// The closed loop frequency used to compute PID position control gains for a
// joint if no gains are specified in the SDF.
constexpr double kTargetPIDJointPositionClosedLoopFreqRadPerSec = 15;

// A joint entity and a pid that helps control the joint.
struct PIDJoint {
  ::gz::sim::Entity joint_entity;
  ::gz::math::PID pid;
  std::string joint_name;
};

// Get PID controlled joints from a model entity and SDF plugin XML element.
//
// The <plugin> element should be a child of the <model> element. Only joints
// mentioned in the plugin XML are considered PID controlled.
absl::StatusOr<std::vector<PIDJoint>> GetPIDJoints(
    const ::gz::sim::Entity& entity, const ::sdf::Element* sdf_element,
    ::gz::sim::EntityComponentManager& ecm);

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_GRIPPERS_PID_JOINT_UTIL_H_
