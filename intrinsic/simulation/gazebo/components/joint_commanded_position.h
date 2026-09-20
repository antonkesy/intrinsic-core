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

#ifndef INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_JOINT_COMMANDED_POSITION_H_
#define INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_JOINT_COMMANDED_POSITION_H_

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>
#include <vector>

namespace intrinsic::simulation {

/// \brief Commanded joint positions in SI units (rad for revolute, m for
/// prismatic). The component wraps a std::vector of size equal to the degrees
/// of freedom of the joint.
using JointCommandedPosition = gz::sim::components::Component<
    std::vector<double>, class JointCommandedPositionTag,
    gz::sim::serializers::VectorDoubleSerializer>;

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_JOINT_COMMANDED_POSITION_H_
