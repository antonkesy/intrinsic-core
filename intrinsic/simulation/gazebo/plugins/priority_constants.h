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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_PRIORITY_CONSTANTS_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_PRIORITY_CONSTANTS_H_

#include "gz/sim/System.hh"

// This file contains constants that define integer priority values used to
// control the order of execution of the PreUpdate and Update methods of
// Gazebo systems (the execution order of other callbacks is not affected). The
// default priority value is zero, and systems with smaller priority values are
// executed earlier. Systems with the same priority value are executed in the
// order in which they are loaded.
//
// For more information, see
// https://github.com/gazebosim/gz-sim/blob/gz-sim9/include/gz/sim/System.hh.
namespace intrinsic::simulation::plugins {

// Configure HardwareModuleLauncher to execute before the Physics system to
// ensure that HardwareModuleLauncher::Update calls GazeboHwm::RequestIconTick
// before Physics::Update. This allows the Icon Tick to run in parallel with
// Gazebo's Physics Update.
constexpr gz::sim::System::PriorityType kHardwareModuleLauncherPriority =
    gz::sim::systems::kPrePhysicsPriority;

// Configure ServiceStateAggregator to execute before HardwareModuleLauncher
// to ensure that ServiceStateAggregator::PreUpdate has a chance to update
// the ServiceState components before HardwareModuleLauncher::PreUpdate
// attempts to use them.
constexpr gz::sim::System::PriorityType kServiceStateAggregatorPriority =
    kHardwareModuleLauncherPriority - 10;
}  // namespace intrinsic::simulation::plugins
#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_PRIORITY_CONSTANTS_H_
