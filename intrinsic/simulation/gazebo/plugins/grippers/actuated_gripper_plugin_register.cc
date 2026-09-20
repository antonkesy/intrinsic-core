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

#include "gz/plugin/RegisterStatic.hh"
#include "gz/sim/System.hh"
#include "intrinsic/simulation/gazebo/plugins/grippers/actuated_gripper_plugin.h"

namespace intrinsic {
namespace simulation {

GZ_ADD_STATIC_PLUGIN(ActuatedGripperPlugin, ::gz::sim::System,
                     ActuatedGripperPlugin::ISystemConfigure,
                     ActuatedGripperPlugin::ISystemPreUpdate,
                     ActuatedGripperPlugin::ISystemPostUpdate)

GZ_ADD_STATIC_PLUGIN_ALIAS(ActuatedGripperPlugin,
                           "intrinsic::simulation::ActuatedStickyGripperPlugin")

}  // namespace simulation
}  // namespace intrinsic
