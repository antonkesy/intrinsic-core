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
#include "intrinsic/simulation/gazebo/plugins/generic_action/generic_action_plugin.h"

namespace intrinsic {
namespace simulation {

GZ_ADD_STATIC_PLUGIN(GenericActionPlugin, ::gz::sim::System,
                     GenericActionPlugin::ISystemConfigure,
                     GenericActionPlugin::ISystemPreUpdate,
                     GenericActionPlugin::ISystemPostUpdate);

GZ_ADD_STATIC_PLUGIN_ALIAS(GenericActionPlugin,
                           "intrinsic::simulation::GenericActionPlugin");

}  // namespace simulation
}  // namespace intrinsic
