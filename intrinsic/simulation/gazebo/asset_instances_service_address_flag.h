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

#ifndef INTRINSIC_SIMULATION_GAZEBO_ASSET_INSTANCES_SERVICE_ADDRESS_FLAG_H_
#define INTRINSIC_SIMULATION_GAZEBO_ASSET_INSTANCES_SERVICE_ADDRESS_FLAG_H_

#include <string>

#include "absl/flags/declare.h"

// If several things in the same binary want to connect to an Asset instances
// service, they should all include this header and depend on this library.
//
// Otherwise, if they happen to pick the same name for their individual flags,
// absl is unhappy. It's not allowed to define the same flag in multiple
// places!
ABSL_DECLARE_FLAG(std::string, asset_instances_service_address);

#endif  // INTRINSIC_SIMULATION_GAZEBO_ASSET_INSTANCES_SERVICE_ADDRESS_FLAG_H_
