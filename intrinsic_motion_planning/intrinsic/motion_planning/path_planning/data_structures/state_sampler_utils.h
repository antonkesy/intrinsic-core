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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_STATE_SAMPLER_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_STATE_SAMPLER_UTILS_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"

namespace intrinsic {

// This function is used to instantiate a registered StateSamplers from
// a generic specification, the degree of freedoms of the robot for which the
// state sampler is initiated, and a validator functions that performs a
// check to ensure if the generated configuration is within the joint limits.
//
// This function returns an error if any state sampler fails to instantiate, or
// the implementation cannot find a registered state sampler to instantiate from
// the configuration.
absl::StatusOr<std::unique_ptr<StateSampler>> CreateStateSampler(
    const proto::StateSamplerSpecification& spec,
    const KinematicsSystemProxy& proxy);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_STATE_SAMPLER_UTILS_H_
