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

#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_utils.h"

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler.h"
#include "intrinsic/motion_planning/path_planning/data_structures/state_sampler_configs.pb.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

absl::StatusOr<std::string> GetStateSamplerAlias(
    const proto::StateSamplerSpecification& spec) {
  return spec.name();
}

}  // namespace

absl::StatusOr<std::unique_ptr<StateSampler>> CreateStateSampler(
    const proto::StateSamplerSpecification& spec,
    const KinematicsSystemProxy& proxy) {
  INTR_ASSIGN_OR_RETURN(const std::string state_sampler_alias,
                        GetStateSamplerAlias(spec));

  return StateSamplerRegistry::Dispatch(std::string(state_sampler_alias), spec,
                                        proxy);
}

}  // namespace intrinsic
