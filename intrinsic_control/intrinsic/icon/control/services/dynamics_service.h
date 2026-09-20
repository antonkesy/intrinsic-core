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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_DYNAMICS_SERVICE_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_DYNAMICS_SERVICE_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"

namespace intrinsic::icon {

// DynamicsService provides access to a robot's rigid body dynamics solvers.
//
// See dynamics_service_impl.cc for configuration details.
class DynamicsService {
 public:
  static constexpr char kDefaultSolverName[] = "robotics_library_dynamics";
  static constexpr bool kUseZeroGravityByDefault = false;

  // Creates a dynamics solver for part 'part_name'.
  //
  // NON-REALTIME
  //
  // Returns util::InvalidArgumentError if 1) the part is not found; 2) if the
  // part does not have dynamics associated with it; 3) if no dynamics solver
  // with name dyanmics_solver_name was registered; or 4) if dynamics solver
  // could not be created for the part.
  virtual absl::StatusOr<std::unique_ptr<icon::RigidBodyInterface>>
  CreateDynamicsSolverForPart(absl::string_view part_name) const = 0;

  virtual ~DynamicsService() = default;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_DYNAMICS_SERVICE_H_
