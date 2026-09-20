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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_DYNAMICS_FROM_KINEMATICS_SERVICE_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_DYNAMICS_FROM_KINEMATICS_SERVICE_H_

#include <memory>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/services/dynamics_service.h"
#include "intrinsic/icon/control/services/kinematics_service.h"
#include "intrinsic/icon/control/services/service.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/server/config/services_config.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/kinematics/skeleton.h"

namespace intrinsic::icon {

// Implementation of DynamicsService.
//
// Some things to note about this implementation:
//
// 1) Users must provide a resource_id or part_name for
//    each part they desire a dynamics solver for. `solver_name` and
//    `use_zero_gravity` are optional parameters.
//    DynamicsFromKinematicsService's initialization will fail with an error if
//    * `name` is missing
//    * `use_zero_gravity` is not a boolean value
//    * `solver_name` does not match any registered dynamics solver's name.
//
// 2) Each resource_id or part_name tag must match one of the parts available
// via
//    KinematicsService. If this is not the case, DynamicsFromKinematicsService
//    will return an error at initialization time.
//
// 3) Calls to the service must specify the part name. The service will respond
//    with either a unique pointer to an initialized RBD instance, or an error
//    describing why the call failed.
//
// 4) Multiple calls to the dynamics service with the same part name will return
//    a fresh RBD solver instance. These calls *are not* realtime safe!
struct DynamicsInfo {
  std::string solver_name;
  std::unique_ptr<kinematics::Skeleton> skeleton;
  bool use_zero_gravity;
};

class DynamicsFromKinematicsService
    : public ServiceImplBase<DynamicsFromKinematicsService, DynamicsService,
                             KinematicsService> {
 public:
  static absl::StatusOr<std::unique_ptr<DynamicsFromKinematicsService>> Create(
      const Service::InitParameters& parameters,
      KinematicsService* kinematics_service,
      const intrinsic_proto::icon::DynamicsServiceConfig& proto_config);

  absl::StatusOr<std::unique_ptr<RigidBodyInterface>>
  CreateDynamicsSolverForPart(absl::string_view part_name) const override
      INTRINSIC_NON_REALTIME_ONLY;

 private:
  absl::Status Initialize(
      const Service::InitParameters& parameters,
      KinematicsService* kinematics_service,
      const intrinsic_proto::icon::DynamicsServiceConfig& proto_config);

  absl::flat_hash_map<std::string, DynamicsInfo> part_dynamics_info_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_DYNAMICS_FROM_KINEMATICS_SERVICE_H_
