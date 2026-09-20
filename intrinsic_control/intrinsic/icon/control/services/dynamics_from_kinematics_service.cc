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

#include "intrinsic/icon/control/services/dynamics_from_kinematics_service.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/services/dynamics_service.h"
#include "intrinsic/icon/control/services/kinematics_service.h"
#include "intrinsic/icon/control/services/service.h"
#include "intrinsic/icon/control/services/service_registration.h"
#include "intrinsic/icon/dynamics/dynamics_factory.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/server/config/services_config.pb.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::Status DynamicsFromKinematicsService::Initialize(
    const Service::InitParameters& parameters,
    KinematicsService* kinematics_service,
    const intrinsic_proto::icon::DynamicsServiceConfig& proto_config) {
  INTRINSIC_ASSERT_NON_REALTIME();

  std::string solver_name = DynamicsService::kDefaultSolverName;
  if (!proto_config.has_solver_name() && !proto_config.solver_name().empty()) {
    solver_name = proto_config.solver_name();
  }

  bool use_zero_gravity = DynamicsService::kUseZeroGravityByDefault;
  if (proto_config.has_use_zero_gravity()) {
    use_zero_gravity = proto_config.use_zero_gravity();
  }

  std::vector<std::string> parts_to_load;
  if (!proto_config.part_name().empty()) {
    parts_to_load = {proto_config.part_name()};
  } else {
    parts_to_load = kinematics_service->ListPartNames();
  }
  if (parts_to_load.empty()) {
    return absl::InvalidArgumentError(
        "DynamicsServiceConfig has an empty `part_name`, and no parts were "
        "found via KinematicsService or running as a resource.");
  }

  for (const std::string& name : parts_to_load) {
    std::unique_ptr<kinematics::Skeleton> skeleton =
        kinematics_service->CreateKinematicsModelForPart(name);
    if (skeleton == nullptr) {
      return absl::FailedPreconditionError(absl::StrCat(
          "KinematicsService returned null skeleton for part \"", name, "\"."));
    }

    if (bool inserted =
            part_dynamics_info_
                .try_emplace(name,
                             DynamicsInfo{.solver_name = solver_name,
                                          .skeleton = std::move(skeleton),
                                          .use_zero_gravity = use_zero_gravity})
                .second;
        !inserted) {
      return absl::AlreadyExistsError(
          absl::StrCat("Duplicate part name: ", name));
    }
    INTRINSIC_RT_LOG(INFO) << "Registered dynamics solver \"" << solver_name
                           << "\" for part \"" << name << "\".";
  }

  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<DynamicsFromKinematicsService>>
DynamicsFromKinematicsService::Create(
    const Service::InitParameters& parameters,
    KinematicsService* kinematics_service,
    const intrinsic_proto::icon::DynamicsServiceConfig& proto_config) {
  auto service = std::make_unique<DynamicsFromKinematicsService>();
  INTR_RETURN_IF_ERROR(
      service->Initialize(parameters, kinematics_service, proto_config));
  return service;
}

absl::StatusOr<std::unique_ptr<RigidBodyInterface>>
DynamicsFromKinematicsService::CreateDynamicsSolverForPart(
    absl::string_view part_name) const {
  INTRINSIC_ASSERT_NON_REALTIME();
  auto part = part_dynamics_info_.find(part_name);
  if (part == part_dynamics_info_.end()) {
    return absl::NotFoundError(
        absl::StrCat("No part added with name ", part_name, "."));
  }

  // Create a clone of the skeleton to pass ownership to the dynamics
  // implementation.
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<RigidBodyInterface> dynamics_impl,
      icon::GetGlobalDynamicsFactory().CreateDynamicsImpl(
          part->second.solver_name, part->second.skeleton->Clone()));

  if (part->second.use_zero_gravity) {
    dynamics_impl->SetGravity(eigenmath::Vector3d::Zero());
  }
  return dynamics_impl;
}

INTRINSIC_REGISTER_SERVICE(DynamicsFromKinematicsService, DynamicsService);

}  // namespace intrinsic::icon
