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

#include "intrinsic/icon/control/services/assembly_from_world_service.h"

#include <memory>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/icon/control/services/assembly_service.h"
#include "intrinsic/icon/control/services/service.h"
#include "intrinsic/icon/control/services/service_registration.h"
#include "intrinsic/icon/control/services/world_service.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/world_utils.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::Status AssemblyFromWorldService::Initialize(
    const Service::InitParameters& parameters, WorldService* world_service) {
  INTRINSIC_ASSERT_NON_REALTIME();

  INTR_ASSIGN_OR_RETURN(
      auto skeleton_and_ids,
      icon::GetWholeWorldTreeSkeleton(*world_service->GetObjectWorldClient()));
  skeleton_ = std::move(skeleton_and_ids.skeleton);
  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<AssemblyFromWorldService>>
AssemblyFromWorldService::Create(const Service::InitParameters& parameters,
                                 WorldService* world_service) {
  auto service = std::make_unique<AssemblyFromWorldService>();
  INTR_RETURN_IF_ERROR(service->Initialize(parameters, world_service));
  return service;
}

const intrinsic::kinematics::Skeleton*
AssemblyFromWorldService::GetSkeletonOrNull() const {
  return skeleton_.get();
}

INTRINSIC_REGISTER_SERVICE(AssemblyFromWorldService, AssemblyService);

}  // namespace intrinsic::icon
