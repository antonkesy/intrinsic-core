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

#include "intrinsic/icon/control/services/kinematics_from_world_service.h"

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
#include "absl/types/span.h"
#include "intrinsic/icon/control/services/kinematics_service.h"
#include "intrinsic/icon/control/services/service.h"
#include "intrinsic/icon/control/services/service_registration.h"
#include "intrinsic/icon/control/services/world_service.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/world_utils.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_factory.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/object_world_ids.h"

namespace intrinsic::icon {

absl::Status KinematicsFromWorldService::Initialize(
    const Service::InitParameters& parameters, WorldService* world_service,
    bool allow_non_real_time_inverse_kinematics) {
  control_frequency_hz_ = parameters.control_frequency_hz;
  allow_non_real_time_inverse_kinematics_ =
      allow_non_real_time_inverse_kinematics;
  std::shared_ptr<const world::ObjectWorldClient> world_client =
      world_service->GetObjectWorldClient();
  INTR_ASSIGN_OR_RETURN(std::vector<WorldObjectName> object_names,
                        world_client->ListObjectNames());
  for (const auto& name : object_names) {
    absl::StatusOr<world::KinematicObject> kinematic_object =
        world_client->GetKinematicObject(name);
    if (!kinematic_object.ok()) {
      continue;
    }
    if (skeleton_by_part_name_.contains(kinematic_object->Name().value())) {
      return absl::AlreadyExistsError(absl::StrCat(
          "World contains multiple robot entities named '",
          kinematic_object->Name().value(), "', but names must be unique."));
    }
    INTR_ASSIGN_OR_RETURN(auto skeleton_and_ids,
                          icon::GetSkeletonForObject(*world_client, name));

    skeleton_by_part_name_[kinematic_object->Name().value()] =
        std::move(skeleton_and_ids.skeleton);
  }

  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<KinematicsFromWorldService>>
KinematicsFromWorldService::Create(
    const Service::InitParameters& parameters, WorldService* world_service,
    bool allow_non_real_time_inverse_kinematics) {
  auto service = std::make_unique<KinematicsFromWorldService>();
  INTR_RETURN_IF_ERROR(service->Initialize(
      parameters, world_service, allow_non_real_time_inverse_kinematics));
  return service;
}

std::unique_ptr<kinematics::InverseKinematicsInterface>
KinematicsFromWorldService::CreateInverseKinematicsSolverForPart(
    absl::string_view part_name) const {
  INTRINSIC_ASSERT_NON_REALTIME();
  auto skeleton_it = skeleton_by_part_name_.find(part_name);
  if (skeleton_it == skeleton_by_part_name_.end()) {
    INTRINSIC_RT_LOG(ERROR)
        << "No kinematics data for part '" << part_name << "'";
    return nullptr;
  }
  absl::StatusOr<kinematics::Chain> kinematic_chain =
      absl::NotFoundError(absl::StrCat(
          "Part '", part_name,
          "' has no solvable kinematics chains. Have you added IK solvers?"));
  // Use the first solvable chain we find. Usually, models only have one
  // solvable chain, and using that allows us to reliably pick base and tip IDs,
  // even if the model has multiple tips.
  bool found_solvable_chain = false;
  for (const auto& [base_id, tip_to_solver_map] :
       skeleton_it->second->GetSolverKeys()) {
    for (const auto& [tip_id, solver_keys] : tip_to_solver_map) {
      if (solver_keys.empty()) {
        continue;
      }
      if (found_solvable_chain == true) {
        INTRINSIC_RT_LOG(WARNING)
            << "Part '" << part_name
            << "' has more than one solvable chain. We've picked one at "
               "random, but check your configuration to see whether you've "
               "specified more solver keys than you intend to.";
        continue;
      }
      kinematic_chain = kinematics::CreateChainFromModel(*skeleton_it->second,
                                                         base_id, tip_id);
    }
  }
  if (!kinematic_chain.ok()) {
    INTRINSIC_RT_LOG(ERROR)
        << "Failed to build kinematic chain for part '" << part_name
        << "': " << kinematic_chain.status().ToString();
    return nullptr;
  }

  const kinematics::InverseKinematicsInterface::Options options{
      .frequency = control_frequency_hz_};

  const absl::Span<const std::string> solver_keys =
      skeleton_it->second->GetAllSolverKeys(kinematic_chain->GetBaseId(),
                                            kinematic_chain->GetTipId());
  for (const std::string& solver_key : solver_keys) {
    // Build the solver, and check if it's realtime safe.
    auto ik_solver = kinematics::GetGlobalInverseKinematicsFactory()
                         .CreateInverseKinematicsSolver(
                             solver_key, *kinematic_chain, options);
    if (!ik_solver.ok()) {
      INTRINSIC_RT_LOG(ERROR)
          << "Failed to build solver '" << solver_key << "' for part '"
          << part_name << "': " << ik_solver.status().ToString();
      return nullptr;
    }

    if (!ik_solver.value()->IsRealtimeSafe()) {
      if (allow_non_real_time_inverse_kinematics_) {
        INTRINSIC_RT_LOG(INFO)
            << "Solver '" << solver_key
            << "' is not realtime safe, but we're allowing it because "
               "allow_non_real_time_inverse_kinematics_ is true.";
      } else {
        // If the solver is not realtime safe, continue until we find one that
        // is.
        INTRINSIC_RT_LOG(INFO)
            << "Solver '" << solver_key
            << "' is not realtime safe, looking for another solver that is.";
        continue;
      }
    }

    // If it is, return it.
    return std::move(*ik_solver);
  }
  INTRINSIC_RT_LOG(ERROR)
      << "Failed to find a realtime safe solver, returning nullptr";
  return nullptr;
}

// Creates a kinematics model interface pointer by part name.
//
// NON-REALTIME
//
// Returns nullptr if 1) the part is not found; 2) if the part does not have
// kinematics associated with it;
std::unique_ptr<kinematics::Skeleton>
KinematicsFromWorldService::CreateKinematicsModelForPart(
    absl::string_view part_name) const {
  INTRINSIC_ASSERT_NON_REALTIME();
  auto skeleton_it = skeleton_by_part_name_.find(part_name);
  if (skeleton_it == skeleton_by_part_name_.end()) {
    return nullptr;
  }
  return skeleton_it->second->Clone();
}

// Returns a pointer to the ModelInterface for the part, or nullptr if the
// part is not found or it does not have an associated kinematics model.
const kinematics::ModelInterface*
KinematicsFromWorldService::GetKinematicsModelForPart(
    absl::string_view part_name) const {
  auto skeleton_it = skeleton_by_part_name_.find(part_name);
  if (skeleton_it == skeleton_by_part_name_.end()) {
    return nullptr;
  }
  return skeleton_it->second.get();
}

std::vector<std::string> KinematicsFromWorldService::ListPartNames() const {
  std::vector<std::string> part_names;
  part_names.reserve(skeleton_by_part_name_.size());
  for (const auto& [name, _] : skeleton_by_part_name_) {
    part_names.push_back(name);
  }
  return part_names;
}

INTRINSIC_REGISTER_SERVICE(KinematicsFromWorldService, KinematicsService);

}  // namespace intrinsic::icon
