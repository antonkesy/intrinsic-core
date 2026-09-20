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

#include "intrinsic/kinematics/ik/inverse_kinematics_factory.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik_interface.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {

namespace {

// Returns a vector of keys associated to the solvers in `solver_registry`.
template <typename T>
std::vector<std::string> GetSolverKeys(const T& solver_registry) {
  std::vector<std::string> solver_keys;
  for (auto& [solver_key, unused_solver_fn] : solver_registry) {
    solver_keys.push_back(std::move(solver_key));
  }
  return solver_keys;
}

}  // namespace

absl::StatusOr<std::unique_ptr<InverseKinematicsInterface>>
InverseKinematicsFactory::CreateInverseKinematicsSolver(
    absl::string_view solver_name, kinematics::Chain chain,
    const InverseKinematicsInterface::Options& options) const {
  absl::MutexLock lock(mutex_);
  auto it = non_realtime_creator_registry_.find(solver_name);
  if (it == non_realtime_creator_registry_.end()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("No IK solver registered with name %s", solver_name));
  }
  INTR_ASSIGN_OR_RETURN(auto solver, it->second(std::move(chain), options));
  if (solver == nullptr) {
    return absl::InternalError(
        absl::StrFormat("The creator for solver %s should have returned a "
                        "non-null unique_ptr but it didn't.",
                        solver_name));
  }
  return std::move(solver);
}

absl::StatusOr<std::unique_ptr<InverseKinematicsInterfaceRT>>
InverseKinematicsFactory::CreateRealtimeInverseKinematicsSolver(
    absl::string_view solver_name, kinematics::Chain chain,
    const InverseKinematicsInterface::Options& options) const {
  absl::MutexLock lock(mutex_);
  auto it = realtime_creator_registry_.find(solver_name);
  if (it == realtime_creator_registry_.end()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "No realtime IK solver registered with name %s", solver_name));
  }
  INTR_ASSIGN_OR_RETURN(auto solver, it->second(std::move(chain), options));
  if (solver == nullptr) {
    return absl::InternalError(
        absl::StrFormat("The creator for solver %s should have returned a "
                        "non-null unique_ptr but it didn't. ",
                        solver_name));
  }
  return std::move(solver);
}

absl::StatusOr<std::unique_ptr<ConstrainedIKInterface>>
InverseKinematicsFactory::CreateConstrainedInverseKinematicsSolver(
    absl::string_view solver_name, kinematics::Chain chain,
    const InverseKinematicsInterface::Options& options) const {
  absl::MutexLock lock(mutex_);
  auto it = constrained_creator_registry_.find(solver_name);
  if (it == constrained_creator_registry_.end()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "No constrained IK solver registered with name %s", solver_name));
  }
  INTR_ASSIGN_OR_RETURN(auto solver, it->second(std::move(chain), options));
  if (solver == nullptr) {
    return absl::InternalError(
        absl::StrFormat("The creator for solver %s should have returned a "
                        "non-null unique_ptr but it didn't. ",
                        solver_name));
  }
  return std::move(solver);
}

absl::Status InverseKinematicsFactory::RegisterNonRealtimeIKSolver(
    absl::string_view name, std::function<CreateSignature> create_func) {
  absl::MutexLock lock(mutex_);
  if (non_realtime_creator_registry_.contains(name)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Registering non-RT solver with key '%s' failed. "
        "Another solver had already been registered with such key.",
        name));
  }
  non_realtime_creator_registry_.emplace(std::string(name),
                                         std::move(create_func));
  return absl::OkStatus();
}

absl::Status InverseKinematicsFactory::RegisterRealtimeIKSolver(
    absl::string_view name,
    std::function<CreateRealtimeSignature> create_func) {
  std::function<CreateSignature> non_realtime_create_func = create_func;
  absl::MutexLock lock(mutex_);
  if (realtime_creator_registry_.contains(name)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Registering RT solver with key '%s' failed. "
        "Another solver had already been registered with such key.",
        name));
  }
  if (non_realtime_creator_registry_.contains(name)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Registering realtime solver with key '%s' as non-realtime failed, "
        "even though real-time registration was successful. "
        "Another non-realtime solver had already "
        "been registered with such key.",
        name));
  }
  realtime_creator_registry_.emplace(std::string(name), std::move(create_func));
  non_realtime_creator_registry_.emplace(std::string(name),
                                         std::move(non_realtime_create_func));
  return absl::OkStatus();
}

absl::Status InverseKinematicsFactory::RegisterConstrainedIKSolver(
    absl::string_view name,
    std::function<CreateConstrainedSignature> create_func) {
  absl::MutexLock lock(mutex_);
  if (constrained_creator_registry_.contains(name)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Registering constrained solver with key '%s' failed. "
        "Another solver had already been registered with such key.",
        name));
  }
  if (non_realtime_creator_registry_.contains(name)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Registering constrained solver with key '%s' as non-realtime "
        "failed, "
        "even though 'constrained' registration was successful. "
        "Another non-realtime solver had already "
        "been registered with such key.",
        name));
  }
  std::function<CreateSignature> non_realtime_create_func = create_func;
  constrained_creator_registry_.emplace(std::string(name),
                                        std::move(create_func));
  non_realtime_creator_registry_.emplace(std::string(name),
                                         std::move(non_realtime_create_func));
  return absl::OkStatus();
}

std::vector<std::string> InverseKinematicsFactory::GetRegisteredIKSolverKeys()
    const {
  absl::MutexLock lock(mutex_);
  return GetSolverKeys(non_realtime_creator_registry_);
}

std::vector<std::string>
InverseKinematicsFactory::GetRegisteredRealtimeIKSolverKeys() const {
  absl::MutexLock lock(mutex_);
  return GetSolverKeys(realtime_creator_registry_);
}

std::vector<std::string>
InverseKinematicsFactory::GetRegisteredConstrainedIKSolverKeys() const {
  absl::MutexLock lock(mutex_);
  return GetSolverKeys(constrained_creator_registry_);
}

void InverseKinematicsFactory::Clear() {
  absl::MutexLock lock(mutex_);
  non_realtime_creator_registry_.clear();
  realtime_creator_registry_.clear();
  constrained_creator_registry_.clear();
}

InverseKinematicsFactory& GetGlobalInverseKinematicsFactory() {
  INTRINSIC_ASSERT_NON_REALTIME();
  static auto* r = new InverseKinematicsFactory;  // go/totw/110.
  return *r;
}

}  // namespace kinematics
}  // namespace intrinsic
