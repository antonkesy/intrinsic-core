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

#ifndef INTRINSIC_KINEMATICS_IK_INVERSE_KINEMATICS_FACTORY_H_
#define INTRINSIC_KINEMATICS_IK_INVERSE_KINEMATICS_FACTORY_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik_interface.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"

namespace intrinsic {
namespace kinematics {

// This class encapsulates three registries for: (1) general non-realtime safe
// IK solvers, (2) realtime-safe IK solvers and (3) constrained IK solvers.
// Since realtime-safe solvers and constrained solvers are also general solvers,
// any solver registered as (2) or (3) is automatically registered as (1)
// non-realtime safe as well. The converse is not true: a solver that is only
// registered as non-realtime safe with RegisterNonRealtimeIKSolver cannot be
// retrieved with CreateRealtimeInverseKinematicsSolver or
// CreateConstrainedInverseKinematicsSolver.
class InverseKinematicsFactory {
 public:
  using CreateSignature =
      absl::StatusOr<std::unique_ptr<InverseKinematicsInterface>>(
          kinematics::Chain chain,
          const InverseKinematicsInterface::Options& options);
  using CreateRealtimeSignature =
      absl::StatusOr<std::unique_ptr<InverseKinematicsInterfaceRT>>(
          kinematics::Chain chain,
          const InverseKinematicsInterface::Options& options);
  using CreateConstrainedSignature =
      absl::StatusOr<std::unique_ptr<ConstrainedIKInterface>>(
          kinematics::Chain chain,
          const InverseKinematicsInterface::Options& options);

  // Returns a unique_ptr to a new inverse kinematics solver implementing
  // InverseKinematicsInterface. Returns a status error if no creator was
  // registered for the provided solver key or if the creator fails. The
  // returned solver implements InverseKinematicsInterface and may or may not be
  // real-time capable. However, it is guaranteed to implement the non-realtime
  // ComputeIK methods. If you specifically need a realtime solver, use the
  // CreateRealtimeInverseKinematicsSolver function.
  absl::StatusOr<std::unique_ptr<InverseKinematicsInterface>>
  CreateInverseKinematicsSolver(
      absl::string_view solver_name, kinematics::Chain chain,
      const InverseKinematicsInterface::Options& options =
          InverseKinematicsInterface::Options()) const;

  // Same as above, but the returned solver is real-time safe and implements the
  // InverseKinematicsInterfaceRT interface.
  // Returns nullptr if no real-time safe solver is registered with
  // `solver_name` or if the creator function fails.
  absl::StatusOr<std::unique_ptr<InverseKinematicsInterfaceRT>>
  CreateRealtimeInverseKinematicsSolver(
      absl::string_view solver_name, kinematics::Chain chain,
      const InverseKinematicsInterface::Options& options =
          InverseKinematicsInterface::Options()) const;

  // Returns a unique_ptr to a new constrained IK solver implementing
  // ConstrainedIKInterface. Returns a status error if no constrained solver had
  // been registered with `solver_name` or if the creator fails.
  absl::StatusOr<std::unique_ptr<ConstrainedIKInterface>>
  CreateConstrainedInverseKinematicsSolver(
      absl::string_view solver_name, kinematics::Chain chain,
      const InverseKinematicsInterface::Options& options =
          InverseKinematicsInterface::Options()) const;

  // Registers a non-realtime inverse kinematics solver, by specifying the
  // function 'create_func` that creates the solver. Returns true on successful
  // registration, otherwise false if the key already existed.
  absl::Status RegisterNonRealtimeIKSolver(
      absl::string_view name, std::function<CreateSignature> create_func);

  // Same as above, but this registers the solver as both realtime safe and
  // non-realtime safe. I.e. a solver registered with this function will be
  // accessible via both CreateInverseKinematicsSolver and
  // CreateRealtimeInverseKinematicsSolver.
  absl::Status RegisterRealtimeIKSolver(
      absl::string_view name,
      std::function<CreateRealtimeSignature> create_func);

  // Registers a constrained IK solver, by specifying the
  // function 'create_func` that creates the solver.  Additionally, this solver
  // is automatically registered as a canonical non-realtime solver as well.
  // I.e. a solver registered with this function will be accessible via both
  // CreateInverseKinematicsSolver and CreateConstrainedInverseKinematicsSolver.
  // Returns ok on successful registration, otherwise a status error.
  absl::Status RegisterConstrainedIKSolver(
      absl::string_view name,
      std::function<CreateConstrainedSignature> create_func);

  // Returns a vector with the keys of the IK solvers that have been registered.
  std::vector<std::string> GetRegisteredIKSolverKeys() const;

  // Returns a vector with the keys of the realtime-safe IK solvers that have
  // been registered.
  std::vector<std::string> GetRegisteredRealtimeIKSolverKeys() const;

  // Returns a vector with the keys of the constrained IK solvers that have been
  // registered.
  std::vector<std::string> GetRegisteredConstrainedIKSolverKeys() const;

 private:
  // Removes all registrations.
  void Clear();

  // Registry of functions and mutex used to construct the IK solvers.
  // Only using one mutex for all registries, if that becomes an issue, we can
  // split it up.
  mutable absl::Mutex mutex_;
  absl::flat_hash_map<std::string, std::function<CreateSignature>>
      non_realtime_creator_registry_ ABSL_GUARDED_BY(mutex_);
  absl::flat_hash_map<std::string, std::function<CreateRealtimeSignature>>
      realtime_creator_registry_ ABSL_GUARDED_BY(mutex_);
  absl::flat_hash_map<std::string, std::function<CreateConstrainedSignature>>
      constrained_creator_registry_ ABSL_GUARDED_BY(mutex_);

  friend class InverseKinematicsFactoryPeer;
};

// Access the global registry.
InverseKinematicsFactory& GetGlobalInverseKinematicsFactory();

}  // namespace kinematics
}  // namespace intrinsic
#endif  // INTRINSIC_KINEMATICS_IK_INVERSE_KINEMATICS_FACTORY_H_
