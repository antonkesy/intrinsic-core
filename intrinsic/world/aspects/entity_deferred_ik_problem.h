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

#ifndef INTRINSIC_WORLD_ASPECTS_ENTITY_DEFERRED_IK_PROBLEM_H_
#define INTRINSIC_WORLD_ASPECTS_ENTITY_DEFERRED_IK_PROBLEM_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/aspects/entity_world_interface.h"
#include "intrinsic/world/cartesian_kinematic_view.h"
#include "intrinsic/world/entity_id.h"

namespace intrinsic {
namespace entity_kinematic_world_details {

// This is an implementation of IkSolutions that has a precomputed set of
// solutions and will expose those through the IkSolutions interface.
class EntityDeferredIKProblem : public IkSolutions {
 public:
  explicit EntityDeferredIKProblem(
      const entity_aspect_world_details::EntityWorld& world,
      AttachmentEntityId base, AttachmentEntityId tip, const Pose3d& base_t_tip,
      const eigenmath::VectorXd* q_init = nullptr);

  // Since more than one robot is involved in the IK problem, the solver key
  // need to be passed directly.
  EntityDeferredIKProblem(const entity_aspect_world_details::EntityWorld& world,
                          const std::string& solver_key,
                          AttachmentEntityId base, AttachmentEntityId tip,
                          const Pose3d& base_t_tip,
                          const eigenmath::VectorXd* q_init = nullptr);

  std::optional<eigenmath::VectorXd> GetSingleSolution() const override;

  // Attempts to compute an IK solution which is on the same kinematic branch as
  // the initialized joint state.
  std::optional<eigenmath::VectorXd> GetSameBranchSolution() const override;

  std::vector<eigenmath::VectorXd> GetSampledSolutions(
      size_t max_num_solutions,
      std::optional<const FilterPolicy*> filter) const override;

 private:
  std::string solver_key_;
  std::unique_ptr<kinematics::Skeleton> skeleton_;
  Pose3d base_t_tip_;

  eigenmath::VectorXd* q_init_ = nullptr;
  // In the event that we get a q_init value from the caller we will copy it
  // here and reference it with q_init_.
  eigenmath::VectorXd q_init_storage_;
};

}  // namespace entity_kinematic_world_details
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_ASPECTS_ENTITY_DEFERRED_IK_PROBLEM_H_
