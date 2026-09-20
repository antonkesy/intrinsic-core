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

#ifndef INTRINSIC_WORLD_CARTESIAN_KINEMATIC_VIEW_H_
#define INTRINSIC_WORLD_CARTESIAN_KINEMATIC_VIEW_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"

namespace intrinsic {

// The IkSolutions encapsulates the results for an IK solver. It allows
// you to sort and filter the results. The way you can filter results is using
// the FilterPolicy class and you can use the SortingPolicy class to sort
// solutions. The FilterPolicy can come in handy if you want to provide a way to
// exclude solutions that are in collision. You can also use SortingPolicy to
// make sure that the top result is the furthest from a singularity or the
// closes to the previous point or any other algorithm.
class IkSolutions {
 public:
  class FilterPolicy {
   public:
    virtual ~FilterPolicy() = default;

    // Returns true if the dof_values can be used.
    virtual bool IsValid(const eigenmath::VectorXd& dof_values) const = 0;
  };

  virtual ~IkSolutions() = default;

  // Returns one of the IK solutions represented by this object, the solution
  // picked from the set of available solutions is implementation specific. If
  // no solution exists it will return nullopt.
  virtual std::optional<eigenmath::VectorXd> GetSingleSolution() const = 0;

  // Returns an IK solution which is on the same branch as the initialized joint
  // position of this object. It will return nullopt if:
  // (a) a child class does not implement the method, or
  // (b) no solution exists.
  virtual std::optional<eigenmath::VectorXd> GetSameBranchSolution() const {
    return std::nullopt;
  }

  // Returns a set of solutions from the solutions represented by this object.
  // The selection is guided by the given filtering strategy.
  virtual std::vector<eigenmath::VectorXd> GetSampledSolutions(
      size_t max_num_solutions,
      std::optional<const FilterPolicy*> filter) const = 0;
};

// A CartesianKinematicView exposes a cartesian-space interface to a collection
// of DOFs between two objects from the PhysicalWorld aspect. It can then solve
// IK given a desired transform between those two objects. The exact kinematics
// of the underlying system are abstracted from the user.
// For FK, provides access to a DofKinematicView which can be used to set the
// underlying DOFs to concrete values. Afterwards, the updated world state can
// be observed through, e.g., PhysicalWorld::GetTransform.
class CartesianKinematicView {
 public:
  virtual ~CartesianKinematicView() = default;

  // Returns the PhysicalEntityIds associated with this CartesianKinematicView.
  virtual std::pair<PhysicalEntityId, PhysicalEntityId> ObjectsBeingControlled()
      const = 0;

  // Returns the name of the solver used to compute the IK solutions.
  virtual absl::StatusOr<std::string> GetIKSolverKey() const = 0;

  // Returns a set of solutions that would result in the two objects, being
  // controlled by this view, to have the desired transform between them. If
  // there is no error it will always return a valid pointer to a set of
  // solutions. There could be no solutions but the IkSolutions object will be
  // valid and return no solutions when asked.
  virtual std::unique_ptr<IkSolutions> GetIkSolutions(
      const Pose3d& obj1_t_obj2) const = 0;

  // Returns a set of solutions that would result in the two objects, being
  // controlled by this view, to have the desired transform between them. This
  // is similar to the other GetIkSolutions call but takes an extra hint about
  // what joint configuration should be considered. It is useful when chaining
  // motions to consider the previous solution to make the motion smaller.
  virtual std::unique_ptr<IkSolutions> GetIkSolutions(
      const Pose3d& obj1_t_obj2,
      const eigenmath::VectorXd& dof_values) const = 0;

  // Returns the underlying DofKinematicView representing the Dofs being
  // controlled by this CartesianKinematicView.
  virtual std::shared_ptr<const DofKinematicView> GetDofView() const = 0;

  // Returns the underlying DofKinematicView representing the Dofs being
  // controlled by this CartesianKinematicView.
  // For FK, the returned DofKinematicView can be used to set given values and
  // then observe the updated state of the world through, e.g.,
  // PhysicalWorld::GetTransform.
  virtual std::shared_ptr<DofKinematicView> GetDofView() = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_CARTESIAN_KINEMATIC_VIEW_H_
