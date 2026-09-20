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

#ifndef INTRINSIC_KINEMATICS_TEST_UTILS_H_
#define INTRINSIC_KINEMATICS_TEST_UTILS_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/types/joint_limits.h"

namespace intrinsic {
namespace kinematics {
namespace testing {

// Compare the actual element id array to the an expected array defined by
// names.
void CheckUnorderedElementIdsAreArray(
    const ModelInterface& model, absl::Span<const ElementId> actual_element_ids,
    const std::vector<std::string>& expected_element_names);

struct TestSetup {
  struct Parameters {
    // model name
    std::string name;

    // elements
    int number_dof;

    std::vector<ElementId> joint_ids;
    std::vector<ElementId> link_ids;
    std::vector<ElementId> element_ids;
    std::vector<ElementId> dof_ids;
    std::vector<ElementId> tip_ids;
    ElementId base_id;

    std::pair<ElementId, std::string> test_element;
    std::pair<ElementId, std::string> test_joint;
    std::pair<ElementId, std::string> test_link;

    // limits
    JointLimits soft_limits;
    JointLimits system_limits;
    double effort_limits;

    // default configuration of the dofs
    std::vector<double> default_configuration;
  };
  Parameters reference_values;
  std::unique_ptr<Skeleton> skeleton;

  explicit TestSetup(const std::string& name)
      : skeleton(std::make_unique<Skeleton>(name)) {}

  TestSetup(const TestSetup& other)
      : reference_values(other.reference_values),
        skeleton(other.skeleton->Clone()) {}
};

// Set up links and joints to take the form
//                 l1
//                 |
//                 j1
//                /  \
//               l2   l7
//              /      \
//             j2       j8(f)
//            /  \       |
//           l4   l5     l8
//          /      |      |
//         j5(f)   j6(f)  j9
//          |      |      |
//          cf1    j7     cf2
absl::StatusOr<std::unique_ptr<Skeleton>> CreateSkeleton();
absl::StatusOr<std::unique_ptr<TestSetup>> CreateSkeletonTestSetup();

// Set up links and joints to take the form
//                 l1
//                 |
//                 j1
//                 |
//                 l2
//                 |
//                 j2
//                 |
//                 l3
//                 |
//                 j8(f)
//                 |
//                 l8-cf1 (optional: add_2nd_frame)
//                 |
//                 j9
//                 |
//                 cf2
absl::StatusOr<std::unique_ptr<Skeleton>> CreateChainSkeleton();
absl::StatusOr<std::unique_ptr<TestSetup>> CreateSkeletonChainSetup(
    bool add_2nd_frame = false);

// Create a test skeleton (test setup respectively) which holds a variable
// number of revolute joints plus a fixed joint at the tip. Set up links and
// joints to take the form
//                 l0
//                 |
//                 l1
//                 |
//                ...
//                 |
//               l_ndof
//                 |
//             fixed_joint
//                 |
//             tool_link
//
// A caller can optionally provide `system_limits`. If `system_limits` are
// nullopt, the Skeleton has infinite limits. All joints automatically inherit
// soft position limits based on the system limits. That is, all joints have a
// soft upper position limit of min(system_limits.max_position), and a soft
// lower position limit of max(system_limits.min_position).
absl::StatusOr<std::unique_ptr<Skeleton>> CreateSerialChainSkeleton(
    int ndof, std::optional<JointLimits> system_limits = std::nullopt);
absl::StatusOr<std::unique_ptr<TestSetup>> CreateSerialChainSkeletonTestSetup(
    int ndof, std::optional<JointLimits> system_limits = std::nullopt);

// Set up links and joints, including dependent joints, to take the form
//                 l1
//                 |
//                 j1
//                /  \
//               l2   l7
//              /      \
//             j2       j8(f)
//            /  \       |
//           l4   l5     l8
//          /      |      |
//         j5(f)   j6(f)  j9(d)
//          |      |      |
//          cf1    j7(d)  cf2
//
// Dependent joints are marked with (d) in the diagram, therefore
// - j7 is a dependent joint coupled to j2.
// - j9 is a dependent joint coupled to j1.
absl::StatusOr<std::unique_ptr<Skeleton>> CreateSkeletonWithDependentJoints();
absl::StatusOr<std::unique_ptr<TestSetup>>
CreateSkeletonTestSetupWithDependentJoints(
    std::optional<JointLimits> system_limits = std::nullopt);

}  // namespace testing
}  // namespace kinematics
}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_TEST_UTILS_H_
