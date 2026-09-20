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

#include "intrinsic/kinematics/test_utils.h"

#include <gmock/gmock.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "internal/testing.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_matchers.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/link.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace kinematics {
namespace testing {

namespace {

JointLimits CreateTestSkeletonSystemLimits() {
  JointLimits limits;
  CHECK_EQ(limits.SetSize(4), icon::OkStatus());
  limits.max_position.setConstant(2.0);
  limits.min_position.setConstant(-1.5);
  limits.max_velocity.setConstant(2.5);
  limits.max_acceleration.setConstant(25);
  limits.max_jerk.setConstant(200);
  return limits;
}

JointLimits CreateTestSkeltonSoftLimits() {
  JointLimits limits;
  CHECK_EQ(limits.SetSize(4), icon::OkStatus());
  limits.max_position.setConstant(1.8);
  limits.min_position.setConstant(-1.2);
  limits.max_velocity.setConstant(1.5);
  limits.max_acceleration.setConstant(15);
  limits.max_jerk.setConstant(100);
  return limits;
}

constexpr double kEffortLimit = 1.45;
void SetJointLimitsFromControlLimitsForIndex(const JointLimits& control_limits,
                                             int index,
                                             Joint::Limits& joint_limits) {
  CHECK(control_limits.size() > index)
      << "Index provided larger than size of vector.";
  joint_limits.position.lower = control_limits.min_position[index];
  joint_limits.position.upper = control_limits.max_position[index];
  joint_limits.acceleration = control_limits.max_acceleration[index];
  joint_limits.velocity = control_limits.max_velocity[index];
  joint_limits.jerk = control_limits.max_jerk[index];
  joint_limits.effort = kEffortLimit;
}

}  // namespace

void CheckUnorderedElementIdsAreArray(
    const ModelInterface& model, absl::Span<const ElementId> actual_element_ids,
    const std::vector<std::string>& expected_element_names) {
  std::vector<ElementId> expected_element_ids;
  for (const auto& name : expected_element_names) {
    INTRINSIC_RT_ASSERT_OK_AND_ASSIGN(auto id, model.FindElementIdByName(name));
    expected_element_ids.push_back(id);
  }

  EXPECT_THAT(actual_element_ids,
              ::testing::UnorderedElementsAreArray(expected_element_ids))
      << "Actual elements are: "
      << absl::StrJoin(actual_element_ids, ", ",
                       [&](std::string* str, const auto& id) {
                         absl::StrAppend(str, model.GetElementName(id),
                                         " (id=", id.value(), ")");
                       })
      << "\nExpected elements are: "
      << absl::StrJoin(expected_element_ids, ", ",
                       [&](std::string* str, const auto& id) {
                         absl::StrAppend(str, model.GetElementName(id),
                                         " (id=", id.value(), ")");
                       });
}

absl::StatusOr<std::unique_ptr<Skeleton>> CreateSkeleton() {
  INTR_ASSIGN_OR_RETURN(auto test_setup, CreateSkeletonTestSetup());
  return std::move(test_setup->skeleton);
}

absl::StatusOr<std::unique_ptr<TestSetup>> CreateSkeletonTestSetup() {
  const std::string name = "test_skeleton";
  auto tree_setup = std::make_unique<TestSetup>(name);
  tree_setup->reference_values.effort_limits = kEffortLimit;

  // Every link has the length of one; As a result, each parent is one unit
  // translated by the parent.
  Pose3d parent_t_this;
  parent_t_this.translation()[2] = -1;

  // Setup
  tree_setup->reference_values.name = name;
  tree_setup->reference_values.number_dof = 4;
  tree_setup->reference_values.default_configuration =
      std::vector<double>({1.0, 2.1, 0, -1.2});
  tree_setup->reference_values.system_limits = CreateTestSkeletonSystemLimits();
  tree_setup->reference_values.soft_limits = CreateTestSkeltonSoftLimits();

  INTR_ASSIGN_OR_RETURN(auto l1_id, tree_setup->skeleton->CreateLink(
                                        "link1", Link::Parameters()));
  tree_setup->reference_values.link_ids.push_back(l1_id);
  tree_setup->reference_values.element_ids.push_back(l1_id);
  tree_setup->reference_values.base_id = l1_id;

  Joint::Parameters j1_param;
  j1_param.type = Joint::REVOLUTE;
  j1_param.default_configuration =
      tree_setup->reference_values.default_configuration[0];
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.system_limits, 0, j1_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.soft_limits, 0, j1_param.soft_limits);
  INTR_ASSIGN_OR_RETURN(
      auto j1_id, tree_setup->skeleton->CreateJoint("joint1", j1_param, l1_id,
                                                    parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j1_id);
  tree_setup->reference_values.element_ids.push_back(j1_id);
  tree_setup->reference_values.dof_ids.push_back(j1_id);

  Pose3d pose_l2(Eigen::Quaterniond(
      Eigen::AngleAxis<double>(-M_PI_2, eigenmath::Vector3d::UnitY())));
  INTR_ASSIGN_OR_RETURN(
      auto l2_id, tree_setup->skeleton->CreateLink("link2", Link::Parameters(),
                                                   j1_id, pose_l2));
  tree_setup->reference_values.link_ids.push_back(l2_id);
  tree_setup->reference_values.element_ids.push_back(l2_id);

  Pose3d pose_l7(Eigen::Quaterniond(
      Eigen::AngleAxis<double>(M_PI_2, eigenmath::Vector3d::UnitY())));
  INTR_ASSIGN_OR_RETURN(
      auto l7_id, tree_setup->skeleton->CreateLink("link7", Link::Parameters(),
                                                   j1_id, pose_l7));
  tree_setup->reference_values.link_ids.push_back(l7_id);
  tree_setup->reference_values.element_ids.push_back(l7_id);
  tree_setup->reference_values.test_link = std::make_pair(l7_id, "link7");

  Joint::Parameters j2_param;
  j2_param.type = Joint::REVOLUTE;
  j2_param.default_configuration =
      tree_setup->reference_values.default_configuration[1];
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.system_limits, 1, j2_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.soft_limits, 1, j2_param.soft_limits);
  INTR_ASSIGN_OR_RETURN(
      auto j2_id, tree_setup->skeleton->CreateJoint("joint2", j2_param, l2_id,
                                                    parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j2_id);
  tree_setup->reference_values.element_ids.push_back(j2_id);
  tree_setup->reference_values.dof_ids.push_back(j2_id);
  tree_setup->reference_values.test_joint = std::make_pair(j2_id, "joint2");

  Joint::Parameters j8_param;
  j8_param.type = Joint::FIXED;
  INTR_ASSIGN_OR_RETURN(
      auto j8_id, tree_setup->skeleton->CreateJoint("joint8", j8_param, l7_id,
                                                    parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j8_id);
  tree_setup->reference_values.element_ids.push_back(j8_id);

  INTR_ASSIGN_OR_RETURN(auto l4_id, tree_setup->skeleton->CreateLink(
                                        "link4", Link::Parameters(), j2_id));
  tree_setup->reference_values.link_ids.push_back(l4_id);
  tree_setup->reference_values.element_ids.push_back(l4_id);
  INTR_ASSIGN_OR_RETURN(auto l5_id, tree_setup->skeleton->CreateLink(
                                        "link5", Link::Parameters(), j2_id));
  tree_setup->reference_values.link_ids.push_back(l5_id);
  tree_setup->reference_values.element_ids.push_back(l5_id);
  INTR_ASSIGN_OR_RETURN(auto l8_id, tree_setup->skeleton->CreateLink(
                                        "link8", Link::Parameters(), j8_id));
  tree_setup->reference_values.link_ids.push_back(l8_id);
  tree_setup->reference_values.element_ids.push_back(l8_id);

  Joint::Parameters j6_param;
  j6_param.type = Joint::FIXED;
  INTR_ASSIGN_OR_RETURN(
      auto j6_id, tree_setup->skeleton->CreateJoint("joint6", j6_param, l5_id));
  tree_setup->reference_values.joint_ids.push_back(j6_id);
  tree_setup->reference_values.element_ids.push_back(j6_id);

  Joint::Parameters j5_param;
  j5_param.type = Joint::FIXED;
  INTR_ASSIGN_OR_RETURN(auto j5, Joint::Create("joint5", j5_param));
  INTR_ASSIGN_OR_RETURN(auto j5_id, tree_setup->skeleton->AddJoint(
                                        std::move(j5), l4_id, parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j5_id);
  tree_setup->reference_values.element_ids.push_back(j5_id);

  Joint::Parameters j9_param;
  j9_param.type = Joint::REVOLUTE;
  j9_param.default_configuration =
      tree_setup->reference_values.default_configuration[3];
  j9_param.axis = eigenmath::Vector3d::UnitY();
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.system_limits, 3, j9_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.soft_limits, 3, j9_param.soft_limits);
  INTR_ASSIGN_OR_RETURN(auto j9, Joint::Create("joint9", j9_param));
  INTR_ASSIGN_OR_RETURN(auto j9_id, tree_setup->skeleton->AddJoint(
                                        std::move(j9), l8_id, parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j9_id);
  tree_setup->reference_values.element_ids.push_back(j9_id);
  tree_setup->reference_values.dof_ids.push_back(j9_id);

  INTR_ASSIGN_OR_RETURN(
      auto cf1_id, tree_setup->skeleton->CreateCoordinateFrame("cf1", j5_id));
  tree_setup->reference_values.element_ids.push_back(cf1_id);
  tree_setup->reference_values.tip_ids.push_back(cf1_id);
  tree_setup->reference_values.test_element.first = cf1_id;
  tree_setup->reference_values.test_element.second = "cf1";

  Joint::Parameters j7_param;
  j7_param.type = Joint::REVOLUTE;
  j7_param.default_configuration =
      tree_setup->reference_values.default_configuration[2];
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.system_limits, 2, j7_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.soft_limits, 2, j7_param.soft_limits);
  INTR_ASSIGN_OR_RETURN(
      auto j7_id, tree_setup->skeleton->CreateJoint("joint7", j7_param, j6_id));
  tree_setup->reference_values.joint_ids.push_back(j7_id);
  tree_setup->reference_values.element_ids.push_back(j7_id);
  tree_setup->reference_values.dof_ids.push_back(j7_id);
  tree_setup->reference_values.tip_ids.push_back(j7_id);
  CHECK_EQ(tree_setup->reference_values.dof_ids.size(),
           tree_setup->reference_values.number_dof);

  INTR_ASSIGN_OR_RETURN(
      auto cf2_id, tree_setup->skeleton->CreateCoordinateFrame("cf2", j9_id));
  tree_setup->reference_values.element_ids.push_back(cf2_id);
  tree_setup->reference_values.tip_ids.push_back(cf2_id);

  INTR_RETURN_IF_ERROR(tree_setup->skeleton->SetSolverKey(l1_id, j7_id, "ur"));
  return tree_setup;
}

absl::StatusOr<std::unique_ptr<Skeleton>> CreateChainSkeleton() {
  INTR_ASSIGN_OR_RETURN(auto test_setup, CreateSkeletonChainSetup());
  return std::move(test_setup->skeleton);
}

absl::StatusOr<std::unique_ptr<TestSetup>> CreateSkeletonChainSetup(
    bool add_2nd_frame) {
  const std::string name = "Test Chain";
  auto chain_setup = std::make_unique<TestSetup>(name);
  chain_setup->reference_values.effort_limits = kEffortLimit;

  // Set the default system and soft limits
  chain_setup->reference_values.system_limits =
      CreateTestSkeletonSystemLimits();
  chain_setup->reference_values.soft_limits = CreateTestSkeltonSoftLimits();

  // Every link has the length of one; As a result, each parent is one unit
  // translated by the parent.
  Pose3d parent_t_this;
  parent_t_this.translation()[2] = -1;

  // Setup
  chain_setup->reference_values.name = name;
  chain_setup->reference_values.number_dof = 3;

  INTR_ASSIGN_OR_RETURN(auto l1_id, chain_setup->skeleton->CreateLink(
                                        "link1", Link::Parameters()));
  chain_setup->reference_values.link_ids.push_back(l1_id);
  chain_setup->reference_values.element_ids.push_back(l1_id);
  chain_setup->reference_values.base_id = l1_id;

  Joint::Parameters j1_param;
  j1_param.type = Joint::REVOLUTE;
  SetJointLimitsFromControlLimitsForIndex(
      chain_setup->reference_values.system_limits, 0, j1_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      chain_setup->reference_values.soft_limits, 0, j1_param.soft_limits);

  INTR_ASSIGN_OR_RETURN(
      auto j1_id, chain_setup->skeleton->CreateJoint("joint1", j1_param, l1_id,
                                                     parent_t_this));
  chain_setup->reference_values.joint_ids.push_back(j1_id);
  chain_setup->reference_values.element_ids.push_back(j1_id);
  chain_setup->reference_values.dof_ids.push_back(j1_id);

  INTR_ASSIGN_OR_RETURN(auto l2_id, chain_setup->skeleton->CreateLink(
                                        "link2", Link::Parameters(), j1_id));
  chain_setup->reference_values.link_ids.push_back(l2_id);
  chain_setup->reference_values.element_ids.push_back(l2_id);

  Joint::Parameters j2_param;
  j2_param.type = Joint::REVOLUTE;
  j2_param.axis = eigenmath::Vector3d::UnitY();
  SetJointLimitsFromControlLimitsForIndex(
      chain_setup->reference_values.system_limits, 1, j2_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      chain_setup->reference_values.soft_limits, 1, j2_param.soft_limits);
  INTR_ASSIGN_OR_RETURN(auto j2_id, chain_setup->skeleton->CreateJoint(
                                        "joint2", j2_param, l2_id));
  chain_setup->reference_values.joint_ids.push_back(j2_id);
  chain_setup->reference_values.element_ids.push_back(j2_id);
  chain_setup->reference_values.dof_ids.push_back(j2_id);

  INTR_ASSIGN_OR_RETURN(auto l3_id, chain_setup->skeleton->CreateLink(
                                        "link3", Link::Parameters(), j2_id));
  chain_setup->reference_values.link_ids.push_back(l3_id);
  chain_setup->reference_values.element_ids.push_back(l3_id);

  Joint::Parameters j8_param;
  j8_param.type = Joint::FIXED;
  INTR_ASSIGN_OR_RETURN(
      auto j8_id, chain_setup->skeleton->CreateJoint("joint8", j8_param, l3_id,
                                                     parent_t_this));
  chain_setup->reference_values.joint_ids.push_back(j8_id);
  chain_setup->reference_values.element_ids.push_back(j8_id);

  INTR_ASSIGN_OR_RETURN(auto l8_id, chain_setup->skeleton->CreateLink(
                                        "link8", Link::Parameters(), j8_id));
  chain_setup->reference_values.link_ids.push_back(l8_id);
  chain_setup->reference_values.element_ids.push_back(l8_id);

  Joint::Parameters j9_param;
  j9_param.type = Joint::REVOLUTE;
  INTR_ASSIGN_OR_RETURN(auto j9, Joint::Create("joint9", j9_param));
  INTR_ASSIGN_OR_RETURN(auto j9_id, chain_setup->skeleton->AddJoint(
                                        std::move(j9), l8_id, parent_t_this));
  chain_setup->reference_values.joint_ids.push_back(j9_id);
  chain_setup->reference_values.element_ids.push_back(j9_id);
  chain_setup->reference_values.dof_ids.push_back(j9_id);

  INTR_ASSIGN_OR_RETURN(
      auto cf2_id, chain_setup->skeleton->CreateCoordinateFrame("cf2", j9_id));
  chain_setup->reference_values.element_ids.push_back(cf2_id);
  chain_setup->reference_values.tip_ids.push_back(cf2_id);

  if (add_2nd_frame) {
    INTR_ASSIGN_OR_RETURN(
        auto cf1_id,
        chain_setup->skeleton->CreateCoordinateFrame("cf1", l8_id));
    chain_setup->reference_values.element_ids.push_back(cf1_id);
    chain_setup->reference_values.tip_ids.push_back(cf1_id);
  }

  return chain_setup;
}

absl::StatusOr<std::unique_ptr<Skeleton>> CreateSerialChainSkeleton(
    int ndof, std::optional<JointLimits> system_limits) {
  INTR_ASSIGN_OR_RETURN(auto test_setup, CreateSerialChainSkeletonTestSetup(
                                             ndof, std::move(system_limits)));
  return std::move(test_setup->skeleton);
}

absl::StatusOr<std::unique_ptr<TestSetup>> CreateSerialChainSkeletonTestSetup(
    int ndof, std::optional<JointLimits> system_limits) {
  // Validate limits, if any
  if (system_limits.has_value() && system_limits->size() != ndof) {
    return absl::InvalidArgumentError(
        absl::StrCat("Soft limits have ", system_limits->size(),
                     " DoFs, but Skeleton only has ", ndof,
                     " DoFs. Make sure `ndof` and the size of your custom "
                     "limits are the same!"));
  }
  if (!system_limits.has_value()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(system_limits, JointLimits::Unlimited(ndof));
  }
  const std::string name = "Variable DoF Serial Test Chain";
  auto chain_setup = std::make_unique<TestSetup>(name);
  chain_setup->reference_values.effort_limits = kEffortLimit;
  chain_setup->reference_values.system_limits = *system_limits;
  chain_setup->reference_values.soft_limits = *system_limits;
  // Every link has the length of one; As a result, each parent is one unit
  // translated by the parent.
  Pose3d parent_t_this;
  parent_t_this.translation()[2] = 1.0;

  chain_setup->reference_values.name = name;
  chain_setup->reference_values.number_dof = ndof;

  ElementId link_parent_id = kInvalidElementId;

  for (int i = 0; i < ndof; i++) {
    INTR_ASSIGN_OR_RETURN(
        auto link_id,
        chain_setup->skeleton->CreateLink(absl::StrCat("link", i),
                                          Link::Parameters(), link_parent_id));
    chain_setup->reference_values.link_ids.push_back(link_id);
    chain_setup->reference_values.element_ids.push_back(link_id);
    chain_setup->reference_values.base_id = link_id;

    Joint::Parameters joint_param;
    joint_param.type = Joint::REVOLUTE;
    joint_param.axis = eigenmath::Vector3d::UnitY();

    joint_param.system_limits.position = {
        .lower = system_limits->min_position(i),
        .upper = system_limits->max_position(i)};
    joint_param.system_limits.velocity = system_limits->max_velocity(i);
    joint_param.system_limits.acceleration = system_limits->max_acceleration(i);
    joint_param.system_limits.effort = system_limits->max_torque(i);

    joint_param.soft_limits = joint_param.system_limits;

    INTR_ASSIGN_OR_RETURN(
        auto joint_id,
        chain_setup->skeleton->CreateJoint(
            absl::StrCat("joint", i), joint_param, link_id, parent_t_this));
    chain_setup->reference_values.joint_ids.push_back(joint_id);
    chain_setup->reference_values.element_ids.push_back(joint_id);
    chain_setup->reference_values.dof_ids.push_back(joint_id);

    link_parent_id = joint_id;
  }

  INTR_ASSIGN_OR_RETURN(
      auto last_link_id,
      chain_setup->skeleton->CreateLink(absl::StrCat("link", ndof),
                                        Link::Parameters(), link_parent_id));
  chain_setup->reference_values.link_ids.push_back(last_link_id);
  chain_setup->reference_values.element_ids.push_back(last_link_id);

  // Add fixed joint.
  Joint::Parameters fixed_joint_param;
  fixed_joint_param.type = Joint::FIXED;
  INTR_ASSIGN_OR_RETURN(
      auto fixed_joint_id,
      chain_setup->skeleton->CreateJoint("fixed_joint", fixed_joint_param,
                                         last_link_id, parent_t_this));
  chain_setup->reference_values.joint_ids.push_back(fixed_joint_id);
  chain_setup->reference_values.element_ids.push_back(fixed_joint_id);

  INTR_ASSIGN_OR_RETURN(auto tool_link_id,
                        chain_setup->skeleton->CreateLink(
                            "tool_link", Link::Parameters(), fixed_joint_id));
  chain_setup->reference_values.link_ids.push_back(tool_link_id);
  chain_setup->reference_values.element_ids.push_back(tool_link_id);

  return chain_setup;
}

absl::StatusOr<std::unique_ptr<Skeleton>> CreateSkeletonWithDependentJoints() {
  INTR_ASSIGN_OR_RETURN(auto test_setup,
                        CreateSkeletonTestSetupWithDependentJoints());
  return std::move(test_setup->skeleton);
}

absl::StatusOr<std::unique_ptr<TestSetup>>
CreateSkeletonTestSetupWithDependentJoints(
    std::optional<JointLimits> system_limits) {
  const std::string name = "test_skeleton";
  auto tree_setup = std::make_unique<TestSetup>(name);
  tree_setup->reference_values.effort_limits = kEffortLimit;

  // Every link has the length of one; As a result, each parent is one unit
  // translated by the parent.
  Pose3d parent_t_this;
  parent_t_this.translation()[2] = -1;

  // Setup
  tree_setup->reference_values.name = name;
  tree_setup->reference_values.number_dof = 4;
  tree_setup->reference_values.default_configuration =
      std::vector<double>({1.0, 2.1, 0, -1.2});
  tree_setup->reference_values.system_limits = CreateTestSkeletonSystemLimits();
  tree_setup->reference_values.soft_limits = CreateTestSkeltonSoftLimits();

  INTR_ASSIGN_OR_RETURN(auto l1_id, tree_setup->skeleton->CreateLink(
                                        "link1", Link::Parameters()));
  tree_setup->reference_values.link_ids.push_back(l1_id);
  tree_setup->reference_values.element_ids.push_back(l1_id);
  tree_setup->reference_values.base_id = l1_id;

  Joint::Parameters j1_param;
  j1_param.type = Joint::REVOLUTE;
  j1_param.default_configuration =
      tree_setup->reference_values.default_configuration[0];
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.system_limits, 0, j1_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.soft_limits, 0, j1_param.soft_limits);
  INTR_ASSIGN_OR_RETURN(
      auto j1_id, tree_setup->skeleton->CreateJoint("joint1", j1_param, l1_id,
                                                    parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j1_id);
  tree_setup->reference_values.element_ids.push_back(j1_id);
  tree_setup->reference_values.dof_ids.push_back(j1_id);

  Pose3d pose_l2(Eigen::Quaterniond(
      Eigen::AngleAxis<double>(-M_PI_2, eigenmath::Vector3d::UnitY())));
  INTR_ASSIGN_OR_RETURN(
      auto l2_id, tree_setup->skeleton->CreateLink("link2", Link::Parameters(),
                                                   j1_id, pose_l2));
  tree_setup->reference_values.link_ids.push_back(l2_id);
  tree_setup->reference_values.element_ids.push_back(l2_id);

  Pose3d pose_l7(Eigen::Quaterniond(
      Eigen::AngleAxis<double>(M_PI_2, eigenmath::Vector3d::UnitY())));
  INTR_ASSIGN_OR_RETURN(
      auto l7_id, tree_setup->skeleton->CreateLink("link7", Link::Parameters(),
                                                   j1_id, pose_l7));
  tree_setup->reference_values.link_ids.push_back(l7_id);
  tree_setup->reference_values.element_ids.push_back(l7_id);
  tree_setup->reference_values.test_link = std::make_pair(l7_id, "link7");

  Joint::Parameters j2_param;
  j2_param.type = Joint::REVOLUTE;
  j2_param.default_configuration =
      tree_setup->reference_values.default_configuration[1];
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.system_limits, 1, j2_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.soft_limits, 1, j2_param.soft_limits);
  INTR_ASSIGN_OR_RETURN(
      auto j2_id, tree_setup->skeleton->CreateJoint("joint2", j2_param, l2_id,
                                                    parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j2_id);
  tree_setup->reference_values.element_ids.push_back(j2_id);
  tree_setup->reference_values.dof_ids.push_back(j2_id);
  tree_setup->reference_values.test_joint = std::make_pair(j2_id, "joint2");

  Joint::Parameters j8_param;
  j8_param.type = Joint::FIXED;
  INTR_ASSIGN_OR_RETURN(
      auto j8_id, tree_setup->skeleton->CreateJoint("joint8", j8_param, l7_id,
                                                    parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j8_id);
  tree_setup->reference_values.element_ids.push_back(j8_id);

  INTR_ASSIGN_OR_RETURN(auto l4_id, tree_setup->skeleton->CreateLink(
                                        "link4", Link::Parameters(), j2_id));
  tree_setup->reference_values.link_ids.push_back(l4_id);
  tree_setup->reference_values.element_ids.push_back(l4_id);
  INTR_ASSIGN_OR_RETURN(auto l5_id, tree_setup->skeleton->CreateLink(
                                        "link5", Link::Parameters(), j2_id));
  tree_setup->reference_values.link_ids.push_back(l5_id);
  tree_setup->reference_values.element_ids.push_back(l5_id);
  INTR_ASSIGN_OR_RETURN(auto l8_id, tree_setup->skeleton->CreateLink(
                                        "link8", Link::Parameters(), j8_id));
  tree_setup->reference_values.link_ids.push_back(l8_id);
  tree_setup->reference_values.element_ids.push_back(l8_id);

  Joint::Parameters j6_param;
  j6_param.type = Joint::FIXED;
  INTR_ASSIGN_OR_RETURN(
      auto j6_id, tree_setup->skeleton->CreateJoint("joint6", j6_param, l5_id));
  tree_setup->reference_values.joint_ids.push_back(j6_id);
  tree_setup->reference_values.element_ids.push_back(j6_id);

  Joint::Parameters j5_param;
  j5_param.type = Joint::FIXED;
  INTR_ASSIGN_OR_RETURN(auto j5, Joint::Create("joint5", j5_param));
  INTR_ASSIGN_OR_RETURN(auto j5_id, tree_setup->skeleton->AddJoint(
                                        std::move(j5), l4_id, parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j5_id);
  tree_setup->reference_values.element_ids.push_back(j5_id);

  Joint::Parameters j9_param;
  j9_param.type = Joint::REVOLUTE;
  j9_param.default_configuration =
      tree_setup->reference_values.default_configuration[3];
  j9_param.axis = eigenmath::Vector3d::UnitY();
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.system_limits, 3, j9_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.soft_limits, 3, j9_param.soft_limits);

  // Linear dependency for joint 9.
  Joint::LinearDependency j9_linear_dependency;
  j9_linear_dependency.alpha_self = .5;
  j9_linear_dependency.alpha_leading = {
      {/*leading_joint_id=*/j1_id, /*alpha=*/1.0}};
  INTR_ASSIGN_OR_RETURN(
      auto j9, Joint::Create("joint9", j9_param, j9_linear_dependency));
  INTR_ASSIGN_OR_RETURN(auto j9_id, tree_setup->skeleton->AddJoint(
                                        std::move(j9), l8_id, parent_t_this));
  tree_setup->reference_values.joint_ids.push_back(j9_id);
  tree_setup->reference_values.element_ids.push_back(j9_id);
  tree_setup->reference_values.dof_ids.push_back(j9_id);

  INTR_ASSIGN_OR_RETURN(
      auto cf1_id, tree_setup->skeleton->CreateCoordinateFrame("cf1", j5_id));
  tree_setup->reference_values.element_ids.push_back(cf1_id);
  tree_setup->reference_values.tip_ids.push_back(cf1_id);
  tree_setup->reference_values.test_element.first = cf1_id;
  tree_setup->reference_values.test_element.second = "cf1";

  Joint::Parameters j7_param;
  j7_param.type = Joint::REVOLUTE;
  j7_param.default_configuration =
      tree_setup->reference_values.default_configuration[2];
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.system_limits, 2, j7_param.system_limits);
  SetJointLimitsFromControlLimitsForIndex(
      tree_setup->reference_values.soft_limits, 2, j7_param.soft_limits);

  // Linear dependency for joint 7.
  Joint::LinearDependency j7_linear_dependency;
  j7_linear_dependency.alpha_self = 1.0;
  j7_linear_dependency.alpha_leading = {
      {/*leading_joint_id=*/j2_id, /*alpha=*/-1.0}};
  INTR_ASSIGN_OR_RETURN(auto j7_id,
                        tree_setup->skeleton->CreateJoint(
                            "joint7", j7_param, j7_linear_dependency, j6_id));
  tree_setup->reference_values.joint_ids.push_back(j7_id);
  tree_setup->reference_values.element_ids.push_back(j7_id);
  tree_setup->reference_values.dof_ids.push_back(j7_id);
  tree_setup->reference_values.tip_ids.push_back(j7_id);
  CHECK_EQ(tree_setup->reference_values.dof_ids.size(),
           tree_setup->reference_values.number_dof);

  INTR_ASSIGN_OR_RETURN(
      auto cf2_id, tree_setup->skeleton->CreateCoordinateFrame("cf2", j9_id));
  tree_setup->reference_values.element_ids.push_back(cf2_id);
  tree_setup->reference_values.tip_ids.push_back(cf2_id);

  INTR_RETURN_IF_ERROR(tree_setup->skeleton->SetSolverKey(l1_id, j7_id, "ur"));
  return tree_setup;
}

}  // namespace testing
}  // namespace kinematics
}  // namespace intrinsic
