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

#include "intrinsic/icon/hardware_modules/universal_robots/ur_calibration_utils.h"

#include <array>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "gloop/util/gtl/flat_map.h"
#include "include/ur_calibration/calibration.hpp"
#include "include/ur_client_library/primary/robot_state/kinematics_info.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

using ur_calibration::Calibration;
using ur_calibration::DHRobot;
using ur_calibration::DHSegment;
using ::urcl::primary_interface::KinematicsInfo;

namespace intrinsic::icon {
// A transformation represented as XYZ and roll-pitch-paw (fixed axis) values.

struct XyzRpy {
  eigenmath::Vector3d translation;  // Translation in meters.
  eigenmath::Vector3d rotation;     // Roll-pitch-yaw in radians.
};

namespace {

constexpr std::array<std::string, 6> kJointNames = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

// Note: `base_link` is not included since its pose relative to its parent is
// not related to the kinematics of the robot.
constexpr std::array<std::string, 6> kLinkNames = {
    "shoulder_link", "upper_arm_link", "forearm_link",
    "wrist_1_link",  "wrist_2_link",   "wrist_3_link"};

constexpr auto kUrModelToSolver =
    gtl::fixed_flat_map_of<absl::string_view, absl::string_view>({
        {"UR3", "rt_calibrated_ur3e"},
        {"UR5", "rt_calibrated_ur5e"},
        {"UR10", "rt_calibrated_ur10e"},
        {"UR16", "rt_calibrated_ur16e"},
        {"UR20", "rt_calibrated_ur20"},
    });

absl::StatusOr<std::vector<DHSegment>> GetDefaultDhSegments(
    absl::string_view robot_model) {
  const absl::flat_hash_map<std::string, std::vector<DHSegment>>
      ur_official_dh_params = {{"UR3",
                                {
                                    {0.15185, 0, 0, M_PI / 2},
                                    {0, -0.24355, 0, 0},
                                    {0, -0.2132, 0, 0.0},
                                    {0.13105, 0, 0, M_PI / 2},
                                    {0.08535, 0, 0, -M_PI / 2},
                                    {0.0921, 0, 0, 0},
                                }},
                               {"UR5",
                                {
                                    {0.1625, 0, 0, M_PI / 2},
                                    {0, -0.425, 0, 0},
                                    {0, -0.3922, 0, 0.0},
                                    {0.1333, 0, 0, M_PI / 2},
                                    {0.0997, 0, 0, -M_PI / 2},
                                    {0.0996, 0, 0, 0},
                                }},
                               {"UR10",
                                {
                                    {0.1807, 0, 0, M_PI / 2},
                                    {0, -0.6127, 0, 0},
                                    {0, -0.57155, 0, 0.0},
                                    {0.17415, 0, 0, M_PI / 2},
                                    {0.11985, 0, 0, -M_PI / 2},
                                    {0.11655, 0, 0, 0},
                                }},
                               {"UR16",
                                {
                                    {0.1807, 0, 0, M_PI / 2},
                                    {0, -0.4784, 0, 0},
                                    {0, -0.36, 0, 0.0},
                                    {0.17415, 0, 0, M_PI / 2},
                                    {0.11985, 0, 0, -M_PI / 2},
                                    {0.11655, 0, 0, 0},
                                }},
                               {"UR20",
                                {
                                    {0.2363, 0, 0, M_PI / 2},
                                    {0, -0.862, 0, 0},
                                    {0, -0.7287, 0, 0.0},
                                    {0.201, 0, 0, M_PI / 2},
                                    {0.1593, 0, 0, -M_PI / 2},
                                    {0.1543, 0, 0, 0},
                                }}};
  if (!ur_official_dh_params.contains(robot_model)) {
    return absl::NotFoundError(
        absl::StrCat("No DH parameters found for robot model: ", robot_model));
  }
  return ur_official_dh_params.at(robot_model);
}

absl::StatusOr<std::vector<XyzRpy>> GetDefaultUrIntrinsicParameters(
    absl::string_view robot_model) {
  const absl::flat_hash_map<std::string, std::vector<XyzRpy>>
      ur_official_intrinsic_params = {
          {"UR3",
           {
               {.translation = {0, 0, 0.15185}, .rotation = {0, 0, M_PI}},
               {.translation = {0, 0, 0}, .rotation = {0, 0, 0}},
               {.translation = {0.24355, 0, 0}, .rotation = {0, 0, 0}},
               {.translation = {0.2132, 0, 0}, .rotation = {0, 0, 0}},
               {.translation = {0, 0.13105, -0.08535}, .rotation = {0, 0, 0}},
               {.translation = {0, 0.0921, 0.0},
                .rotation = {-M_PI / 2, M_PI, 0.0}},
           }},
          {"UR5",
           {
               {.translation = {0, 0, 0.1625}, .rotation = {0, 0, M_PI}},
               {.translation = {0, 0, 0}, .rotation = {0, 0, 0}},
               {.translation = {0.425, 0, 0}, .rotation = {0, 0, 0}},
               {.translation = {0.3922, 0, 0}, .rotation = {0, 0, 0}},
               {.translation = {0, 0.1333, -0.0997}, .rotation = {0, 0, 0}},
               {.translation = {0, 0.0996, 0.0},
                .rotation = {-M_PI / 2, M_PI, 0.0}},
           }},
          {"UR10",
           {
               {.translation = {0, 0, 0.1807}, .rotation = {0, 0, M_PI}},
               {.translation = {0, 0, 0}, .rotation = {0, 0, 0}},
               {.translation = {0.6127, 0, 0}, .rotation = {0, 0, 0}},
               {.translation = {0.57155, 0, 0}, .rotation = {0, 0, 0}},
               {.translation = {0, 0.17415, -0.11985}, .rotation = {0, 0, 0}},
               {.translation = {0, 0.11655, 0.0},
                .rotation = {-M_PI / 2, M_PI, 0.0}},
           }},
          {"UR16",
           {{.translation = {0, 0, 0.1807}, .rotation = {0, 0, M_PI}},
            {.translation = {0, 0, 0}, .rotation = {0, 0, 0}},
            {.translation = {0.4784, 0, 0}, .rotation = {0, 0, 0}},
            {.translation = {0.36, 0, 0}, .rotation = {0, 0, 0}},
            {.translation = {0, 0.17415, -0.11985}, .rotation = {0, 0, 0}},
            {.translation = {0, 0.11655, 0.0},
             .rotation = {-M_PI / 2, M_PI, 0.0}}}},
          {"UR20",
           {{.translation = {0, 0, 0.2363}, .rotation = {0, 0, M_PI}},
            {.translation = {0, 0, 0}, .rotation = {0, 0, 0}},
            {.translation = {0.862, 0, 0}, .rotation = {0, 0, 0}},
            {.translation = {0.7287, 0, 0}, .rotation = {0, 0, 0}},
            {.translation = {0, 0.201, -0.1593}, .rotation = {0, 0, 0}},
            {.translation = {0, 0.1543, 0.0},
             .rotation = {-M_PI / 2, M_PI, 0.0}}}}};
  if (!ur_official_intrinsic_params.contains(robot_model)) {
    return absl::NotFoundError(absl::StrCat(
        "No intrinsic parameters found for robot model: ", robot_model));
  }
  return ur_official_intrinsic_params.at(robot_model);
}

// Convert a vector of 4x4 Eigen matrices to eigenmath::AffineTransform3d.
std::vector<eigenmath::AffineTransform3d> toAffine3d(
    const std::vector<Eigen::Matrix4d>& chain) {
  std::vector<eigenmath::AffineTransform3d> affine_chain(chain.size());
  for (int i = 0; i < chain.size(); ++i) {
    affine_chain[i] = chain[i];
  }
  return affine_chain;
}

eigenmath::AffineTransform3d XyzRpyToAffine(const XyzRpy& xyzrpy) {
  eigenmath::Matrix3d rotation;
  rotation =
      eigenmath::AngleAxisd(xyzrpy.rotation[2], eigenmath::Vector3d::UnitZ()) *
      eigenmath::AngleAxisd(xyzrpy.rotation[1], eigenmath::Vector3d::UnitY()) *
      eigenmath::AngleAxisd(xyzrpy.rotation[0], eigenmath::Vector3d::UnitX());
  return Eigen::Translation3d(xyzrpy.translation) * rotation;
}

std::vector<eigenmath::AffineTransform3d> GetChain(
    const std::vector<XyzRpy>& xyzrpy_vec) {
  std::vector<eigenmath::AffineTransform3d> chain;
  chain.reserve(xyzrpy_vec.size());
  for (const auto& xyzrpy : xyzrpy_vec) {
    chain.push_back(XyzRpyToAffine(xyzrpy));
  }

  return chain;
}

std::vector<eigenmath::AffineTransform3d> ComputeCumulativeTransforms(
    const std::vector<eigenmath::AffineTransform3d>& tfs) {
  std::vector<eigenmath::AffineTransform3d> cumulative_tfs(6);
  cumulative_tfs[0] = tfs[0];
  for (int i = 1; i < 6; ++i) {
    cumulative_tfs[i] = cumulative_tfs[i - 1] * tfs[i];
  }
  return cumulative_tfs;
}

std::vector<eigenmath::AffineTransform3d> GetCalibratedChain(
    const std::vector<eigenmath::AffineTransform3d>& chain_A,
    const std::vector<eigenmath::AffineTransform3d>& chain_A_calibrated,
    const std::vector<eigenmath::AffineTransform3d>& chain_B,
    const eigenmath::AffineTransform3d& base_A_to_base_B =
        eigenmath::AffineTransform3d::Identity()) {
  auto chain_B_aligned = chain_B;
  chain_B_aligned[0] = chain_B_aligned[0] * base_A_to_base_B;

  const auto cumulative_chain_A = ComputeCumulativeTransforms(chain_A);
  const auto cumulative_chain_B = ComputeCumulativeTransforms(chain_B_aligned);

  std::vector<eigenmath::AffineTransform3d> R_Bi_Ai(6);
  for (int i = 0; i < 6; ++i) {
    R_Bi_Ai[i] = cumulative_chain_B[i].rotation().inverse() *
                 cumulative_chain_A[i].rotation();
  }

  std::vector<eigenmath::AffineTransform3d> T_Ai_Aip(6);
  for (int i = 0; i < 6; ++i) {
    T_Ai_Aip[i] = chain_A[i].inverse() * chain_A_calibrated[i];
  }

  std::vector<eigenmath::AffineTransform3d> T_Bi_Bip(6);
  for (int i = 0; i < 6; ++i) {
    T_Bi_Bip[i] = R_Bi_Ai[i].inverse() * T_Ai_Aip[i] * R_Bi_Ai[i];
  }

  std::vector<eigenmath::AffineTransform3d> T_Bim1_Bip(6);
  for (int i = 0; i < 6; ++i) {
    T_Bim1_Bip[i] = chain_B[i] * T_Bi_Bip[i];
  }

  return T_Bim1_Bip;
}

}  // namespace

DHRobot DhRobotFromKinematicsInfo(const KinematicsInfo& kin_info) {
  DHRobot robot;

  for (int i = 0; i < 6; ++i) {
    robot.segments_.push_back(DHSegment(kin_info.dh_d_[i], kin_info.dh_a_[i],
                                        kin_info.dh_theta_[i],
                                        kin_info.dh_alpha_[i]));
  }

  return robot;
}

absl::StatusOr<std::vector<EntityAndTransform>> CalculateChain(
    const KinematicsInfo& kin_info, absl::string_view robot_model) {
  DHRobot calibrated_robot_dh = DhRobotFromKinematicsInfo(kin_info);
  INTR_ASSIGN_OR_RETURN(const auto& robot_dh,
                        GetDefaultDhSegments(robot_model));
  DHRobot nominal_robot_dh(robot_dh);
  INTR_ASSIGN_OR_RETURN(auto nominal_ur_xyzrpy_parameters,
                        GetDefaultUrIntrinsicParameters(robot_model));
  const auto nominal_chain_intrinsic_convention =
      GetChain(nominal_ur_xyzrpy_parameters);

  const Calibration cal_A_nominal(nominal_robot_dh);
  const Calibration cal_A_calibrated(calibrated_robot_dh);

  const auto nominal_chain_ur_convention =
      toAffine3d(cal_A_nominal.getSimplified());
  const auto calibrated_chain_ur_convention =
      toAffine3d(cal_A_calibrated.getSimplified());

  const auto calibrated_chain_intrinsic_convention = GetCalibratedChain(
      nominal_chain_ur_convention, calibrated_chain_ur_convention,
      nominal_chain_intrinsic_convention);

  if (calibrated_chain_intrinsic_convention.size() != kJointNames.size()) {
    return absl::InvalidArgumentError(
        "The number of defined joints does not match the length of the chain.");
  }

  std::vector<EntityAndTransform> robot_chain_updates;
  robot_chain_updates.reserve(calibrated_chain_intrinsic_convention.size());
  for (int i = 0; i < calibrated_chain_intrinsic_convention.size(); ++i) {
    robot_chain_updates.push_back(
        {.entity_name = kJointNames[i],
         .parent_t_entity =
             Pose(calibrated_chain_intrinsic_convention[i].matrix()),
         .is_joint = true});
  };
  // Includes the links so that the robot chain is complete.
  for (const auto& link_name : kLinkNames) {
    robot_chain_updates.push_back({.entity_name = link_name,
                                   .parent_t_entity = Pose::Identity(),
                                   .is_joint = false});
  }
  return robot_chain_updates;
}

absl::StatusOr<std::string> GetCalibratedRobotIkSolverKey(
    absl::string_view robot_model) {
  if (!kUrModelToSolver.contains(robot_model)) {
    return absl::NotFoundError(
        absl::StrCat("No solver found for robot model: ", robot_model));
  }
  return std::string(kUrModelToSolver.at(robot_model));
}

std::string GetCalibratedRobotIkSolverTip() { return kLinkNames.back(); }

}  // namespace intrinsic::icon
