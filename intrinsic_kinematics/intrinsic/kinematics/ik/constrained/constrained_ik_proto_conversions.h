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

#ifndef INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINED_IK_PROTO_CONVERSIONS_H_
#define INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINED_IK_PROTO_CONVERSIONS_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/constrained/constrained_ik.pb.h"
#include "intrinsic/kinematics/ik/constrained/constraints.h"
#include "intrinsic/kinematics/ik/constrained/cost_functions.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kinematics {

// Converts joint limits constraint in proto format to JointStateP. Returns
// 'kInternalError' in case the underlying copyJoints method fails.
absl::Status FromProto(
    const intrinsic_proto::kinematics::JointPositionLimitsConstraint& proto,
    JointStateP& min_position, JointStateP& max_position);

// Converts PointConstraint proto into a PointConstraint instance.
// Returns 'kFailedPrecondition' error if a requested element does not exist in
// the 'chain' or if the fields in the 'proto' do not comply with constraint
// requirements.
absl::StatusOr<std::unique_ptr<PointConstraint>> FromProto(
    const intrinsic_proto::kinematics::PointConstraint& proto,
    const Chain* chain);

// Converts PoseConstraint proto into a PoseConstraint instance.
// Returns 'kFailedPrecondition' error if a requested element does not exist in
// the 'chain' or if the fields in the 'proto' do not comply with constraint
// requirements.
absl::StatusOr<std::unique_ptr<PoseConstraint>> FromProto(
    const intrinsic_proto::kinematics::PoseConstraint& proto,
    const Chain* chain);

// Converts EllipsoidConstraint proto into an EllipsoidConstraint instance.
// Returns 'kFailedPrecondition' error if a requested element does not exist in
// the 'chain' or if the fields in the 'proto' do not comply with constraint
// requirements.
absl::StatusOr<std::unique_ptr<EllipsoidConstraint>> FromProto(
    const intrinsic_proto::kinematics::PositionEllipsoidConstraint& proto,
    const Chain* chain);

// Converts PlaneConstraint proto into a PlaneConstraint instance.
// Returns 'kFailedPrecondition' error if a requested element does not exist in
// the 'chain' or if the fields in the 'proto' do not comply with constraint
// requirements.
absl::StatusOr<std::unique_ptr<PlaneConstraint>> FromProto(
    const intrinsic_proto::kinematics::PlaneConstraint& proto,
    const Chain* chain);

// Convert OrientationWithFreeAxis constraint proto into an
// OrientationWithFreeAxisConstraint instance.
// Returns 'kFailedPrecondition' error if a requested element does not exist in
// the 'chain' or if the fields in the 'proto' do not comply with constraint
// requirements.
absl::StatusOr<std::unique_ptr<OrientationWithFreeAxisConstraint>> FromProto(
    const intrinsic_proto::kinematics::OrientationWithFreeAxisConstraint& proto,
    const Chain* chain);

// Converts OrientationConeConstraint proto into an OrientationConeConstraint
// instance.
// Returns 'kFailedPrecondition' error if a requested element does not exist in
// the 'chain' or if the fields in the 'proto' do not comply with constraint
// requirements.
absl::StatusOr<std::unique_ptr<OrientationConeConstraint>> FromProto(
    const intrinsic_proto::kinematics::OrientationConeConstraint& proto,
    const Chain* chain);

// Converts JointPositionCost proto into a JointPositionCost instance.
// Returns 'kFailedPrecondition' error if a requested element does not exist in
// the 'chain' or if the fields in the 'proto' do not comply with constraint
// requirements.
absl::StatusOr<std::unique_ptr<JointPositionCost>> FromProto(
    const intrinsic_proto::kinematics::JointPositionCost& proto,
    const Chain* chain);

// Converts ManipulabilityCost proto into a ManipulabilityCost instance.
// Returns 'kFailedPrecondition' error if a requested element does not exist in
// the 'chain' or if the fields in the 'proto' do not comply with constraint
// requirements.
absl::StatusOr<std::unique_ptr<ManipulabilityCost>> FromProto(
    const intrinsic_proto::kinematics::ManipulabilityCost& proto,
    const Chain* chain);

// Convenience helpers to transform ElementIds into a proto::RobotFrame and vice
// versa.
intrinsic_proto::kinematics::RobotFrame ToRobotFrame(ElementId element_id);
ElementId FromRobotFrame(
    const intrinsic_proto::kinematics::RobotFrame& robot_frame);

}  // namespace intrinsic::kinematics

#endif  // INTRINSIC_KINEMATICS_IK_CONSTRAINED_CONSTRAINED_IK_PROTO_CONVERSIONS_H_
