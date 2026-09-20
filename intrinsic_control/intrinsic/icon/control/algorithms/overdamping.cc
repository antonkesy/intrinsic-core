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

#include "intrinsic/icon/control/algorithms/overdamping.h"

#include <cmath>
#include <limits>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/compute_critical_damping.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

icon::RealtimeStatusOr<double> ComputeOverdampingConstantFromStiffnessScalars(
    const OverdampingFromStiffnessScalarsInputParameters& input_parameters) {
  // Substitutes std::numeric_limits<double>::infinity() entries with
  // std::numeric_limits<double>::max():
  const double preprocessed_stiffness_robot_joints =
      (input_parameters.stiffness_robot_joints >
       std::numeric_limits<double>::max())
          ? std::numeric_limits<double>::max()
          : input_parameters.stiffness_robot_joints;
  const double preprocessed_stiffness_compliance_device =
      (input_parameters.stiffness_compliance_device >
       std::numeric_limits<double>::max())
          ? std::numeric_limits<double>::max()
          : input_parameters.stiffness_compliance_device;
  const double preprocessed_stiffness_environment =
      (input_parameters.stiffness_environment >
       std::numeric_limits<double>::max())
          ? std::numeric_limits<double>::max()
          : input_parameters.stiffness_environment;
  const double preprocessed_desired_virtual_stiffness_contact =
      (input_parameters.desired_virtual_stiffness_contact >
       std::numeric_limits<double>::max())
          ? std::numeric_limits<double>::max()
          : input_parameters.desired_virtual_stiffness_contact;

  // Verifies that each stiffness term is greater or equal `kMinimumStiffness`:
  if (preprocessed_stiffness_robot_joints < kMinimumStiffness) {
    return icon::InvalidArgumentError(
        FixedStrCat<RealtimeStatus::kMaxMessageLength>(
            "stiffness_robot_joints must be >= ", kMinimumStiffness,
            ", but got ", preprocessed_stiffness_robot_joints, " instead."));
  }
  if (preprocessed_stiffness_compliance_device < kMinimumStiffness) {
    return icon::InvalidArgumentError(
        FixedStrCat<RealtimeStatus::kMaxMessageLength>(
            "stiffness_compliance_device must be >= ", kMinimumStiffness,
            ", but got ", preprocessed_stiffness_compliance_device,
            " instead."));
  }
  if (preprocessed_stiffness_environment < kMinimumStiffness) {
    return icon::InvalidArgumentError(
        FixedStrCat<RealtimeStatus::kMaxMessageLength>(
            "stiffness_environment must be >= ", kMinimumStiffness,
            ", but got ", preprocessed_stiffness_environment, " instead."));
  }
  if (preprocessed_desired_virtual_stiffness_contact < kMinimumStiffness) {
    return icon::InvalidArgumentError(
        FixedStrCat<RealtimeStatus::kMaxMessageLength>(
            "desired_virtual_stiffness_contact must be >= ", kMinimumStiffness,
            ", but got ", preprocessed_desired_virtual_stiffness_contact,
            " instead."));
  }

  const double stiffness_robot_joints_inverse =
      1.0 / preprocessed_stiffness_robot_joints;
  const double stiffness_compliance_device_inverse =
      1.0 / preprocessed_stiffness_compliance_device;
  const double stiffness_environment_inverse =
      1.0 / preprocessed_stiffness_environment;
  const double stiffness_combined_inverse =
      stiffness_robot_joints_inverse + stiffness_compliance_device_inverse +
      stiffness_environment_inverse;
  const double stiffness_combined = 1.0 / stiffness_combined_inverse;

  const double kappa =
      stiffness_combined / preprocessed_desired_virtual_stiffness_contact;

  switch (input_parameters.criterion) {
    case OverdampingConstantCriterion::kBenady:
      return sqrt(1.0 + kappa);
    case OverdampingConstantCriterion::kHInfinity:
      return 0.5 * (sqrt(1.0 + (2.0 * kappa)) - 1.0);
    case OverdampingConstantCriterion::kStrictlyPositiveReal:
      return 0.5 * (sqrt(1.0 + kappa) - 1.0);
    default:
      return icon::InvalidArgumentError(
          FixedStrCat<RealtimeStatus::kMaxMessageLength>(
              "Unknown criterion: ", input_parameters.criterion));
  }
}

RealtimeStatusOr<eigenmath::MatrixNd> ComputeOverdamping(
    const eigenmath::MatrixNd& inertia, double environment_stiffness) {
  // An arbitrary, positive stiffness value is used to compute overdamping,
  // though the resulting damping matrix is only slightly affected by it.
  constexpr double kMockStiffness = 1234.0;
  eigenmath::MatrixNd mock_stiffness_matrix =
      eigenmath::MatrixNd::Identity(inertia.rows(), inertia.cols()) *
      kMockStiffness;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      eigenmath::MatrixNd critical_damping,
      ComputeCriticalDamping(inertia, mock_stiffness_matrix));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      double damping_coeff,
      ComputeOverdampingConstantFromStiffnessScalars({
          .desired_virtual_stiffness_contact = kMockStiffness,
          .stiffness_compliance_device = environment_stiffness,
      }));

  return eigenmath::MatrixNd(critical_damping * damping_coeff);
}

}  // namespace intrinsic::icon
