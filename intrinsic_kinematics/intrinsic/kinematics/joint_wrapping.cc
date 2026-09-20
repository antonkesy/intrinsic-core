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

#include "intrinsic/kinematics/joint_wrapping.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/joint.h"
#include "intrinsic/kinematics/model_interface.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic {
namespace kinematics {

using eigenmath::VectorNd;

icon::RealtimeStatusOr<bool> IsWithinLimitsAfterWrapping(
    const ModelInterface& model, const JointStateP& joint_state,
    const JointLimits& joint_limits) {
  if (joint_state.size() != joint_limits.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of provided joint state of ", joint_state.size(),
        " is different from the limit size ", joint_limits.size()));
  }
  if (!joint_limits.IsValid()) {
    return icon::InvalidArgumentError("Joint limit is invalid.");
  }

  const VectorNd& q = joint_state.position;

  for (std::size_t i = 0; i < joint_state.size(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto joint_id,
                                  model.GetElementIdForDofIndex(i));
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint, model.GetJoint(joint_id));
    const auto& joint_params = joint->GetParameters();

    if (q[i] < joint_limits.min_position[i] ||
        q[i] > joint_limits.max_position[i]) {
      // q is outside the limit. We try to wrap it within the limits.
      switch (joint_params.type) {
        case Joint::PRISMATIC:
          // Joint is prismatic can't wrap"
          return false;
        case Joint::REVOLUTE: {
          double wrapped_q = WrapJoint(q[i], joint_limits.max_position[i],
                                       joint_limits.min_position[i]);
          if (!isfinite(wrapped_q)) {
            return false;
          }
          break;
        }
        default:
          return icon::InvalidArgumentError("Unsupported joint type.");
      }
    }
  }
  return true;
}

double WrapJoint(double val, double upper, double lower) {
  return WrapJoint(val, upper, lower, (upper + lower) / 2);
}

double WrapJoint(double val, double upper, double lower, double nearby) {
  double min_diff = std::numeric_limits<double>::max();
  double result = std::numeric_limits<double>::quiet_NaN();

  CHECK(std::isfinite(nearby));
  if (!std::isfinite(lower)) {
    lower = std::numeric_limits<double>::lowest();
  }
  if (!std::isfinite(upper)) {
    upper = std::numeric_limits<double>::max();
  }

  CHECK_GE(nearby, lower);
  CHECK_LE(nearby, upper);

  // If `val` is actually within `kJointWrappingBoundaryTolerance` away from
  // either the `upper` or `lower` bounds, we shall simply return `val` clamped
  // by [`lower`, `upper`].
  // This is to make sure we don't wrap if we are very close to the solution,
  // which is also why fmod is not used.
  if ((std::abs(upper - val) <= kJointWrappingBoundaryTolerance) ||
      (std::abs(val - lower) <= kJointWrappingBoundaryTolerance)) {
    return std::clamp(val, lower, upper);
  }

  double loop_val = val;
  while (loop_val < upper - kJointWrappingBoundaryTolerance) {
    double diff = std::abs(loop_val - nearby);
    if ((diff >= min_diff) &&
        (loop_val < upper - kJointWrappingBoundaryTolerance)) {
      // We are moving away from nearby and we satisfy the upper limits.
      break;
    }
    if ((loop_val > lower + kJointWrappingBoundaryTolerance) &&
        (loop_val < upper - kJointWrappingBoundaryTolerance) &&
        (diff < min_diff)) {
      result = loop_val;
      min_diff = diff;
    }
    loop_val += 2 * M_PI;
  }

  min_diff = std::numeric_limits<double>::max();
  while (loop_val > lower + kJointWrappingBoundaryTolerance) {
    double diff = std::abs(loop_val - nearby);

    if ((diff >= min_diff) &&
        (loop_val > lower + kJointWrappingBoundaryTolerance)) {
      // We are moving away from nearby and we satisfy the lower limits.
      break;
    }

    if ((loop_val > lower + kJointWrappingBoundaryTolerance) &&
        (loop_val < upper - kJointWrappingBoundaryTolerance) &&
        (diff < min_diff)) {
      result = loop_val;
      min_diff = diff;
    }
    loop_val -= 2 * M_PI;
  }

  if (result > upper || result < lower) {
    VLOG(2) << "Failed to wrap. result=" << result << ", lower=" << lower
            << ", upper=" << upper;
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (!isfinite(result)) {
    VLOG(2) << "Failed to wrap. val=" << val << ", lower=" << lower
            << ", upper=" << upper;
  }

  return result;
}

icon::RealtimeStatusOr<bool> Wrap(const ModelInterface& model,
                                  const JointLimits& dof_limits,
                                  const VectorNd& nearby_q, VectorNd* q) {
  if (q == nullptr) {
    return icon::InvalidArgumentError("q is null");
  }
  if (q->size() != model.GetNumberDegreesOfFreedom()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "q size=", q->size(),
        " != model.ndof=", model.GetNumberDegreesOfFreedom()));
  }
  if (!dof_limits.IsValid()) {
    return icon::InvalidArgumentError("Limit is invalid.");
  }

  if (!q->allFinite()) {
    return false;
  }

  // We can't rely on soft limit since they can change over time.
  // TODO(jeanfrancoisd): Provide a state to get the soft limits.
  const VectorNd& lower_q = dof_limits.min_position;
  const VectorNd& upper_q = dof_limits.max_position;
  for (size_t i = 0; i < model.GetNumberDegreesOfFreedom(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto joint_id,
                                  model.GetElementIdForDofIndex(i));
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto* joint, model.GetJoint(joint_id));
    if (joint->GetParameters().type == Joint::Type::REVOLUTE) {
      const double val = (*q)(i);
      const double wrapped_val =
          WrapJoint(val, upper_q(i), lower_q(i), nearby_q(i));
      if (!std::isfinite(wrapped_val)) {
        return false;
      }
      (*q)(i) = wrapped_val;
    }
  }

  return true;
}

// TODO(jeanfrancoisd): Consider converting to WrapFilterSort to avoid calling
// IsWithinLimits after this.
icon::RealtimeStatus WrapAndSort(const ModelInterface& model,
                                 const JointLimits& dof_limits,
                                 const eigenmath::VectorNd& nearby_q,
                                 absl::Span<eigenmath::VectorNd>* qs) {
  const int n_dof = model.GetNumberDegreesOfFreedom();
  if (nearby_q.size() != n_dof || dof_limits.size() != n_dof) {
    // We check size here to make sure we don't die in the sort loop.
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size mismatch, ndof=", n_dof, ", nearby_q=", nearby_q.size(),
        ", dof_limits=", dof_limits.size()));
  }
  if (!dof_limits.IsValid()) {
    return icon::InvalidArgumentError("Limits is invalid.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto is_within_limits,
                                IsWithinLimits(nearby_q, dof_limits));
  if (!is_within_limits) {
    return icon::InvalidArgumentError(
        "Nearby joint values should be within the dof limits.");
  }

  for (auto& q : *qs) {
    if (q.allFinite()) {
      INTRINSIC_RT_RETURN_IF_ERROR(
          Wrap(model, dof_limits, nearby_q, &q).status());
    }
  }

  std::sort(qs->begin(), qs->end(),
            [&nearby_q, &dof_limits](const eigenmath::VectorNd& qa,
                                     const eigenmath::VectorNd& qb) {
              // qa invalid: choose qb
              INTRINSIC_RT_ASSIGN_OR_DIE(auto qa_is_within_limits,
                                         IsWithinLimits(qa, dof_limits));
              if (!qa.allFinite() || !qa_is_within_limits) {
                return false;
              }
              // qa valid and qb invalid: choose qa
              INTRINSIC_RT_ASSIGN_OR_DIE(auto qb_is_within_limits,
                                         IsWithinLimits(qb, dof_limits));
              if (!qb.allFinite() || !qb_is_within_limits) {
                return true;
              }

              // qa and qb valid: choose the one closest to nearby_q (metric
              // based on L2 norm).
              return (qa - nearby_q).squaredNorm() <
                     (qb - nearby_q).squaredNorm();
            });

  return icon::OkStatus();
}

}  // namespace kinematics
}  // namespace intrinsic
