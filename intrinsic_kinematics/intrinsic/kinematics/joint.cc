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

#include "intrinsic/kinematics/joint.h"

#include <memory>
#include <optional>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic {
namespace kinematics {
namespace {

icon::RealtimeStatus CheckJointParametersImpl(absl::string_view prefix,
                                              const Joint::Limits& limits) {
  if (limits.position.lower > limits.position.upper) {
    return icon::InternalError(icon::RealtimeStatus::StrCat(
        prefix, " position limits are invalid: lower=", limits.position.lower,
        ", upper=", limits.position.upper));
  }

  if (limits.velocity < 0) {
    return icon::InternalError(icon::RealtimeStatus::StrCat(
        prefix, " velocity limit is invalid: limit=", limits.velocity));
  }

  if (limits.acceleration < 0) {
    return icon::InternalError(icon::RealtimeStatus::StrCat(
        prefix, " acceleration limit is invalid: limit=", limits.acceleration));
  }

  if (limits.jerk < 0) {
    return icon::InternalError(icon::RealtimeStatus::StrCat(
        prefix, " jerk limit is invalid: limit=", limits.jerk));
  }

  if (limits.effort < 0) {
    return icon::InternalError(icon::RealtimeStatus::StrCat(
        prefix, " effort limit is invalid: limit=", limits.effort));
  }

  return icon::OkStatus();
}

}  // namespace

/*static*/
absl::StatusOr<std::unique_ptr<Joint>> Joint::Create(
    absl::string_view name, const Joint::Parameters& parameters,
    std::optional<LinearDependency> linear_dependency) {
  if (linear_dependency.has_value()) {
    // If there is no leading joint, this is not a proper dependency.
    if (linear_dependency->alpha_leading.empty()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "For Joint ", name,
          " a dependency was set, but the set of leading joints is empty."));
    }
    // If alpha_self is zero, this is not a fully actuated dependency. In this
    // case the joint degenerates to a mimicked joint, which is not handled yet.
    if (AlmostEquals(linear_dependency->alpha_self, 0.0)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "For Joint ", name,
          " a dependency was set, but alpha_self is zero. This would result in "
          "a mimicked joint, which are not supported yet."));
    }
  }

  // absl::WrapUnique due to private constructor.
  return absl::WrapUnique(new Joint(name, parameters, linear_dependency));
}

Joint::Joint(absl::string_view name, const Joint::Parameters& params,
             std::optional<LinearDependency> linear_dependency)
    : Element(name), params_(params), linear_dependency_(linear_dependency) {}

const Joint::PositionLimits& Joint::GetSystemPositionLimit() const {
  return params_.system_limits.position;
}

const Joint::PositionLimits& Joint::GetSoftPositionLimit() const {
  return params_.soft_limits.position;
}

const Joint::Limits& Joint::GetSystemLimits() const {
  return params_.system_limits;
}

const Joint::Limits& Joint::GetSoftLimits() const {
  return params_.soft_limits;
}

Joint::Type Joint::GetType() const { return params_.type; }

double Joint::GetDefaultConfiguration() const {
  return params_.default_configuration;
}

eigenmath::Vector3d Joint::GetAxis() const { return params_.axis; }

const Joint::Parameters& Joint::GetParameters() const { return params_; }

bool Joint::IsStaticFrame() const {
  // Check type of joint
  return params_.type == Joint::FIXED;
}

bool Joint::IsDof() const { return params_.type != Joint::FIXED; }

Pose3d Joint::GetJointOutboundTransform(double q_derived) const {
  switch (params_.type) {
    case Joint::REVOLUTE: {
      return Pose3d(parent_t_this_ *
                    Pose3d(Eigen::Quaterniond(
                        Eigen::AngleAxis<double>(q_derived, params_.axis))));
    }
    case Joint::PRISMATIC: {
      return Pose3d(parent_t_this_ *
                    Pose3d(eigenmath::Vector3d(q_derived * params_.axis)));
    }
    case Joint::FIXED: {
      return Pose3d(parent_t_this_);
    }
  }
}

bool Joint::IsDependent() const { return linear_dependency_.has_value(); }

icon::RealtimeStatusOr<const Joint::LinearDependency*>
Joint::GetLinearDependency() const {
  if (!linear_dependency_.has_value()) {
    return icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
        "Joint '", GetName(), "' has no linear dependency."));
  }

  // Returns a pointer to the value of the optional. This is safe because the
  // optional is set in the constructor and never modified afterwards.
  return &*linear_dependency_;
}

icon::RealtimeStatus Joint::CheckJointParameters() const {
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckJointParametersImpl("Soft", params_.soft_limits));
  INTRINSIC_RT_RETURN_IF_ERROR(
      CheckJointParametersImpl("System", params_.system_limits));
  return icon::OkStatus();
}

}  // namespace kinematics
}  // namespace intrinsic
