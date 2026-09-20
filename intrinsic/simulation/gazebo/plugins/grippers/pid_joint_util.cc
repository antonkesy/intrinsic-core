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

#include "intrinsic/simulation/gazebo/plugins/grippers/pid_joint_util.h"

#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "gz/math/PID.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Name.hh"
#include "intrinsic/simulation/gazebo/plugins/joint_device_util.h"
#include "intrinsic/util/status/status_macros.h"
#include "sdf/Element.hh"
#include "sdf/Joint.hh"

namespace intrinsic {
namespace simulation {
namespace {

// Fallback gains are chosen assuming a generalized mass of 1.
constexpr double kFallbackGainKp =
    kTargetPIDJointPositionClosedLoopFreqRadPerSec *
    kTargetPIDJointPositionClosedLoopFreqRadPerSec;
constexpr double kFallbackGainKd =
    2 * kTargetPIDJointPositionClosedLoopFreqRadPerSec;

constexpr char kElementNameKpGain[] = "kp";
constexpr char kElementNameKiGain[] = "ki";
constexpr char kElementNameKdGain[] = "kd";
constexpr char kGainElementName[] = "gains";

constexpr char kJointNameElementName[] = "joint";

using ::gz::sim::Entity;
using ::gz::sim::EntityComponentManager;
using NameComponent = ::gz::sim::components::Name;

absl::StatusOr<double> GetJointGeneralizedMass(
    Entity joint_entity, const EntityComponentManager& ecm) {
  std::optional<::sdf::JointType> joint_type =
      ecm.ComponentData<::gz::sim::components::JointType>(joint_entity);
  if (!joint_type.has_value()) {
    return FailedPreconditionErrorBuilder()
           << "ECM does not have a JointType component";
  }
  switch (*joint_type) {
    case ::sdf::JointType::PRISMATIC: {
      return GetChildLinkTotalMass(joint_entity,
                                   kJointDeviceJointNameScopeSeparator, ecm);
      break;
    }
    case ::sdf::JointType::REVOLUTE: {
      return GetChildLinkTotalInertia(joint_entity,
                                      kJointDeviceJointNameScopeSeparator, ecm);
      break;
    }
    default:
      return InvalidArgumentErrorBuilder()
             << "Unsupported joint type " << static_cast<int>(*joint_type);
  }
}

absl::StatusOr<double> GetCriticalDampingDGain(
    double p_gain, Entity joint_entity, const EntityComponentManager& ecm) {
  INTR_ASSIGN_OR_RETURN(double joint_generalized_mass,
                        GetJointGeneralizedMass(joint_entity, ecm));
  // Ref: https://en.wikipedia.org/wiki/Damping#Damping_ratio_definition
  return 2.0 * sqrt(p_gain * joint_generalized_mass);
}

struct PDGains {
  double p_gain;
  double d_gain;
};

absl::StatusOr<PDGains> GetStablePDGains(Entity joint_entity,
                                         const EntityComponentManager& ecm) {
  INTR_ASSIGN_OR_RETURN(double joint_generalized_mass,
                        GetJointGeneralizedMass(joint_entity, ecm));

  // Ref: https://en.wikipedia.org/wiki/Damping#Damping_ratio_definition
  double kp = kTargetPIDJointPositionClosedLoopFreqRadPerSec *
              kTargetPIDJointPositionClosedLoopFreqRadPerSec *
              joint_generalized_mass;
  double kd = 2.0 * kTargetPIDJointPositionClosedLoopFreqRadPerSec *
              joint_generalized_mass;
  return PDGains{.p_gain = kp, .d_gain = kd};
}

struct SdfPIDGains {
  std::optional<double> kp;
  std::optional<double> ki;
  std::optional<double> kd;

  void SetMissingGainsFrom(const SdfPIDGains& other) {
    if (!kp.has_value() && other.kp.has_value()) {
      kp = other.kp;
    }
    if (!ki.has_value() && other.ki.has_value()) {
      ki = other.ki;
    }
    if (!kd.has_value() && other.kd.has_value()) {
      kd = other.kd;
    }
  }

  gz::math::PID ToPid() const {
    return gz::math::PID(kp.value_or(kFallbackGainKp), ki.value_or(0),
                         kd.value_or(kFallbackGainKd));
  }
};
SdfPIDGains GetSdfPIDGains(const ::sdf::Element* absl_nonnull gains_element) {
  SdfPIDGains result;
  if (auto kp_element = gains_element->FindElement(kElementNameKpGain);
      kp_element != nullptr) {
    result.kp = kp_element->Get<double>();
  }
  if (auto ki_element = gains_element->FindElement(kElementNameKiGain);
      ki_element != nullptr) {
    result.ki = ki_element->Get<double>();
  }
  if (auto kd_element = gains_element->FindElement(kElementNameKdGain);
      kd_element != nullptr) {
    result.kd = kd_element->Get<double>();
  }
  return result;
}

absl::StatusOr<PIDJoint> GetPIDJoint(
    const ::sdf::Element* absl_nonnull joint_element, Entity parent_entity,
    const EntityComponentManager& ecm, SdfPIDGains sdf_default_gains) {
  std::string joint_name;
  if (joint_element->HasAttribute("name")) {
    joint_name = joint_element->GetAttribute("name")->GetAsString();
  } else {
    joint_name = joint_element->Get<std::string>();
  }
  if (joint_name.empty()) {
    return absl::InvalidArgumentError("Joint name is empty.");
  }
  SdfPIDGains gains = GetSdfPIDGains(joint_element);
  gains.SetMissingGainsFrom(sdf_default_gains);
  if (gains.ki.has_value() &&
      (!gains.kp.has_value() || !gains.kd.has_value())) {
    LOG(WARNING)
        << "Ki gain is set but either kp or kd gain is not set for joint '"
        << joint_name << "'. Ignoring ki gain.";
    gains.ki = std::nullopt;
  }
  if (gains.kd.has_value() && !gains.kp.has_value()) {
    LOG(WARNING) << "Kd gain is set but kp gain is not set for joint '"
                 << joint_name << "'. Ignoring kd gain.";
    gains.kd = std::nullopt;
  }
  INTR_ASSIGN_OR_RETURN(
      Entity joint_entity,
      GetJointEntityFromRelativeScopedName(
          joint_name, kJointDeviceJointNameScopeSeparator, parent_entity, ecm));
  if (gains.kp.has_value() && !gains.kd.has_value()) {
    // If only kp is set, try to set kd to critical damping.
    absl::StatusOr<double> critical_d =
        GetCriticalDampingDGain(*gains.kp, joint_entity, ecm);
    LOG_IF(WARNING, !critical_d.ok())
        << "Could not compute critical damping for joint '" << joint_name
        << "'. Error: " << critical_d.status();
    gains.kd = critical_d.value_or(kFallbackGainKd);
  } else if (!gains.kp.has_value()) {
    // If neither kp nor kd is set, try to set kp and kd for stable control.
    absl::StatusOr<PDGains> stable_pd_gains =
        GetStablePDGains(joint_entity, ecm);
    LOG_IF(WARNING, !stable_pd_gains.ok())
        << "Could not compute stable PD gains for joint '" << joint_name
        << "'. Error: " << stable_pd_gains.status();
    gains.kp = stable_pd_gains.ok() ? stable_pd_gains->p_gain : kFallbackGainKp;
    gains.kd = stable_pd_gains.ok() ? stable_pd_gains->d_gain : kFallbackGainKd;
  }
  CHECK(gains.kp.has_value());
  CHECK(gains.kd.has_value());
  LOG(INFO) << "Set PID gains for joint '" << joint_name
            << "', kp = " << *gains.kp << ", ki = " << gains.ki.value_or(0)
            << ", kd = " << *gains.kd;
  return PIDJoint{.joint_entity = joint_entity,
                  .pid = gains.ToPid(),
                  .joint_name = std::move(joint_name)};
}
}  // namespace

absl::StatusOr<std::vector<PIDJoint>> GetPIDJoints(
    const Entity& entity, const ::sdf::Element* sdf_element,
    EntityComponentManager& ecm) {
  if (sdf_element == nullptr) {
    return absl::InvalidArgumentError("SDF element is null.");
  }
  SdfPIDGains sdf_default_gains;
  if (auto gains_element = sdf_element->FindElement(kGainElementName);
      gains_element != nullptr) {
    sdf_default_gains = GetSdfPIDGains(gains_element.get());
  }

  std::vector<PIDJoint> result;
  for (auto joint_element = sdf_element->FindElement(kJointNameElementName);
       joint_element != nullptr;
       joint_element = joint_element->GetNextElement(kJointNameElementName)) {
    absl::StatusOr<PIDJoint> pid_joint =
        GetPIDJoint(joint_element.get(), entity, ecm, sdf_default_gains);
    if (!pid_joint.ok()) {
      LOG(ERROR) << "Failed to get PID joint, error: " << pid_joint.status()
                 << ", for joint xml: "
                 << joint_element->ToString(/*_prefix=*/"");
    } else {
      result.push_back(*pid_joint);
    }
  }
  return result;
}

}  // namespace simulation
}  // namespace intrinsic
