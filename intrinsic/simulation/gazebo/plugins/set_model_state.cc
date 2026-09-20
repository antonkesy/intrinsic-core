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

#include "intrinsic/simulation/gazebo/plugins/set_model_state.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "gz/common/Profiler.hh"
#include "gz/math/Angle.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "sdf/Element.hh"

namespace intrinsic::simulation {
using ::gz::math::Angle;
using ::gz::sim::Model;

namespace {
bool parseScalarWithDegrees(Angle& _scalar, ::sdf::ElementConstPtr _elem) {
  if (_elem) {
    // parse degrees attribute, default false
    std::pair<bool, bool> degreesPair = _elem->Get<bool>("degrees", false);
    // parse element scalar value, default 0.0
    std::pair<double, bool> scalarPair = _elem->Get<double>("", 0.0);
    if (scalarPair.second) {
      if (degreesPair.first) {
        _scalar.SetDegree(scalarPair.first);
      } else {
        _scalar.SetRadian(scalarPair.first);
      }
      return true;
    }
  }
  return false;
}
}  // namespace

void SetModelState::Configure(const ::gz::sim::Entity& _entity,
                              const std::shared_ptr<const ::sdf::Element>& _sdf,
                              ::gz::sim::EntityComponentManager& _ecm,
                              ::gz::sim::EventManager& /*_eventMgr*/) {
  this->model_ = Model(_entity);
  const std::string modelName = this->model_.Name(_ecm);
  //! [Configure]

  if (!this->model_.Valid(_ecm)) {
    LOG(ERROR) << "SetModelState plugin should be attached to a model entity. "
               << "Failed to initialize.";
    return;
  }

  auto modelStateElem = _sdf->FindElement("model_state");
  if (!modelStateElem) {
    LOG(ERROR) << "No <model_state> specified in model with name [" << modelName
               << "], not updating state.";
    return;
  }

  for (auto jointStateElem = modelStateElem->FindElement("joint_state");
       jointStateElem != nullptr;
       jointStateElem = jointStateElem->GetNextElement("joint_state")) {
    std::pair<std::string, bool> namePair =
        jointStateElem->Get<std::string>("name", "");
    if (!namePair.second) {
      LOG(ERROR) << "No name specified for joint_state, skipping.";
      continue;
    }
    const auto& jointName = namePair.first;

    gz::sim::Entity jointEntity = this->model_.JointByName(_ecm, jointName);
    if (jointEntity == ::gz::sim::kNullEntity) {
      LOG(ERROR) << "Unable to find joint with name [" << jointName << "] "
                 << "in model with name [" << modelName << "], skipping.";
      continue;
    }

    if (!_ecm.EntityHasComponentType(
            jointEntity, ::gz::sim::components::JointAxis::typeId)) {
      LOG(ERROR) << "Joint with name [" << jointName << "] "
                 << "in model with name [" << modelName << "] "
                 << "has no JointAxis component (is it a fixed joint?), "
                 << "skipping.";
      continue;
    }

    if (_ecm.EntityHasComponentType(
            jointEntity, ::gz::sim::components::JointAxis2::typeId)) {
      LOG(WARNING) << "Joint with name [" << jointName << "] "
                   << "in model with name [" << modelName << "] "
                   << "has a JointAxis2 component, but multi-axis joints are "
                   << "not yet supported by this plugin. Only the first "
                   << "joint axis state will be set.";
    }

    std::vector<double> jointVelocity;

    {
      auto axisElem = jointStateElem->FindElement("axis_state");
      if (axisElem) {
        auto positionElem = axisElem->FindElement("position");
        if (positionElem) {
          Angle position;
          if (parseScalarWithDegrees(position, positionElem)) {
            std::vector<double> jointPosition{position.Radian()};
            _ecm.SetComponentData<::gz::sim::components::JointPositionReset>(
                jointEntity, jointPosition);
            LOG(INFO) << "Set initial position of joint " << jointName << " to "
                      << position;
          }
        }

        auto velocityElem = axisElem->FindElement("velocity");
        if (velocityElem) {
          Angle velocity;
          if (parseScalarWithDegrees(velocity, velocityElem)) {
            std::vector<double> jointVelocity{velocity.Radian()};
            _ecm.SetComponentData<::gz::sim::components::JointVelocityReset>(
                jointEntity, jointVelocity);
            LOG(INFO) << "Set initial velocity of joint " << jointName << " to "
                      << velocity;
          }
        }
      }
      // else
      // {
      //   // <axis_state> not found
      // }
    }
  }
}

void SetModelState::Reset(const ::gz::sim::UpdateInfo& _info,
                          ::gz::sim::EntityComponentManager& _ecm) {
  GZ_PROFILE("SetModelState::Reset");

  // TODO(scpeters) Reset to the same state that was set in Configure
}

}  // namespace intrinsic::simulation
