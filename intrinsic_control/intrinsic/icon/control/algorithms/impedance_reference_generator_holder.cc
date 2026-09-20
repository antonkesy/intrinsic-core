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

#include "intrinsic/icon/control/algorithms/impedance_reference_generator_holder.h"

#include <variant>

#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_reference_generator_interface.h"
#include "intrinsic/icon/control/algorithms/position_reflexxes_cartesian_impedance_reference_generator.h"
#include "intrinsic/icon/control/algorithms/quintic_spline_cartesian_impedance_reference_generator.h"
#include "intrinsic/icon/control/algorithms/reflexxes_nullspace_reference_generator.h"
#include "intrinsic/icon/control/algorithms/simple_cartesian_impedance_reference_generator.h"
#include "intrinsic/icon/control/algorithms/velocity_reflexxes_cartesian_impedance_reference_generator.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"

namespace intrinsic::icon {

namespace {

struct ImpedanceReferenceGeneratorVisitor {
  PositionReflexxesCartesianImpedanceReferenceGenerator&
      position_reflexxes_generator;
  VelocityReflexxesCartesianImpedanceReferenceGenerator&
      velocity_reflexxes_generator;
  SimpleCartesianImpedanceReferenceGenerator& simple_generator;
  QuinticSplineCartesianImpedanceReferenceGenerator& quintic_generator;
  CartesianImpedanceReferenceGeneratorInterface*& active_generator;

  // Configures the quintic spline Cartesian impedance reference generator and
  // sets it as active, returns an error-status in case of configuration
  // failure.
  RealtimeStatus operator()(
      const cartesian_impedance::QuinticSplineCartesianGeneratorParameters&
          params) {
    INTRINSIC_RT_RETURN_IF_ERROR(quintic_generator.Configure(params));
    active_generator = &quintic_generator;
    return OkStatus();
  }

  // Configures the simple lowpass Cartesian impedance reference generator and
  // sets it as active, returns an error-status in case of configuration
  // failure.
  RealtimeStatus operator()(
      const cartesian_impedance::SimpleImpedanceGeneratorParameters& params) {
    INTRINSIC_RT_RETURN_IF_ERROR(simple_generator.Configure(params));
    active_generator = &simple_generator;
    return OkStatus();
  }

  // Configures the Cartesian Reflexxes impedance reference generator and
  // sets it as active, returns an error-status in case of configuration
  // failure.
  RealtimeStatus operator()(
      const cartesian_impedance::
          CartesianPositionReflexxesImpedanceGeneratorParameters& params) {
    active_generator = &position_reflexxes_generator;
    return OkStatus();
  }

  // Configures the Cartesian Velocity impedance reference generator and
  // sets it as active, returns an error-status in case of configuration
  // failure.
  RealtimeStatus operator()(
      const cartesian_impedance::
          CartesianVelocityReflexxesImpedanceGeneratorParameters& params) {
    active_generator = &velocity_reflexxes_generator;
    return OkStatus();
  }
};

struct NullspaceReferenceGeneratorVisitor {
  ReflexxesNullspaceReferenceGenerator& reflexxes_generator;
  SimpleNullspaceReferenceGenerator& simple_generator;
  QuinticSplineNullspaceReferenceGenerator& quintic_generator;
  NullspaceReferenceGeneratorInterface*& active_generator;

  // Configures the Reflexxes nullspace reference generator and sets it
  // as active, returns an error-status in case of configuration failure.
  RealtimeStatus operator()(
      const cartesian_impedance::ReflexxesNullspaceReferenceGeneratorParameters&
          params) {
    active_generator = &reflexxes_generator;
    return OkStatus();
  }

  // Configures the simple lowpass nullspace reference generator and sets it
  // as active, returns an error-status in case of configuration failure.
  RealtimeStatus operator()(
      const cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters&
          params) {
    INTRINSIC_RT_RETURN_IF_ERROR(simple_generator.Configure(params));
    active_generator = &simple_generator;
    return OkStatus();
  }

  // Configures the quintic spline nullspace reference generator and sets it
  // as active, returns an error-status in case of configuration failure.
  RealtimeStatus operator()(
      const cartesian_impedance::
          QuinticSplineNullspaceReferenceGeneratorParameters& params) {
    INTRINSIC_RT_RETURN_IF_ERROR(quintic_generator.Configure(params));
    active_generator = &quintic_generator;
    return OkStatus();
  }
};

}  // namespace

ImpedanceReferenceGeneratorHolder::ImpedanceReferenceGeneratorHolder(
    double frequency_hz)
    : position_reflexxes_generator_(frequency_hz),
      velocity_reflexxes_generator_(frequency_hz),
      quintic_generator_(1.0 / frequency_hz),
      active_generator_(&simple_generator_) {}

CartesianImpedanceReferenceGeneratorInterface&
ImpedanceReferenceGeneratorHolder::GetActiveReferenceGenerator() {
  return *active_generator_;
}

RealtimeStatus ImpedanceReferenceGeneratorHolder::ConfigureReferenceGenerator(
    const std::variant<
        cartesian_impedance::
            CartesianPositionReflexxesImpedanceGeneratorParameters,
        cartesian_impedance::
            CartesianVelocityReflexxesImpedanceGeneratorParameters,
        cartesian_impedance::SimpleImpedanceGeneratorParameters,
        cartesian_impedance::QuinticSplineCartesianGeneratorParameters>&
        variants) {
  return std::visit(
      ImpedanceReferenceGeneratorVisitor{
          .position_reflexxes_generator = position_reflexxes_generator_,
          .velocity_reflexxes_generator = velocity_reflexxes_generator_,
          .simple_generator = simple_generator_,
          .quintic_generator = quintic_generator_,
          .active_generator = active_generator_},
      variants);
}

NullspaceReferenceGeneratorHolder::NullspaceReferenceGeneratorHolder(
    int njoints, double frequency_hz)
    : reflexxes_generator_(njoints, frequency_hz),
      quintic_generator_(1.0 / frequency_hz),
      active_generator_(&simple_generator_) {}

NullspaceReferenceGeneratorInterface&
NullspaceReferenceGeneratorHolder::GetActiveReferenceGenerator() {
  return *active_generator_;
}

RealtimeStatus NullspaceReferenceGeneratorHolder::ConfigureReferenceGenerator(
    const std::variant<
        cartesian_impedance::ReflexxesNullspaceReferenceGeneratorParameters,
        cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters,
        cartesian_impedance::
            QuinticSplineNullspaceReferenceGeneratorParameters>& variants) {
  return std::visit(
      NullspaceReferenceGeneratorVisitor{
          .reflexxes_generator = reflexxes_generator_,
          .simple_generator = simple_generator_,
          .quintic_generator = quintic_generator_,
          .active_generator = active_generator_},
      variants);
}

}  // namespace intrinsic::icon
