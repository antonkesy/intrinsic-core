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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_IMPEDANCE_REFERENCE_GENERATOR_HOLDER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_IMPEDANCE_REFERENCE_GENERATOR_HOLDER_H_

#include <variant>

#include "absl/types/variant.h"
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

// Used for configuring and selecting active impedance reference generators.
class ImpedanceReferenceGeneratorHolder {
 public:
  // `frequency_hz` is the control system update rate in Hz.
  explicit ImpedanceReferenceGeneratorHolder(double frequency_hz);

  // Configures the Cartesian impedance reference generator matching one of the
  // variants of `params` and sets it active. Returns an error-status in case of
  // configuration failure.
  RealtimeStatus ConfigureReferenceGenerator(
      const std::variant<
          cartesian_impedance::
              CartesianPositionReflexxesImpedanceGeneratorParameters,
          cartesian_impedance::
              CartesianVelocityReflexxesImpedanceGeneratorParameters,
          cartesian_impedance::SimpleImpedanceGeneratorParameters,
          cartesian_impedance::QuinticSplineCartesianGeneratorParameters>&
          variants);

  // Returns the Cartesian impedance reference generator which has been
  // configured in the last call to `ConfigureReferenceGenerator()`. Defaults to
  // the SimpleCartesianImpedanceReferenceGenerator.
  CartesianImpedanceReferenceGeneratorInterface& GetActiveReferenceGenerator();

 private:
  PositionReflexxesCartesianImpedanceReferenceGenerator
      position_reflexxes_generator_;
  VelocityReflexxesCartesianImpedanceReferenceGenerator
      velocity_reflexxes_generator_;
  SimpleCartesianImpedanceReferenceGenerator simple_generator_;
  QuinticSplineCartesianImpedanceReferenceGenerator quintic_generator_;
  CartesianImpedanceReferenceGeneratorInterface* active_generator_;
};

// Used for configuring and selecting active nullspace reference generators.
class NullspaceReferenceGeneratorHolder {
 public:
  // Takes the number of robot joints `njoints` and `frequency_hz`, the control
  // system update rate in Hz.
  NullspaceReferenceGeneratorHolder(int njoints, double frequency_hz);

  // Configures the nullspace impedance reference generator matching one of the
  // variants of `params` and sets it active. Returns an error-status in case of
  // configuration failure.
  RealtimeStatus ConfigureReferenceGenerator(
      const std::variant<
          cartesian_impedance::ReflexxesNullspaceReferenceGeneratorParameters,
          cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters,
          cartesian_impedance::
              QuinticSplineNullspaceReferenceGeneratorParameters>& variants);

  // Returns the nullspace reference generator which has been configured and
  // activated in the last call to `ConfigureReferenceGenerator()`. Defaults to
  // the SimpleNullspaceReferenceGenerator.
  NullspaceReferenceGeneratorInterface& GetActiveReferenceGenerator();

 private:
  ReflexxesNullspaceReferenceGenerator reflexxes_generator_;
  SimpleNullspaceReferenceGenerator simple_generator_;
  QuinticSplineNullspaceReferenceGenerator quintic_generator_;
  NullspaceReferenceGeneratorInterface* active_generator_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_IMPEDANCE_REFERENCE_GENERATOR_HOLDER_H_
