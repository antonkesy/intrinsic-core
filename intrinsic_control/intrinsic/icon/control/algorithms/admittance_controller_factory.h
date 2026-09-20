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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_ADMITTANCE_CONTROLLER_FACTORY_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_ADMITTANCE_CONTROLLER_FACTORY_H_

#include <cstddef>
#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/control/algorithms/admittance_controller_interface.h"

namespace intrinsic::icon {

// Factory function to create an AdmittanceControllerInterface instance based on
// system parameters and build target (Core vs Enterprise).
absl::StatusOr<std::unique_ptr<AdmittanceControllerInterface>>
CreateAdmittanceController(size_t njoints, double control_frequency_hz,
                           const CartesianImpedanceParameters& params);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_ADMITTANCE_CONTROLLER_FACTORY_H_
