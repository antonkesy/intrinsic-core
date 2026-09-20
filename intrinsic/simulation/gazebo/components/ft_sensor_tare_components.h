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

#ifndef INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_FT_SENSOR_TARE_COMPONENTS_H_
#define INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_FT_SENSOR_TARE_COMPONENTS_H_

#include <array>

#include "gz/sim/components/Component.hh"
#include "intrinsic/icon/utils/sensor_utils.h"

namespace intrinsic {
namespace simulation {

// Data structure copied from intrinsic::ati::AtiForceTorqueBusDevice.
struct ForceTorqueTaringData {
  // Sensor bias which gets set during taring operation and is subsequently
  // subtracted from wrench measurements to correct for unmodelled payload, etc.
  std::array<::intrinsic::icon::DofSensorBias, 6> ft_sensor_bias;
  bool taring_in_progress = false;
  int taring_cycles = 0;
};

using ForceTorqueTaring =
    gz::sim::components::Component<ForceTorqueTaringData,
                                   class ForceTorqueTaringTag>;

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_FT_SENSOR_TARE_COMPONENTS_H_
