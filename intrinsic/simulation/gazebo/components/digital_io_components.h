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

#ifndef INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_DIGITAL_IO_COMPONENTS_H_
#define INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_DIGITAL_IO_COMPONENTS_H_

#include <vector>

#include "gz/sim/components/Component.hh"

namespace intrinsic {
namespace simulation {

struct DigitalIoData {
  // Bit values for one digital input block. We expect std::vector<bool> to
  // explicitly be an optimized "bit vector" data type!
  std::vector<bool> data;
  // True if the digital input / output block was created from a World object
  // that has timeslicer devices.If true, then the simulated hardware module
  // appends a suffix to the name of the input/output. We can't just append the
  // suffix to the name when we convert from World object to SDF, because
  // SimBusHardwareModuleConfig uses the un-suffixed name as a key in the map
  // that prescribes the final hardware interface names, and we might want to
  // look that up.
  bool is_legacy = false;
};

// Note that this implies that every digital input block must be its own entity
// (since the same entity can only carry one instance of each Component class)
using DigitalInput =
    gz::sim::components::Component<DigitalIoData, class DigitalInputTag>;
// Bit values for one digital output block. We expect std::vector<bool> to
// explicitly be an optimized "bit vector" data type!
// Note that this implies that every digital input block must be its own entity
// (since the same entity can only carry one instance of each Component class)
using DigitalOutput =
    gz::sim::components::Component<DigitalIoData, class DigitalOutputTag>;

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_DIGITAL_IO_COMPONENTS_H_
