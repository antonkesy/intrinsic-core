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

#ifndef INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_OUTFEED_COMPONENT_H_
#define INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_OUTFEED_COMPONENT_H_

#include <functional>
#include <istream>
#include <optional>
#include <ostream>
#include <string>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "gz/math/Vector3.hh"
#include "gz/sim/components/Component.hh"

namespace intrinsic {
namespace simulation {

// When set on an entity, the outfeed manager will use this data to manage the
// outfeed operation.
struct OutfeedCmdData {
  // The world-space center of the bounds from which we will outfeed objects.
  gz::math::Vector3d world_pos;

  // The 3d bounding box centered on the entity to which this component is
  // attached.
  gz::math::Vector3d bounds;

  // The Intrinsic world object name prefix for which objects to remove.
  std::string prefix;

  // The callback to notify the calling RPC of outfeed status.
  std::function<void(absl::Status)> callback;
};

namespace details {
class OutfeedCmdDataSerializer {
 public:
  static std::ostream& Serialize(std::ostream& _out,
                                 const OutfeedCmdData& _cmd) {
    return _out;
  }

  static std::istream& Deserialize(std::istream& _in, OutfeedCmdData& _cmd) {
    // Make sure bounds are empty and that the callback does nothing but logs
    // that we serialized this awkwardly.
    _cmd.bounds = gz::math::Vector3d(0, 0, 0);
    _cmd.callback = [](absl::Status) {
      LOG(WARNING) << "Serialized the OutfeedCmd component. Ignoring command.";
    };
    return _in;
  }
};
}  // namespace details

// Used by the outfeed to indicate if we're going to remove products in the next
// iteration of the OutfeedManager's PreUpdate() loop. This is so that other
// systems can also trigger outfeed events during their Update() calls. The
// command is set by setting this component to true.
using OutfeedCmd =
    gz::sim::components::Component<OutfeedCmdData, class OutfeedCmdTag,
                                   details::OutfeedCmdDataSerializer>;

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_COMPONENTS_OUTFEED_COMPONENT_H_
