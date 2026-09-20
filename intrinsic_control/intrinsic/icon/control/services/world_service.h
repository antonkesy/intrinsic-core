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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_WORLD_SERVICE_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_WORLD_SERVICE_H_

#include <memory>

#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/world/objects/object_world_client.h"

namespace intrinsic::icon {

// WorldService provides (read) access to a shared World object to other
// timeslicer components. This serves as a central entry point of sorts for
// anything that wants to know about where links/frames are in relation to other
// links/frames (mostly rigid-body kinematics and dynamics models for robot
// parts, but also things like end-effector tooling).
//
// Implementations can load the World from various sources.
class WorldService {
 public:
  virtual ~WorldService() = default;

  // Returns a shared pointer to an ObjectWorldClient that communicates with the
  // world service. The client is configured to use the same World that this
  // ICON server resource is part of.
  // That is, you can use the client to look up information about any robots
  // connected to this ICON server.
  //
  // The shared pointer is always non-null (if a given WorldService
  // implementation can't connect to a World service, its Initialize() method
  // should fail). This function does not guarantee that the World has geometry
  // information.
  //
  // NOTE: NOT Realtime safe!
  //
  // std::shared_ptr manages the lifetime of the underlying ObjectWorldClient
  // object.
  virtual std::shared_ptr<const world::ObjectWorldClient> GetObjectWorldClient()
      const INTRINSIC_NON_REALTIME_ONLY = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_WORLD_SERVICE_H_
