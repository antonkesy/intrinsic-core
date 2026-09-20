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

#include "intrinsic/icon/hal/interfaces/laser_tracker_state_utils.h"

#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "intrinsic/icon/flatbuffers/control_types_copy.h"
#include "intrinsic/icon/flatbuffers/transform_types.fbs.h"
#include "intrinsic/icon/hal/interfaces/laser_tracker_state.fbs.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic_fbs {

flatbuffers::DetachedBuffer BuildLaserTrackerState() {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  Transform pose_sensed;
  CopyTo(&pose_sensed, intrinsic::Pose3d());

  LaserTrackerStateBuilder builder(fbb);
  builder.add_pose_sensed(&pose_sensed);

  fbb.Finish(builder.Finish());
  return fbb.Release();
}
}  // namespace intrinsic_fbs
