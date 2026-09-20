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

#ifndef INTRINSIC_SKILLS_INTERNAL_PREDICT_CONTEXT_VIEW_H_
#define INTRINSIC_SKILLS_INTERNAL_PREDICT_CONTEXT_VIEW_H_

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/geometry/storage/geometry_library.h"  
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/motion_planning/motion_planner_client.h"
#include "intrinsic/skills/cc/equipment_pack.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/internal/kinematic_object_for_position_part.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/frame.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object.h"

namespace intrinsic {
namespace skills {

// PredictContext that just stores references to the objects it provides.
//
// Can be used when a PredictContext is needed and the objects it provides are
// owned by some other object (such as a PreviewContext).
class PredictContextView : public PredictContext {
 public:
  PredictContextView(const EquipmentPack& equipment,
                     motion_planning::MotionPlannerClient& motion_planner,
                     world::ObjectWorldClient& object_world

                     ,
                     GeometryLibrary& geometry_library

                     )
      : equipment_(equipment),
        motion_planner_(motion_planner),
        object_world_(object_world)

        ,
        geometry_library_(geometry_library)

  {}

  motion_planning::MotionPlannerClient& motion_planner() override {
    return motion_planner_;
  }

  world::ObjectWorldClient& object_world() override { return object_world_; }


  GeometryLibrary& geometry_library() const override {
    return geometry_library_;
  }


  absl::StatusOr<world::KinematicObject> GetKinematicObjectForEquipment(
      absl::string_view equipment_name) override {
    return KinematicObjectForPositionPart(equipment_name, equipment_,
                                          object_world());
  }

  absl::StatusOr<world::WorldObject> GetObjectForEquipment(
      absl::string_view equipment_name) override {
    INTR_ASSIGN_OR_RETURN(
        const intrinsic_proto::resources::ResourceHandle handle,
        equipment_.GetHandle(equipment_name));
    return object_world().GetObject(handle);
  }

  absl::StatusOr<world::Frame> GetFrameForEquipment(
      absl::string_view equipment_name, absl::string_view frame_name) override {
    INTR_ASSIGN_OR_RETURN(
        const intrinsic_proto::resources::ResourceHandle handle,
        equipment_.GetHandle(equipment_name));
    return object_world().GetFrame(handle, FrameName(frame_name));
  }

 private:
  const EquipmentPack& equipment_;
  motion_planning::MotionPlannerClient& motion_planner_;
  world::ObjectWorldClient& object_world_;
  GeometryLibrary& geometry_library_;  
};

}  // namespace skills
}  // namespace intrinsic

#endif  // INTRINSIC_SKILLS_INTERNAL_PREDICT_CONTEXT_VIEW_H_
