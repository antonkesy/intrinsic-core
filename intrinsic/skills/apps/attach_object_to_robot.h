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

#ifndef INTRINSIC_SKILLS_APPS_ATTACH_OBJECT_TO_ROBOT_H_
#define INTRINSIC_SKILLS_APPS_ATTACH_OBJECT_TO_ROBOT_H_

#include <memory>

#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
#include "intrinsic/skills/apps/attach_object_to_robot.pb.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/world/objects/object_world_client.h"

namespace intrinsic {
namespace skills {

class AttachObjectToRobotSkill : public SkillInterface {
 public:
  static std::unique_ptr<SkillInterface> CreateSkill();
  ~AttachObjectToRobotSkill() override = default;

  // Skill implementation
  absl::StatusOr<intrinsic_proto::skills::Footprint> GetFootprint(
      const GetFootprintRequest& request,
      GetFootprintContext& context) const override;

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const ExecuteRequest& request, ExecuteContext& context) override;

  absl::StatusOr<std::unique_ptr<::google::protobuf::Message>> Preview(
      const PreviewRequest& request, PreviewContext& context) override;

  static absl::StatusOr<std::unique_ptr<google::protobuf::Message>> ExecuteImpl(
      const intrinsic_proto::skills::AttachObjectToRobotParams& params,
      world::ObjectWorldClient& world);
};

}  // namespace skills
}  // namespace intrinsic

#endif  // INTRINSIC_SKILLS_APPS_ATTACH_OBJECT_TO_ROBOT_H_
