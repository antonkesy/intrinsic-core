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

#ifndef INTRINSIC_ICON_SKILLS_MOVE_GRIPPER_JOINTS_H_
#define INTRINSIC_ICON_SKILLS_MOVE_GRIPPER_JOINTS_H_

#include <memory>

#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
#include "intrinsic/icon/equipment/channel_factory.h"
#include "intrinsic/icon/skills/move_gripper_joints.pb.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

namespace intrinsic::skills {

// Error codes, sync with manifest
constexpr absl::string_view kExtendedStatusComponent =
    "ai.intrinsic.move_gripper_joints";
constexpr int32_t kCollisionErrorCode = 10000;
constexpr int32_t kICONErrorCode = 10001;
constexpr int32_t kOutOfLimitsErrorCode = 10002;
constexpr int32_t kInvalidParametersErrorCode = 10003;

// Skill to move the gripper joints of an Icon2PositionPart.
// Uses the ICON Application Layer API to move the joints to a specified
// configuration. This skill performs collision checking.
class MoveGripperJoints : public SkillInterface {
 public:
  static constexpr char kEquipmentSlot[] = "gripper";

  static std::unique_ptr<SkillInterface> CreateSkill();

  static std::unique_ptr<SkillInterface> Create(
      std::unique_ptr<icon::ChannelFactory> icon_channel_factory);

  MoveGripperJoints(const MoveGripperJoints&) = delete;
  MoveGripperJoints& operator=(const MoveGripperJoints&) = delete;

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const ExecuteRequest& request, ExecuteContext& context) override;

  absl::StatusOr<std::unique_ptr<::google::protobuf::Message>> Preview(
      const PreviewRequest& request, PreviewContext& context) override;

  absl::StatusOr<intrinsic_proto::skills::Footprint> GetFootprint(
      const GetFootprintRequest& request,
      GetFootprintContext& context) const override;

 protected:
  // Factory used to instantiate the connection to ICON. This is injected so
  // that tests can use FakeChannelFactory.
  std::unique_ptr<icon::ChannelFactory> icon_channel_factory_;

 private:
  explicit MoveGripperJoints(
      std::unique_ptr<icon::ChannelFactory> icon_channel_factory);

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> ExecuteInternal(
      const ExecuteRequest& request, ExecuteContext& context);

  absl::StatusOr<std::unique_ptr<::google::protobuf::Message>> PreviewInternal(
      const PreviewRequest& request, PreviewContext& context);
};

}  // namespace intrinsic::skills

#endif  // INTRINSIC_ICON_SKILLS_MOVE_GRIPPER_JOINTS_H_
