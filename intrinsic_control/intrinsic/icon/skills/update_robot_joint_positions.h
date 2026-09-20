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

#ifndef INTRINSIC_ICON_SKILLS_UPDATE_ROBOT_JOINT_POSITIONS_H_
#define INTRINSIC_ICON_SKILLS_UPDATE_ROBOT_JOINT_POSITIONS_H_

#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
#include "intrinsic/icon/equipment/channel_factory.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

namespace intrinsic {
namespace skills {

// This skill reads the joint configuration from a robot and updates the world.
// It is intended to be called after the robot has been jogged manually.
class UpdateRobotJointPositions : public SkillInterface {
 public:
  static constexpr char kEquipmentSlot[] = "robot";

  static std::unique_ptr<SkillInterface> CreateSkill();

  // The Constructor is intended for tests that inject a custom ChannelFactory.
  explicit UpdateRobotJointPositions(
      std::unique_ptr<icon::ChannelFactory> icon_channel_factory)
      : icon_channel_factory_(std::move(icon_channel_factory)) {}

  // Disallow copy- and move- construction and assignment.
  UpdateRobotJointPositions(const UpdateRobotJointPositions&) = delete;
  UpdateRobotJointPositions& operator=(const UpdateRobotJointPositions&) =
      delete;

  // Get the robot joint configuration, log whether the joint configuration in
  // the world differs from the one read from the equipment, and update the
  // world state.
  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const ExecuteRequest& request, ExecuteContext& context) override;

 private:
  // Factory used to instantiate the connection to ICON. This is injected so
  // that tests can use FakeChannelFactory.
  std::unique_ptr<icon::ChannelFactory> icon_channel_factory_;
};

}  // namespace skills
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_SKILLS_UPDATE_ROBOT_JOINT_POSITIONS_H_
