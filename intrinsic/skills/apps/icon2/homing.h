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

#ifndef INTRINSIC_SKILLS_APPS_ICON2_HOMING_H_
#define INTRINSIC_SKILLS_APPS_ICON2_HOMING_H_

#include <memory>

#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
#include "intrinsic/icon/equipment/channel_factory.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/equipment.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

namespace intrinsic::skills {

// A skill that initiates the homing of a robot drive.
class Homing final : public SkillInterface {
 public:
  static constexpr char kEquipmentSlot[] = "robot";

  // Creates a Homing skill.
  static std::unique_ptr<SkillInterface> CreateSkill();

  // Constructs a Homing skill using the provided ICON channel factory.
  explicit Homing(std::unique_ptr<icon::ChannelFactory> icon_channel_factory);

  // Returns the skill's footprint.
  absl::StatusOr<intrinsic_proto::skills::Footprint> GetFootprint(
      const GetFootprintRequest& request,
      GetFootprintContext& context) const override;

  // Executes the skill.
  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const ExecuteRequest& request, ExecuteContext& context) override;

  // Previews the skill.
  absl::StatusOr<std::unique_ptr<::google::protobuf::Message>> Preview(
      const PreviewRequest& request, PreviewContext& context) override;

 private:
  // Factory used to instantiate the connection to ICON. This is injected so
  // that tests can use FakeChannelFactory.
  std::unique_ptr<icon::ChannelFactory> icon_channel_factory_;
};

}  // namespace intrinsic::skills

#endif  // INTRINSIC_SKILLS_APPS_ICON2_HOMING_H_
