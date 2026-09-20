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

#ifndef INTRINSIC_ICON_SKILLS_AIO_READ_INPUT_H_
#define INTRINSIC_ICON_SKILLS_AIO_READ_INPUT_H_

#include <memory>
#include <utility>

#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
#include "intrinsic/icon/equipment/channel_factory.h"
#include "intrinsic/skills/cc/skill_interface.h"

namespace intrinsic::skills {

class AioReadInput final : public SkillInterface {
 public:
  static constexpr char kSkillName[] = "aio_read_input";
  static constexpr char kEquipmentSlot[] = "adio";

  static std::unique_ptr<SkillInterface> CreateSkill();

  explicit AioReadInput(
      std::unique_ptr<icon::ChannelFactory> icon_channel_factory)
      : icon_channel_factory_(std::move(icon_channel_factory)) {}
  ~AioReadInput() override = default;

  absl::StatusOr<intrinsic_proto::skills::Footprint> GetFootprint(
      const GetFootprintRequest& request,
      GetFootprintContext& context) const override;

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const ExecuteRequest& request, ExecuteContext& context) override;

 private:
  // Factory used to instantiate the connection to ICON. This is injected so
  // that tests can use FakeChannelFactory.
  std::unique_ptr<icon::ChannelFactory> icon_channel_factory_;
};

}  // namespace intrinsic::skills

#endif  // INTRINSIC_ICON_SKILLS_AIO_READ_INPUT_H_
