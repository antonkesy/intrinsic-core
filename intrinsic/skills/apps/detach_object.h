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

#ifndef INTRINSIC_SKILLS_APPS_DETACH_OBJECT_H_
#define INTRINSIC_SKILLS_APPS_DETACH_OBJECT_H_

#include <memory>

#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
#include "intrinsic/skills/apps/detach_object.pb.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/world/objects/object_world_client.h"

namespace intrinsic::skills {

// detach_object
//
// Detaches one object from another in the world representation. Note that this
// DOES NOT control the gripper nor any other piece of hardware. See
// detach_object.proto for a description of the input.
class DetachObjectSkill : public SkillInterface {
 public:
  static std::unique_ptr<SkillInterface> CreateSkill();

  ~DetachObjectSkill() override = default;

  absl::StatusOr<intrinsic_proto::skills::Footprint> GetFootprint(
      const GetFootprintRequest& request,
      GetFootprintContext& context) const override;

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> Execute(
      const ExecuteRequest& request, ExecuteContext& context) override;

  absl::StatusOr<std::unique_ptr<::google::protobuf::Message>> Preview(
      const PreviewRequest& request, PreviewContext& context) override;

  static absl::StatusOr<std::unique_ptr<google::protobuf::Message>> ExecuteImpl(
      const intrinsic_proto::skills::DetachObjectParams& params,
      world::ObjectWorldClient& world);
};

}  // namespace intrinsic::skills

#endif  // INTRINSIC_SKILLS_APPS_DETACH_OBJECT_H_
