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

#ifndef INTRINSIC_EXECUTIVE_ENGINE_SKILL_INSTANCE_H_
#define INTRINSIC_EXECUTIVE_ENGINE_SKILL_INSTANCE_H_

#include <functional>
#include <memory>
#include <optional>

#include "absl/base/thread_annotations.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "grpcpp/client_context.h"
#include "intrinsic/assets/dependencies/resolver.h"
#include "intrinsic/assets/proto/v1alpha1/asset_info_internal.pb.h"
#include "intrinsic/executive/clips_cpp/protobuf.h"
#include "intrinsic/executive/engine/skill_action.h"
#include "intrinsic/executive/proto/behavior_call.pb.h"
#include "intrinsic/skills/cc/equipment_pack.h"
#include "intrinsic/skills/internal/skill_registry_client_interface.h"
#include "intrinsic/skills/internal/skill_service_client_interface.h"
#include "intrinsic/skills/proto/skills.pb.h"

namespace intrinsic::executive {

using SkillServiceClientCreator = std::function<
    absl::StatusOr<std::unique_ptr<skills::SkillServiceClientInterface>>(
        intrinsic_proto::skills::SkillInstance)>;

// Establishes a connection to the skill service.
absl::StatusOr<std::unique_ptr<skills::SkillServiceClientInterface>>
CreateSkillServiceClient(
    const intrinsic_proto::skills::SkillInstance& instance_proto,
    std::optional<absl::Duration> connect_timeout,
    SkillServiceClientCreator skill_service_client_creator_mock);

// An instance for a specific skill call
class SkillInstance {
  SkillInstance();
  SkillInstance(const SkillInstance&) = delete;
  SkillInstance& operator=(const SkillInstance&) = delete;

 public:
  SkillInstance(SkillInstance&&) = default;
  SkillInstance& operator=(SkillInstance&&) = default;

  struct CreationContext {
    // Only set when the BehaviorCall has parameters
    std::optional<clips::ProtobufManager::DescriptorPoolInfo> pool_info;
    const assets::Resolver& resolver;
    intrinsic_proto::assets::v1alpha1::InternalSkillData::GrpcTargets
        grpc_targets;  // defines how to connect to this skill instance
    const absl::flat_hash_map<std::string,
                              intrinsic_proto::resources::ResourceHandle>&
        resource_handles;
    std::optional<absl::Duration> connect_timeout;
    SkillServiceClientCreator skill_service_client_creator_mock = nullptr;
    std::vector<std::string> required_equipment_keys;
    std::string id_version;
  };

  static absl::StatusOr<SkillInstance> Create(
      intrinsic_proto::executive::BehaviorCall& behavior_call,
      SkillAction skill_action, const CreationContext& context);

  const skills::SkillServiceClientInterface* GetClient() const {
    return client_.get();
  }
  skills::SkillServiceClientInterface* GetClient() { return client_.get(); }

  const intrinsic_proto::skills::SkillInstance& GetInstanceProto() const {
    return instance_;
  }

  void AbandonCall();
  bool IsAbandoned();
  void UpdateWaitContext(grpc::ClientContext* context);

  // Updates the wait context only if the call is not abandoned.
  // Returns true if the wait context was updated, false if the call was
  // abandoned.
  bool UpdateWaitContextUnlessAbandoned(grpc::ClientContext* context);

 private:
  // This fills the legacy EquipmentPack in equipment for a skill instance from
  // the ResourceHandles retrieved by the resource registry.
  absl::Status FillEquipmentPack(
      const intrinsic_proto::executive::BehaviorCall& behavior_call,
      const CreationContext& context);

  // Fills the instance proto by generating an instance name and retrieving the
  // grpc addresses for this skill instance.
  absl::Status InitializeInstanceProto(
      const intrinsic_proto::executive::BehaviorCall& behavior_call,
      const intrinsic_proto::assets::v1alpha1::InternalSkillData::GrpcTargets&
          grpc_targets,
      absl::string_view id_version);

  std::unique_ptr<skills::SkillServiceClientInterface> client_;
  intrinsic_proto::skills::SkillInstance instance_;

  grpc::ClientContext* absl_nullable wait_context_ ABSL_GUARDED_BY(mutex_) =
      nullptr;
  bool abandoned_ ABSL_GUARDED_BY(mutex_) = false;

  std::unique_ptr<absl::Mutex> mutex_;
};

}  // namespace intrinsic::executive

#endif  // INTRINSIC_EXECUTIVE_ENGINE_SKILL_INSTANCE_H_
