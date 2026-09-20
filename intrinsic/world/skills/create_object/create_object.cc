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

#include "intrinsic/world/skills/create_object/create_object.h"

#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/random/distributions.h"
#include "absl/random/random.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "google/protobuf/message.h"
#include "grpcpp/client_context.h"
#include "intrinsic/assets/catalog/proto/v1/asset_catalog.pb.h"
#include "intrinsic/assets/dependencies/utils.h"
#include "intrinsic/assets/interface_utils.h"
#include "intrinsic/assets/proto/installed_assets.grpc.pb.h"
#include "intrinsic/assets/proto/installed_assets.pb.h"
#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/assets/proto/view.pb.h"
#include "intrinsic/assets/scene_objects/proto/scene_object_manifest.pb.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/proto/transformed_geometry_storage_refs.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/resources/proto/resource_handle.pb.h"
#include "intrinsic/scene/proto/v1/entity.pb.h"
#include "intrinsic/scene/util/scene_object_from_asset.h"
#include "intrinsic/simulation/resources/spawner_server.grpc.pb.h"
#include "intrinsic/simulation/resources/spawner_server.pb.h"
#include "intrinsic/skills/cc/execute_context.h"
#include "intrinsic/skills/cc/execute_request.h"
#include "intrinsic/skills/cc/preview_context.h"
#include "intrinsic/skills/cc/preview_request.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/cc/skill_utils.h"
#include "intrinsic/skills/proto/footprint.pb.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_macros_grpc.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/proto/geometry_component.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"
#include "intrinsic/world/skills/create_object/create_object.pb.h"

using ::ai::intrinsic::CreateObjectParams;
using ObjectNamingSchema =
    ::ai::intrinsic::CreateObjectParams::ObjectNamingSchema;

namespace intrinsic::skills {

namespace {

std::string SpawnerInterfaceUri() {
  return absl::StrCat(
      intrinsic::assets::kGrpcUriPrefix,
      intrinsic_proto::simulation::Spawner::service_full_name());
}

std::string InstalledAssetsReaderInterfaceUri() {
  return absl::StrCat(
      intrinsic::assets::kGrpcUriPrefix,
      intrinsic_proto::assets::v1::InstalledAssetsReader::service_full_name());
}

absl::StatusOr<intrinsic_proto::scene_object::v1::SceneObject> GetSceneObject(
    const ai::intrinsic::CreateObjectParams& params) {
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> channel,
      intrinsic::assets::dependencies::
          ConnectWithRuntimeAssetFallbackForAssetMigrationOnly(
              params.intrinsic_runtime(), InstalledAssetsReaderInterfaceUri(),
              connect::UnlimitedMessageSizeGrpcChannelArgs()));
  INTR_RETURN_IF_ERROR(intrinsic::connect::WaitForChannelReady(channel));
  auto stub =
      intrinsic_proto::assets::v1::InstalledAssetsReader::NewStub(channel);
  return ::intrinsic::SceneObjectFromAsset(params.object_to_create(),
                                           stub.get());
}

std::string GenerateObjectName(const ObjectNamingSchema& schema,
                               absl::string_view default_prefix,
                               const int sequence_idx) {
  const std::string prefix =
      schema.has_prefix() ? schema.prefix() : std::string(default_prefix);
  const std::string temporal_name =
      absl::StrCat(prefix, "_", absl::GetCurrentTimeNanos());

  switch (schema.suffix()) {
    case CreateObjectParams::SUFFIX_UNSPECIFIED:
      return temporal_name;
    case CreateObjectParams::INDEX:
      return absl::StrCat(prefix, "_", sequence_idx);
    case CreateObjectParams::TIMESTAMP:
      return temporal_name;
    default:
      // Enums are open by default in proto3, so we need an explicit `default`
      // case here to avoid compilation errors.
      // https://protobuf.dev/programming-guides/enum/
      return temporal_name;
  }
}

// Randomizes the pose of an object based on the randomization params.
// TODO(b/412365599): Unify this logic with the one in
class PoseRandomizer {
 public:
  explicit PoseRandomizer(
      ai::intrinsic::CreateObjectParams::Randomization randomization_params) {
    if (randomization_params.has_positional_randomization_domain()) {
      prd_ = FromProto(randomization_params.positional_randomization_domain());
      prd_.x() = std::abs(prd_.x());
      prd_.y() = std::abs(prd_.y());
      prd_.z() = std::abs(prd_.z());
    }
    rotation_randomization_ = randomization_params.randomize_any_rotation();
  }

  Pose3d GetRandomized(Pose3d original) {
    // TODO(b/413455933): Add support for determinism between skill projection
    // and execution.
    Pose3d randomized = original;
    eigenmath::Vector3d delta_translation(
        absl::Uniform(gen_, -prd_.x(), prd_.x()),
        absl::Uniform(gen_, -prd_.y(), prd_.y()),
        absl::Uniform(gen_, -prd_.z(), prd_.z()));

    randomized.translation() += delta_translation;
    if (rotation_randomization_) {
      randomized.setQuaternion(eigenmath::Quaterniond::UnitRandom());
    }
    return randomized;
  };

  eigenmath::Vector3d prd_ = eigenmath::Vector3d::Zero();

  bool rotation_randomization_ = false;
  absl::BitGen gen_;
};

// Gets the final randomized poses for the objects to create from the
// CreateObjectParams based on randomization and the initial poses.
absl::StatusOr<std::vector<Pose3d>> GetRandomizedPoses(
    const ai::intrinsic::CreateObjectParams& create_object_params) {
  std::vector<Pose3d> poses;
  PoseRandomizer randomizer(create_object_params.randomization());
  for (const auto& pose_proto : create_object_params.poses()) {
    INTR_ASSIGN_OR_RETURN(Pose3d pose, FromProto(pose_proto),
                          _ << " for poses");
    poses.push_back(randomizer.GetRandomized(pose));
  }
  return poses;
}

// Creates the objects from the installed asset and returns the objects
// created.
absl::StatusOr<std::unique_ptr<ai::intrinsic::CreateObjectResult>>
CreateObjects(const ai::intrinsic::CreateObjectParams& create_object_params,
              world::ObjectWorldClient& world) {
  if (!create_object_params.has_object_to_create()) {
    return absl::InvalidArgumentError(
        "No object_to_create was provided to create the object from.");
  }

  if (create_object_params.attach_to_create_at_frame() &&
      !create_object_params.has_create_at_frame()) {
    return absl::InvalidArgumentError(
        "No create_at_frame was provided to attach the object to.");
  }

  if (create_object_params.poses().empty()) {
    return absl::InvalidArgumentError(
        "No pose was provided to create the object at.");
  }

  if ((create_object_params.object_naming_schema().has_prefix() ||
       create_object_params.object_naming_schema().has_suffix()) &&
      !create_object_params.object_names().empty()) {
    return absl::InvalidArgumentError(
        "Only one of object_naming_schema or object_names should be set.");
  }

  if (!create_object_params.object_names().empty()) {
    if (create_object_params.object_names_size() !=
        create_object_params.poses_size()) {
      return absl::InvalidArgumentError(
          "If object_names is set, its size must match the number of poses.");
    }

    for (const std::string& name : create_object_params.object_names()) {
      if (name.empty()) {
        return absl::InvalidArgumentError("object_names cannot be empty.");
      }
    }
  }

  if (create_object_params.object_naming_schema().has_prefix() &&
      create_object_params.object_naming_schema().prefix().empty()) {
    return absl::InvalidArgumentError(
        "object_naming_schema prefix cannot be an empty string.");
  }
  if (create_object_params.object_naming_schema().has_suffix()) {
    auto suffix_valid = [](auto suffix) {
      switch (suffix) {
        case CreateObjectParams::SUFFIX_UNSPECIFIED:
        case CreateObjectParams::INDEX:
        case CreateObjectParams::TIMESTAMP:
          return true;
        default:
          // Enums are open by default in proto3, so we need an explicit
          // `default` case here to avoid compilation errors.
          // https://protobuf.dev/programming-guides/enum/
          return false;
      }
    };

    if (!suffix_valid(create_object_params.object_naming_schema().suffix())) {
      return absl::InvalidArgumentError(
          "object_naming_schema suffix has an invalid value.");
    }
  }

  INTR_ASSIGN_OR_RETURN(const auto scene_object,
                        GetSceneObject(create_object_params));

  // Calculate the randomized poses for the objects to create.
  INTR_ASSIGN_OR_RETURN(auto poses, GetRandomizedPoses(create_object_params));
  const auto& create_in = create_object_params.create_in_world();
  bool create_in_sim = create_in == CreateObjectParams::SIM ||
                       create_in == CreateObjectParams::BELIEF_AND_SIM;
  bool create_in_belief = create_in != CreateObjectParams::SIM;

  auto result = std::make_unique<ai::intrinsic::CreateObjectResult>();

  // Defaults to root as the parent.
  INTR_ASSIGN_OR_RETURN(world::WorldObject root_object, world.GetRootObject());
  intrinsic_proto::world::TransformNodeReference attachment_parent =
      root_object.AsTransformNode().TransformNodeReference();
  Pose3d parent_t_pose = Pose3d::Identity();
  if (create_object_params.has_create_at_frame()) {
    // Changes the parent if the new object would be attached to a frame.
    if (create_object_params.attach_to_create_at_frame()) {
      attachment_parent = create_object_params.create_at_frame();
    } else {
      // Gets the pose of the frame relative to the root object if the object
      // would not be attached to the frame. This is needed to calculate the
      // pose of the object relative to the root.
      INTR_ASSIGN_OR_RETURN(
          auto parent_node,
          world.GetTransformNode(create_object_params.create_at_frame()));
      INTR_ASSIGN_OR_RETURN(parent_t_pose, world.GetTransform(parent_node));
    }
  }

  // Creates the objects at the provided poses and returns their references.
  for (int i = 0; i < poses.size(); ++i) {
    const auto& pose = poses[i];
    const std::string object_name =
        !create_object_params.object_names().empty()
            ? create_object_params.object_names(i)
            : GenerateObjectName(create_object_params.object_naming_schema(),
                                 scene_object.name(), i);
    if (create_in_belief) {
      auto name = WorldObjectName(object_name);
      // TODO(qingyou): I need a relative pose from parent to entity so that I
      // can get the parent_t_created_object pose.
      INTR_ASSIGN_OR_RETURN(auto parent_node,
                            world.GetTransformNode(attachment_parent));
      INTR_RETURN_IF_ERROR(world.CreateObjectFromSceneObject(
          scene_object, name,
          {
              .parent = parent_node,
              .parent_t_created_object = parent_t_pose * pose,
          }));

      INTR_ASSIGN_OR_RETURN(auto object, world.GetObject(name));
      *result->add_objects() = object.ObjectReference();
    }
    if (create_in_sim) {
      if (!create_object_params.create_object_service().interfaces().contains(
              SpawnerInterfaceUri())) {
        LOG(WARNING)
            << "No spawner dependency provided, skipping create in sim.";
      } else {
        INTR_ASSIGN_OR_RETURN(std::shared_ptr<grpc::Channel> channel,
                              intrinsic::assets::dependencies::Connect(
                                  create_object_params.create_object_service(),
                                  SpawnerInterfaceUri()));
        auto spawner_client =
            intrinsic_proto::simulation::Spawner::NewStub(channel);
        intrinsic_proto::simulation::SpawnObjectRequest spawn_in_sim_request;
        *spawn_in_sim_request.mutable_scene_object() = scene_object;
        *spawn_in_sim_request.mutable_parent() = attachment_parent;
        *spawn_in_sim_request.mutable_spawn_offset() =
            ToProto(parent_t_pose * pose);
        spawn_in_sim_request.set_name(object_name);
        google::protobuf::Empty spawn_in_sim_response;
        grpc::ClientContext client_context;
        INTR_RETURN_IF_ERROR_GRPC(spawner_client->SpawnObject(
            &client_context, spawn_in_sim_request, &spawn_in_sim_response));
      }
    }
  }

  return result;
}

}  // namespace

std::unique_ptr<SkillInterface> CreateObject::CreateSkill() {
  return std::make_unique<CreateObject>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
CreateObject::Execute(const ExecuteRequest& request, ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(auto create_object_params,
                        request.params<ai::intrinsic::CreateObjectParams>());
  return CreateObjects(create_object_params, context.object_world());
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
CreateObject::Preview(const PreviewRequest& request, PreviewContext& context) {
  INTR_ASSIGN_OR_RETURN(auto create_object_params,
                        request.params<ai::intrinsic::CreateObjectParams>());
  return CreateObjects(create_object_params, context.object_world());
}

absl::StatusOr<intrinsic_proto::skills::Footprint> CreateObject::GetFootprint(
    const GetFootprintRequest& request, GetFootprintContext& context) const {
  // Lock the universe to prevent spawning new objects interfere with other
  // skills.
  return ::intrinsic::skills::CreateUniverseLockFootprint();
}

}  // namespace intrinsic::skills
