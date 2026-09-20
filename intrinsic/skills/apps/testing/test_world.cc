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

#include "intrinsic/skills/apps/testing/test_world.h"

#include <memory>
#include <optional>
#include <string>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/strings/strip.h"
#include "intrinsic/util/path_resolver/path_resolver.h"
#include "intrinsic/util/status/annotate.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/gzfile/gzfile.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/world.h"
#include "ortools/base/path.h"

namespace intrinsic {
namespace testing {

absl::StatusOr<::intrinsic::World> LoadTestWorld(
    absl::string_view world_gz_filepath) {
  // TODO(b/383612791): Remove google3 prefix from callers of LoadTestWorld
  const std::string kWorldFilename =
      PathResolver::ResolveRunfilesPath(absl::StripPrefix(
          absl::StripPrefix(absl::string_view(world_gz_filepath), "/"),
          "google3"));

  INTR_ASSIGN_OR_RETURN(auto gz_file, GZFile::Open(kWorldFilename));
  return World::FromFile(*gz_file);
}

absl::StatusOr<::intrinsic::World> LoadTestWorld(
    absl::string_view world_gz_filepath, const WorldObjectName& robot_name,
    absl::string_view robot_resource_name) {
  INTR_ASSIGN_OR_RETURN(World world, LoadTestWorld(world_gz_filepath));

  auto robot_ids = world.GetTypedEntityIds<RobotCollectionsEntityId>();
  if (robot_ids.size() != 1) {
    return absl::InternalError(absl::StrCat(
        "World should contain only one robot, but loaded world contains",
        robot_ids.size(), "."));
  }
  const RobotCollectionsEntityId robot_id = robot_ids[0];
  INTR_ASSIGN_OR_RETURN(WorldEntity * robot_ent, world.GetEntityById(robot_id));
  INTR_RETURN_IF_ERROR(robot_ent->SetAlias(robot_name.value()));

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<object_world::ObjectWorld> object_world,
                        object_world::ObjectWorld::CreateView(world));
  INTR_ASSIGN_OR_RETURN(
      object_world::KinematicObject * robot_object,
      object_world->GetKinematicObject(WorldObjectName(robot_name)));
  INTR_ASSIGN_OR_RETURN(std::optional<std::string> old_resource_name,
                        robot_object->GetResourceName());
  if (!old_resource_name) {
    INTR_RETURN_IF_ERROR(robot_object->SetResourceName(robot_resource_name));
  } else {
    LOG(WARNING) << "Robot named '" << robot_name
                 << "' already has associated resource name: "
                 << *old_resource_name;
  }
  return world;
}

}  // namespace testing
}  // namespace intrinsic
