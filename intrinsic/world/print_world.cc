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

#include "intrinsic/world/print_world.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_replace.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/message.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/world/collision/coal_collision_checker.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/component/ppr_component.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/component/user_data_component.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/grouping.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/proto/robot_component.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace {

std::vector<AttachmentEntityId> GetChildrenOfSorted(
    const World& world, AttachmentEntityId entity_id) {
  std::vector<AttachmentEntityId> children = world.GetChildrenOf(entity_id);
  absl::c_sort(children);
  return children;
}

template <typename... ComponentTypes>
std::vector<world_entity_details::TypedResult<ComponentTypes...>>
GetTypedEntityIdsSorted(const World& world) {
  std::vector<world_entity_details::TypedResult<ComponentTypes...>> result =
      world.GetTypedEntityIds<ComponentTypes...>();
  absl::c_sort(result);
  return result;
}

std::vector<GroupId> GetGroupsSorted(const World& world,
                                     const EntityId entity_id) {
  std::vector<GroupId> groups;

  absl::StatusOr<PhysicalEntityId> object_id_or =
      world.ValidateEntity<PhysicalEntityId>(entity_id);
  if (!object_id_or.ok()) {
    // Currently, only physical objects can be in groups.
    return groups;
  }

  const Grouping& grouping = world.As<Grouping>();
  for (const GroupId& group_id : grouping.GetGroupIds()) {
    if (grouping.GetObjectIds(group_id).count(*object_id_or) != 0) {
      groups.push_back(group_id);
    }
  }
  absl::c_sort(groups);
  return groups;
}

void PrintfWorldInternal(const World& world, const EntityId& id,
                         const std::string& format, const std::string& prefix,
                         std::ostream* output_stream,
                         const PrintfWorldOptions& options) {
  PrintfEntity(world, id, format, prefix, output_stream, options);

  ASSIGN_OR_DIE(const WorldEntity* entity, world.GetEntityById(id));
  auto attachment_component_or = entity->GetComponent<AttachmentComponent>();
  if (attachment_component_or.ok()) {
    const std::string child_prefix = absl::StrCat(prefix, options.indent_str);
    for (const AttachmentEntityId child_id :
         GetChildrenOfSorted(world, (AttachmentEntityId)id)) {
      PrintfWorldInternal(world, child_id, format, child_prefix, output_stream,
                          options);
    }
  }
}

void PrintEntityHeadline(
    AttachmentEntityId id, const WorldEntity& entity, const World& world,
    absl::string_view prefix,
    const WorldHashSet<KinematicsEntityId>& joint_object_ids,
    std::ostream* output_stream) {
  *output_stream << prefix << "Object: " << id;
  if (!entity.GetLocalName().empty()) {
    *output_stream << " local_name: " << entity.GetLocalName();
  }
  if (!entity.GetAlias().empty()) {
    *output_stream << " alias: " << entity.GetAlias();
  }
  if (joint_object_ids.contains(KinematicsEntityId(id.value()))) {
    *output_stream << " is_joint: true";
  }
  if (std::vector<GroupId> groups = GetGroupsSorted(world, id);
      !groups.empty()) {
    *output_stream << " groups: "
                   << absl::StrJoin(groups, ",", absl::StreamFormatter());
  }
  if (const std::set<LabelId>& labels = entity.GetLabels(); !labels.empty()) {
    *output_stream << " labels: "
                   << absl::StrJoin(labels, ",", absl::StreamFormatter());
  }
  *output_stream << std::endl;
}

void PrintTo(const World& world,
             const WorldHashSet<KinematicsEntityId>& joint_object_ids,
             const AttachmentEntityId parent_id, const AttachmentEntityId id,
             const std::string& prefix, std::ostream* output_stream) {
  ASSIGN_OR_DIE(const WorldEntity* entity, world.GetEntityById(id));
  PrintEntityHeadline(id, *entity, world, prefix, joint_object_ids,
                      output_stream);

  const auto root_t_object = world.GetTransform(kRootEntityId, id);
  *output_stream << prefix << " "
                 << "Absolute pose: " << toStringXYZXYZW(root_t_object)
                 << " pose to parent: "
                 << toStringXYZXYZW(world.GetTransform(parent_id, id))
                 << std::endl;

  if (auto geometry_component = entity->GetComponent<GeometryComponent>();
      geometry_component.ok()) {
    auto geometry_names_set = (*geometry_component)->GetGeometryNames();
    std::vector<std::string> geometry_names(geometry_names_set.begin(),
                                            geometry_names_set.end());
    absl::c_sort(geometry_names);

    for (const auto& geometry_name : geometry_names) {
      absl::StatusOr<NamedGeometrySet> geometry_set =
          (*geometry_component)->GetGeometry(geometry_name);
      if (!geometry_set.ok()) {
        continue;
      }

      for (const auto& [_, entry] : *geometry_set) {
        const Geometry& shape = entry.shape();

        auto renderable = shape.GetRenderable();
        const std::string renderable_gltf =
            renderable != nullptr ? "gltf renderable" : "no renderable info";
        *output_stream << prefix << " "
                       << "Renderable: " << renderable_gltf
                       << " geometry:" << geometry_name << std::endl;
      }
    }
  }

  for (const AttachmentEntityId child_id : GetChildrenOfSorted(world, id)) {
    PrintTo(world, joint_object_ids, id, child_id, absl::StrCat(prefix, "  "),
            output_stream);
  }
}

void PrintPhysicalTreeTo(const World& world, const AttachmentEntityId id,
                         const std::string& prefix,
                         std::ostream* output_stream) {
  ASSIGN_OR_DIE(const WorldEntity* entity, world.GetEntityById(id));
  PrintEntityHeadline(id, *entity, world, prefix, /*joint_object_ids=*/{},
                      output_stream);

  for (const AttachmentEntityId child_id : GetChildrenOfSorted(world, id)) {
    PrintPhysicalTreeTo(world, child_id, absl::StrCat(prefix, "| "),
                        output_stream);
  }
}

std::string RobotStatesToString(const World& world) {
  auto robot_ids = GetTypedEntityIdsSorted<RobotCollectionsEntityId>(world);

  std::vector<std::string> lines;
  for (auto robot_id : robot_ids) {
    ASSIGN_OR_DIE(auto base_link, world.GetBaseLink(robot_id));
    auto dof_view_or_status = world.GetDofKinematicView(robot_id);
    if (!dof_view_or_status.ok()) {
      LOG(ERROR) << "Unable to get a dof view  for robot "
                 << world.GetLocalNameForEntityById(robot_id)
                 << " id=" << robot_id
                 << ", error: " << dof_view_or_status.status();
      continue;
    }

    lines.push_back(absl::StrCat(
        world.GetLocalNameForEntityById(robot_id), " id=", robot_id.value(),
        ", q=[", toString(dof_view_or_status.value()->GetDofValues()),
        "], root_t_base=[",
        toStringXYZXYZW(world.GetTransform(kRootEntityId, base_link)), "]"));
  }
  return absl::StrJoin(lines, "\n");
}

}  // namespace

void PrintfEntity(const World& world, const EntityId& id,
                  const std::string& format, const std::string& prefix,
                  std::ostream* output_stream,
                  const PrintfWorldOptions& options) {
  ASSIGN_OR_DIE(const WorldEntity* entity, world.GetEntityById(id));

  // Assemble data (would be nice if this was made lazy).
  const std::vector<GroupId> groups = GetGroupsSorted(world, id);
  const std::set<LabelId>& labels = entity->GetLabels();

  std::string user_data_map_str;
  auto user_data_component_or = entity->GetComponent<UserDataComponent>();
  if (user_data_component_or.ok()) {
    const auto& map = (*user_data_component_or)->UserDataMap();
    user_data_map_str = absl::StrJoin(map, ",", absl::PairFormatter(":"));
  }

  std::string absolute_pose_str = "";
  std::string relative_pose_str = "";
  auto attachment_component_or = entity->GetComponent<AttachmentComponent>();
  if (attachment_component_or.ok()) {
    relative_pose_str =
        toStringXYZXYZW((*attachment_component_or)->GetParentTThis());
    absolute_pose_str = toStringXYZXYZW(
        world.GetTransform(kRootEntityId, (AttachmentEntityId)id));
  }

  // Format the arguments.
  std::string out = format;
  absl::StrReplaceAll(
      {
          {"{id}", absl::StrCat(id.value())},
          {"{local_name}", entity->GetLocalName()},
          {"{alias}", entity->GetAlias()},
          {"{groups}", absl::StrJoin(groups, ",", absl::StreamFormatter())},
          {"{labels}", absl::StrJoin(labels, ",", absl::StreamFormatter())},
          {"{udm}", user_data_map_str},
          {"{pose_a}", absolute_pose_str},
          {"{pose_r}", relative_pose_str},
      },
      &out);
  absl::StrReplaceAll(
      {
          {"\n", absl::StrCat("\n", prefix)},
      },
      &out);
  *output_stream << prefix << out << std::endl;
}

void PrintfWorld(const World& world, const std::string& format,
                 std::ostream* output_stream,
                 const PrintfWorldOptions& options) {
  return PrintfWorldInternal(world, kRootEntityId, format, "", output_stream,
                             options);
}

void PrintAspectsTo(const World& world, std::ostream* output_stream) {
  const auto joints_vec = GetTypedEntityIdsSorted<KinematicsEntityId>(world);
  WorldHashSet<KinematicsEntityId> joint_object_ids(joints_vec.begin(),
                                                    joints_vec.end());
  for (const AttachmentEntityId child_id :
       GetChildrenOfSorted(world, kRootEntityId)) {
    PrintTo(world, joint_object_ids, kRootEntityId, child_id, /*prefix=*/"",
            output_stream);
  }

  *output_stream << "All Collections:\n";
  for (const auto& collection_id :
       GetTypedEntityIdsSorted<CollectionsComponentType>(world)) {
    const auto* entity = world.GetEntityById(collection_id).value();
    const auto* collection =
        world.GetComponentByEntityId<CollectionsComponent>(collection_id)
            .value();
    *output_stream << "  id: " << collection_id.value()
                   << " local_name: " << entity->GetLocalName();
    *output_stream << " labels: ";
    for (const LabelId& label_id : entity->GetLabels()) {
      *output_stream << label_id << ",";
    }
    *output_stream << "\n";
    *output_stream << "    children: "
                   << absl::StrJoin(collection->GetCollectionMembers(
                                        CollectionsComponent::kLinks),
                                    ", ", absl::StreamFormatter())
                   << "\n";
  }

  *output_stream << "All Robots:\n";
  for (const auto& robot_id :
       GetTypedEntityIdsSorted<RobotComponentType>(world)) {
    const auto* entity = world.GetEntityById(robot_id).value();
    const auto* robot =
        world.GetComponentByEntityId<RobotComponent>(robot_id).value();
    *output_stream << "  id: " << robot_id.value()
                   << " local_name: " << entity->GetLocalName();
    *output_stream << " labels: ";
    for (const LabelId& label_id : entity->GetLabels()) {
      *output_stream << label_id << ",";
    }
    *output_stream << "\n";
    *output_stream << "    data: "
                   << google::protobuf::ShortFormat(robot->ToProto().value())
                   << "\n";
  }

  *output_stream << "All Groups:\n";
  const auto& grouping = world.As<Grouping>();
  for (const auto& group : grouping.GetGroupIds()) {
    *output_stream << "\tGroup: " << group << "\n";
  }

  *output_stream << "Collision Exclusion Pairs:\n";
  std::vector<std::pair<PhysicalEntityId, PhysicalEntityId>> exclusion_pairs =
      world.GetExclusionPairs(/*filter_empty_collision_geometry=*/false);
  std::sort(exclusion_pairs.begin(), exclusion_pairs.end());
  for (const auto& pair : exclusion_pairs) {
    if (pair.second < pair.first) {
      continue;
    }
    *output_stream << "\tObject " << pair.first << "('"
                   << world.GetLocalNameForEntityById(pair.first)
                   << "') and Object " << pair.second << "('"
                   << world.GetLocalNameForEntityById(pair.second) << "').\n";
  }

  *output_stream << "Collisions:" << std::endl;
  PrintCollisionsTo(world, output_stream);

  *output_stream << "Robot states:\n"
                 << RobotStatesToString(world) << std::endl;

  *output_stream << "PPR Components:" << std::endl;
  for (const auto& id : GetTypedEntityIdsSorted<PPRComponentType>(world)) {
    const auto* ppr = world.GetComponentByEntityId<PPRComponent>(id).value();

    *output_stream << " id: " << id.value() << "("
                   << world.GetLocalNameForEntityById(id) << ") ppr { ";
    if (auto resource_name = ppr->ResourceName(); resource_name.has_value()) {
      *output_stream << "resource_name: '" << resource_name.value() << "' ";
    }
    *output_stream << "}\n";
  }

  // Add a new line at the end to create a break.
  *output_stream << std::endl;
}

void PrintPhysicalTreeTo(const World& world, std::ostream* output_stream) {
  PrintPhysicalTreeTo(world, kRootEntityId, /*prefix=*/"", output_stream);
  *output_stream << std::endl;
}

void PrintCollisionsTo(
    const World& world, std::ostream* output_stream,
    std::function<std::string(const World& world, const EntityId& entity_id)>
        entity_id_to_string) {
  // {} defaults to all entities.
  return PrintCollisionsForSpecifiedObjectsTo(world, {}, output_stream,
                                              entity_id_to_string);
}

void PrintCollisionsForSpecifiedObjectsTo(
    const World& world, const WorldHashSet<PhysicalEntityId>& objects,
    std::ostream* output_stream,
    std::function<std::string(const World& world, const EntityId& entity_id)>
        entity_id_to_string) {
  if (entity_id_to_string == nullptr) {
    entity_id_to_string = [](const World& world, const EntityId& entity_id) {
      ASSIGN_OR_DIE(const auto* ent, world.GetEntityById(entity_id));
      return ent->GetLocalName();
    };
  }

  // {} defaults to all entities.
  auto collisions = GetCollisionsBetweenSets(
      world, objects, {}, /*check_upper_triangle_only=*/objects.empty());

  *output_stream << "Total collisions: " << collisions.size() << std::endl;
  for (const auto& [obj_id1, obj_id2] : collisions) {
    *output_stream << absl::StrFormat("Collision between '%s' and '%s'.",
                                      entity_id_to_string(world, obj_id1),
                                      entity_id_to_string(world, obj_id2))
                   << std::endl;
  }
}

}  // namespace intrinsic
