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

#include "intrinsic/world/util/entity_search_util.h"

#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/proto/entity_search.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace {

using intrinsic_proto::world::EntitySearchCriteria;

template <typename ComponentType>
bool KeepEntity(const World& world, const EntityId& id,
                EntitySearchCriteria::ByComponents::ComponentState state) {
  if (state ==
      EntitySearchCriteria::ByComponents::COMPONENT_STATE_UNSPECIFIED) {
    return true;
  }

  bool has_component = world.ValidateEntity<ComponentType>(id).ok();
  if (state == EntitySearchCriteria::ByComponents::PRESENT) {
    return has_component;
  } else {
    return !has_component;
  }
}

absl::StatusOr<WorldHashSet<EntityId>> GetEntitiesImpl(
    const World& world, const EntitySearchCriteria& criteria, bool allow_zero,
    bool allow_multiple) {
  WorldHashSet<EntityId> ret;
  switch (criteria.search_type_case()) {
    case EntitySearchCriteria::kById: {
      EntityId entity_id(criteria.by_id().entity_id());
      if (entity_id == kInvalidEntityId) {
        return ::intrinsic::InvalidArgumentErrorBuilder().EmitStackTrace()
               << "Got an invalid entity id";
      }
      if (!world.HasEntity(entity_id)) {
        return intrinsic::NotFoundErrorBuilder().EmitStackTrace()
               << "Could not find entity with id: " << entity_id;
      }
      ret.insert(EntityId(criteria.by_id().entity_id()));
      break;
    }
    case EntitySearchCriteria::kByPath: {
      std::vector<std::string> search_names(criteria.by_path().names().begin(),
                                            criteria.by_path().names().end());
      WorldHashSet<AttachmentEntityId> attachment_entity_ids;
      if (criteria.by_path().can_skip()) {
        attachment_entity_ids =
            world.FindByLocalNames(kRootEntityId, search_names);
      } else {
        attachment_entity_ids =
            world.FindByExactLocalNames(kRootEntityId, search_names);
      }
      for (const AttachmentEntityId id : attachment_entity_ids) {
        ret.insert(EntityId(id.value()));
      }
      break;
    }
    case EntitySearchCriteria::kByAlias: {
      INTR_ASSIGN_OR_RETURN(EntityId id,
                            world.FindByAlias(criteria.by_alias().alias()));
      ret.insert(id);
      break;
    }
    case EntitySearchCriteria::kByLabels: {
      WorldHashSet<LabelId> labels;
      labels.reserve(criteria.by_labels().label_ids_size());
      for (const auto& label : criteria.by_labels().label_ids()) {
        labels.emplace(label);
      }

      auto results = world.GetEntitiesWithAllLabels(
          labels, criteria.by_labels().entity_name());
      ret.insert(results.begin(), results.end());
      break;
    }
    case EntitySearchCriteria::kByCollectionPath: {
      const auto& proto = criteria.by_collection_path();
      std::vector<std::string> path(proto.collection_names().begin(),
                                    proto.collection_names().end());
      WorldHashSet<AttachmentEntityId> attachment_entity_ids =
          world.FindByNameAndCollectionPath(proto.entity_name(), path);
      for (const AttachmentEntityId id : attachment_entity_ids) {
        ret.insert(EntityId(id.value()));
      }
      break;
    }
    case EntitySearchCriteria::kByComponents: {
      const auto& proto = criteria.by_components();
      const auto entities = world.GetEntityIds();
      for (const auto& entity_id : entities) {
        if (!KeepEntity<CollectionsComponentType>(
                world, entity_id, proto.collections_component())) {
          continue;
        }
        if (!KeepEntity<CollectionsMemberComponentType>(
                world, entity_id, proto.collections_member_component())) {
          continue;
        }
        if (!KeepEntity<RobotComponentType>(world, entity_id,
                                            proto.robot_component())) {
          continue;
        }

        ret.insert(entity_id);
      }
      break;
    }
    case EntitySearchCriteria::kByAnd: {
      const auto& proto = criteria.by_and();
      if (proto.criteria_size() == 0) {
        return ::intrinsic::InvalidArgumentErrorBuilder().EmitStackTrace()
               << "Empty ByAnd criteria";
      }

      // Go through each extra criteria and get the results.
      std::vector<WorldHashSet<EntityId>> sub_results;
      for (const auto& sub_criteria : proto.criteria()) {
        INTR_ASSIGN_OR_RETURN(
            WorldHashSet<EntityId> sub_entities,
            GetEntitiesImpl(world, sub_criteria, /*allow_zero=*/allow_zero,
                            /*allow_multiple=*/true));
        sub_results.emplace_back(std::move(sub_entities));
      }

      // Perform the logical AND with the results.
      for (int i = 1; i < sub_results.size(); ++i) {
        sub_results[0] = [&sub_results, i]() -> WorldHashSet<EntityId> {
          WorldHashSet<EntityId> ret;
          for (const auto& sub_result_item : sub_results[i]) {
            if (!sub_results[0].contains(sub_result_item)) {
              continue;
            }
            ret.insert(sub_result_item);
          }
          return ret;
        }();
      }

      ret = std::move(sub_results[0]);
      break;
    }
    case EntitySearchCriteria::kByOr: {
      const auto& proto = criteria.by_or();
      if (proto.criteria_size() == 0) {
        return ::intrinsic::InvalidArgumentErrorBuilder().EmitStackTrace()
               << "Empty ByOr criteria";
      }

      // Go through each extra criteria and get the results combining in the
      // process. This effectively performs the logical OR operation.
      for (const auto& sub_criteria : proto.criteria()) {
        INTR_ASSIGN_OR_RETURN(
            WorldHashSet<EntityId> sub_entities,
            GetEntitiesImpl(world, sub_criteria, /*allow_zero=*/true,
                            /*allow_multiple=*/allow_multiple));
        ret.insert(sub_entities.begin(), sub_entities.end());
      }

      break;
    }
    case EntitySearchCriteria::SEARCH_TYPE_NOT_SET: {
      return ::intrinsic::InvalidArgumentErrorBuilder().EmitStackTrace()
             << "Unknown entity search criteria";
    }
  }

  if (!allow_zero && ret.empty()) {
    return intrinsic::NotFoundErrorBuilder().EmitStackTrace()
           << "found no entities matching criteria: " << absl::StrCat(criteria);
  }

  if (!allow_multiple && ret.size() > 1) {
    return ::intrinsic::InvalidArgumentErrorBuilder().EmitStackTrace()
           << "Found multiple matching entities ("
           << absl::StrJoin(ret, ", ", absl::StreamFormatter())
           << "), expected exactly 1. Criteria: " << absl::StrCat(criteria);
  }

  return ret;
}

}  // namespace

absl::StatusOr<WorldHashSet<EntityId>> GetEntities(
    const World& world, const EntitySearchCriteria& criteria,
    bool allow_multiple) {
  return GetEntitiesImpl(world, criteria, /*allow_zero=*/false, allow_multiple);
}

absl::StatusOr<EntityId> GetSingleEntity(const World& world,
                                         const EntitySearchCriteria& criteria) {
  INTR_ASSIGN_OR_RETURN(WorldHashSet<EntityId> entities,
                        GetEntitiesImpl(world, criteria, /*allow_zero=*/false,
                                        /*allow_multiple=*/false));
  return *entities.begin();
}

EntitySearchCriteria CreatePathSearchCriteria(
    const std::vector<std::string>& local_names, bool can_skip) {
  EntitySearchCriteria search_criteria;
  auto* by_path = search_criteria.mutable_by_path();
  *by_path->mutable_names() = {local_names.begin(), local_names.end()};
  by_path->set_can_skip(can_skip);
  return search_criteria;
}

}  // namespace intrinsic
