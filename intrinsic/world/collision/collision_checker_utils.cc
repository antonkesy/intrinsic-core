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

#include "intrinsic/world/collision/collision_checker_utils.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "google/protobuf/message.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/affine_transform_of_geometry.h"
#include "intrinsic/geometry/api/distance_stats.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/macros.h"
#include "intrinsic/util/proto/pb_hash.h"
#include "intrinsic/world/collision/collision_checker_world.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/collision/collision_types.h"
#include "intrinsic/world/collision/rule_matching.h"
#include "intrinsic/world/collision/util/rule_set_util.h"
#include "intrinsic/world/component/collision_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/collision_action.pb.h"
#include "third_party/simple_lru_cache/simple_lru_cache_inl.h"

ABSL_FLAG(bool, use_global_collision_checking_stats, true,
          "If set, we will use a global accumulator for collision stats "
          "instead of a local per checker accumulator. This also means that "
          "when stats are printed, all the accumulated stats will print.");

ABSL_FLAG(absl::Duration, threshold_for_slow_collision_checks,
          absl::Microseconds(300),
          "Any collision check that takes longer than this will be added to "
          "the distance stats as a 'slow' check.");

namespace intrinsic {

namespace {

// Select an action such that action1 is returned unless action1 is not an
// exclusion and action2 is an exclusion action.
//
// Object ids are for logging only.
intrinsic_proto::world::CollisionAction SelectAction(
    const intrinsic_proto::world::CollisionAction& action1,
    const intrinsic_proto::world::CollisionAction& action2,
    const PhysicalEntityIdPair& objs) {
  if (action1.has_is_excluded() && action1.is_excluded()) {
    return action1;
  }

  if (action2.has_is_excluded() && action2.is_excluded()) {
    return action2;
  }

  // Check if it contains a exclude = false action and ignore in that case.
  // Conflicting actions would have been resolved above.
  if (action1.has_is_excluded() && !action1.is_excluded()) {
    VLOG(1) << "Ignoring is_excluded=false for action1: "
            << google::protobuf::ShortFormat(action1);
    return action2;
  }

  if (action2.has_is_excluded() && !action2.is_excluded()) {
    VLOG(1) << "Ignoring is_excluded=false for action2: "
            << google::protobuf::ShortFormat(action2);
    return action1;
  }

  // If the two rules are the same, we will return first without printing the
  // warning below. This can happen when a rule is symmetric, i.e., it matches
  // the same pair of objects in either direction ('*' x '*').
  if (action1 == action2) {
    return action1;
  }

  VLOG(1) << "An action has already been assigned between " << objs
          << ", but another action tries to override it. Use -v=2 to see "
             "the rules.";
  VLOG(2) << "Ignoring action=" << google::protobuf::ShortFormat(action2)
          << " over action=" << google::protobuf::ShortFormat(action1);
  return action1;
}
}  // namespace

std::multiset<intrinsic_proto::Rule> FillRuleSet(
    const CollisionCheckerWorld& world,
    const intrinsic_proto::RuleSet& rule_set) {
  std::multiset<intrinsic_proto::Rule> result(rule_set.rules().begin(),
                                              rule_set.rules().end());

  // Add exclusion pairs to the rule_set.
  const auto exclusion_pairs =
      world.GetExclusionPairs(/*filter_empty_collision_geometry=*/true);
  intrinsic_proto::world::CollisionAction is_excluded_action;
  is_excluded_action.set_is_excluded(true);
  for (const auto& object_exclusion : exclusion_pairs) {
    intrinsic_proto::Rule rule;
    rule.add_id_1(object_exclusion.first.value());
    rule.add_id_2(object_exclusion.second.value());
    *(rule.mutable_action()) = is_excluded_action;
    result.insert(rule);
  }

  return result;
}

CollisionCheckerCommonData GetCollisionCheckerData(
    const WorldHashSet<PhysicalEntityId>& dynamic_object_ids,
    const CollisionCheckerWorld& world,
    const intrinsic_proto::RuleSet& rule_set) {
  CollisionCheckerCommonData data;
  data.dynamic_object_ids = dynamic_object_ids;

  data.rule_set = FillRuleSet(world, rule_set);

  std::vector<PhysicalEntityId> all_object_ids;
  for (const auto& entity_id : world.GetEntityIds()) {
    ASSIGN_OR_DIE(const auto entity, world.GetEntityById(entity_id));
    if (entity->IsEntityValid<PhysicalEntityId>()) {
      all_object_ids.emplace_back(entity_id);
    }
  }

  // Remove kRootObjectId.
  auto root_entity_iter =
      std::find(all_object_ids.begin(), all_object_ids.end(), kRootEntityId);
  if (root_entity_iter != all_object_ids.end()) {
    all_object_ids.erase(root_entity_iter);
  } else {
    LOG(FATAL) << "We expected the root id";
  }

  for (const auto& id : all_object_ids) {
    if (data.dynamic_object_ids.contains(id)) {
      continue;
    }
    data.static_object_ids.insert(id);
  }

  // TODO(keegang): Looks like we stopped using labels from the world. Since we
  // never use, any special rule set, it doesn't really matter...
  // should retrieve label map using world_.GetLabelsMap();
  for (const auto obj_id : all_object_ids) {
    ASSIGN_OR_DIE(auto entity, world.GetEntityById(obj_id));

    // Remove any objects from the dynamic/static sets that are marked to have
    // no collision response.
    if (entity->HasComponent<CollisionComponent>()) {
      ASSIGN_OR_DIE(auto* collision,
                    entity->GetComponent<CollisionComponent>());
      if (!collision->HasCollisionResponse()) {
        data.dynamic_object_ids.erase(obj_id);
        data.static_object_ids.erase(obj_id);
        continue;
      }
    }
  }

  return data;
}

WorldHashMap<PhysicalEntityIdPair, intrinsic_proto::world::CollisionAction>
GetActionMap(const std::multiset<intrinsic_proto::Rule>& rules,
             const WorldHashSet<PhysicalEntityId>& set1,
             const WorldHashSet<PhysicalEntityId>& set2) {
  WorldHashMap<PhysicalEntityIdPair, intrinsic_proto::world::CollisionAction>
      action_map;
  for (const auto& rule : rules) {
    auto matches = FindMatchingObjects(rule, set1, set2);
    if (matches.empty()) {
      // No objects matched so nothing to add.
      continue;
    }

    for (const auto& match : matches) {
      auto it = action_map.find(match);
      // We need to duplicates the entry such that {a,b} and {b,a} pair can be
      // found in the map.
      if (it != action_map.end()) {
        auto action = SelectAction(it->second, rule.action(), match);
        action_map[match] = action;
        action_map[std::make_pair(match.second, match.first)] = action;

      } else {
        action_map[match] = rule.action();
        action_map[std::make_pair(match.second, match.first)] = rule.action();
      }
    }
  }
  return action_map;
}

std::string GetLocalNameForEntityById(const CollisionCheckerWorld& world,
                                      EntityId id) {
  auto entity_or_status = world.GetEntityById(id);
  if (entity_or_status.ok()) {
    return entity_or_status.value()->GetLocalName();
  }
  return absl::StrCat("InvalidId(", id.value(), ")");
}

std::string GetLocalNamePathForEntityById(const CollisionCheckerWorld& world,
                                          EntityId id) {
  auto path_string =
      world.GetLocalNamePathString(AttachmentEntityId(id.value()), "::");
  if (path_string.ok()) {
    return std::move(path_string).value();
  }

  auto entity = world.GetEntityById(id);
  if (entity.ok()) {
    return absl::StrCat("??::", entity.value()->GetLocalName());
  }

  return absl::StrCat("InvalidId(", id.value(), ")");
}

namespace collision_details {

void AggregateStats(const CollisionStats& s, CollisionStats* stats) {
  stats->checks_clear += s.checks_clear;
  stats->time_clear += s.time_clear;
  stats->checks_invalid += s.checks_invalid;
  stats->time_invalid += s.time_invalid;
  stats->cache_hits += s.cache_hits;
  stats->cache_misses += s.cache_misses;
}

void UpdateStats(const MarginPairConflictStatus& status, absl::Duration time,
                 bool cache_hit, CollisionStats* stats) {
  if (cache_hit) {
    stats->cache_hits++;
  } else {
    stats->cache_misses++;
  }
  if (status == MarginPairConflictStatus::kClear) {
    stats->checks_clear++;
    stats->time_clear += time;
  } else if (status == MarginPairConflictStatus::kInvalid) {
    stats->checks_invalid++;
    stats->time_invalid += time;
  } else {
    LOG(FATAL) << "Unexpected conflict status";
  }
}

std::string ToString(const CollisionStats& cs) {
  int checks_total = cs.checks_clear + cs.checks_invalid;
  double time_clear = absl::ToDoubleSeconds(cs.time_clear);
  double time_invalid = absl::ToDoubleSeconds(cs.time_invalid);
  double time_total = absl::ToDoubleSeconds(cs.time_clear + cs.time_invalid);
  std::string avg_clear =
      cs.checks_clear > 0 ? absl::StrCat(time_clear / cs.checks_clear) : "n/a";
  std::string avg_invalid = cs.checks_invalid > 0
                                ? absl::StrCat(time_invalid / cs.checks_invalid)
                                : "n/a";
  std::string avg_total =
      checks_total > 0 ? absl::StrCat(time_total / checks_total) : "n/a";
  return absl::StrFormat(
      "checks: %d (%d,%d) time: %f (%f,%f) avg: %s (%s,%s) cache hits: %f%%",
      checks_total, cs.checks_clear, cs.checks_invalid, time_total, time_clear,
      time_invalid, avg_total, avg_clear, avg_invalid,
      cs.cache_hits * 100.0 / checks_total);
}

std::pair<EntityId, EntityId> MakeOrderedPair(const EntityId& a,
                                              const EntityId& b) {
  if (a < b) {
    return std::make_pair(a, b);
  } else {
    return std::make_pair(b, a);
  }
}

void LogCollisionCheckingDebugStats(
    const CollisionCheckerWorld& world,
    const DistanceCheckStatistics& distance_stats) {
  auto sorted_distance_stats =
      SortMap<std::pair<EntityId, EntityId>>(distance_stats.time_by_pair);
  std::string extra_header;

  // Only print the last few items unless we have ABSL_VLOG_IS_ON(2)
  static constexpr int kMaxItemsToPrint = 15;
  if (!ABSL_VLOG_IS_ON(2) && sorted_distance_stats.size() > kMaxItemsToPrint) {
    extra_header = absl::StrCat("[truncated to ", kMaxItemsToPrint, "]");
    sorted_distance_stats.erase(sorted_distance_stats.begin(),
                                sorted_distance_stats.end() - kMaxItemsToPrint);
  }

  LOG(INFO) << "Printing individual stats["
            << distance_stats.time_by_pair.size() << "]" << extra_header
            << ": ";
  for (const auto& p : sorted_distance_stats) {
    LOG(INFO) << "(" << p.first.first.value() << "," << p.first.second.value()
              << "): " << ToString(p.second);
  }

  CollisionStats total;
  for (const auto& p : distance_stats.time_by_pair) {
    AggregateStats(p.second, &total);
  }

  LOG(INFO) << "Total: " << ToString(total);

  if (distance_stats.fallback_counter > 0) {
    const double total_seconds =
        absl::ToDoubleSeconds(distance_stats.fallback_time_total);
    LOG(INFO) << "There were " << distance_stats.fallback_counter
              << " fallback collision checks performated with a total time of "
              << total_seconds << "s. This is an average of "
              << (total_seconds / distance_stats.fallback_counter);
  } else {
    LOG(INFO) << "There were no fallback collision checks performed.";
  }

  auto threshold = absl::GetFlag(FLAGS_threshold_for_slow_collision_checks);
  std::vector<std::tuple<EntityId, EntityId, SlowCheckStats::Data>>
      sorted_slow_checks;
  if (threshold > absl::ZeroDuration()) {
    for (const auto& [key, stats] : distance_stats.slow_checks) {
      if (stats.in_collision.duration >= threshold) {
        sorted_slow_checks.emplace_back(key.first, key.second,
                                        stats.in_collision);
      }
      if (stats.not_in_collision.duration >= threshold) {
        sorted_slow_checks.emplace_back(key.first, key.second,
                                        stats.not_in_collision);
      }
    }
    std::sort(sorted_slow_checks.begin(), sorted_slow_checks.end(),
              [](const auto& lhs, const auto& rhs) {
                return std::get<2>(lhs).duration < std::get<2>(rhs).duration;
              });
  }

  LOG(INFO) << "Detected " << sorted_slow_checks.size()
            << " slow checks[>=" << absl::FormatDuration(threshold)
            << "][worst case per pair]:";
  const auto id_to_name = ABSL_VLOG_IS_ON(2) ? GetLocalNamePathForEntityById
                                             : GetLocalNameForEntityById;

  for (const auto& [left_id, right_id, stats] : sorted_slow_checks) {
    std::string left_name =
        absl::StrCat(id_to_name(world, left_id), "[", left_id.value(), "]");
    std::string right_name =
        absl::StrCat(id_to_name(world, right_id), "[", right_id.value(), "]");

    LOG(INFO) << "\t" << left_name << " vs " << right_name
              << " [m:" << stats.margin << ", r:" << stats.result << ", p:{"
              << toString(stats.transform) << "}]: " << stats.duration;
  }
}

bool EntityHasPointCloud(const CollisionCheckerWorld& world,
                         const PhysicalEntityId entity_id) {
  NamedGeometrySet geos =
      world.GetGeometryForEntity(entity_id, kKindCollisionGeometry)
          .value_or(NamedGeometrySet());
  for (const auto& [_, transformed_geo] : geos) {
    if (transformed_geo.shape().GetExactGeometry().HasPointCloud()) {
      return true;
    }
  }
  return false;
}

}  // namespace collision_details

}  // namespace intrinsic
