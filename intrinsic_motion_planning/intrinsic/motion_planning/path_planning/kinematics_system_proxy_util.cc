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

#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy_util.h"

#include <cstddef>
#include <future>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/motion_planning/path_planning/concurrent_proxy.h"
#include "intrinsic/motion_planning/path_planning/dof_view_proxy.h"
#include "intrinsic/motion_planning/path_planning/kinematics_system_proxy.h"
#include "intrinsic/motion_planning/path_planning/path_planner_definitions.h"
#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/path_planning/world_kinematics_system_proxy.h"
#include "intrinsic/motion_planning/proto/motion_specification_proto_utils.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/configuration_validation.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

namespace {
absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>>
CreateConcurrentKinematicsProxy(
    int thread_count, const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    std::optional<
        intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
        constraints_proto,
    const intrinsic_proto::RuleSet& rule_set, bool disable_collision_checking) {
  if (thread_count < 1) {
    return absl::InvalidArgumentError("thread_count must be > 0!");
  }

  auto create_single_proxy = [&object_world, &robot, &collision_checker_config,
                              &constraints_proto, &rule_set,
                              disable_collision_checking]()
      -> absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> {
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<KinematicsSystemProxy> proxy,
        CreateKinematicsProxyWithConfig(
            object_world, robot, collision_checker_config, constraints_proto,
            rule_set, disable_collision_checking));
    // We disable collision check statistics for concurrent proxies to avoid the
    // possibility of multiple collision checkers trying to accumulate the same
    // statistics variable.
    proxy->SetDistanceCheckStatistics(nullptr);
    return proxy;
  };

  std::vector<std::unique_ptr<KinematicsSystemProxy>> proxies;
  proxies.reserve(thread_count);

  if (thread_count == 1) {
    INTR_ASSIGN_OR_RETURN(std::unique_ptr<KinematicsSystemProxy> proxy,
                          create_single_proxy());
    proxies.push_back(std::move(proxy));
  } else {
    std::vector<
        std::future<absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>>>>
        futures;
    futures.reserve(thread_count);
    for (int ii = 0; ii < thread_count; ++ii) {
      futures.push_back(std::async(std::launch::async, create_single_proxy));
    }

    for (auto& future : futures) {
      INTR_ASSIGN_OR_RETURN(std::unique_ptr<KinematicsSystemProxy> proxy,
                            future.get());
      proxies.push_back(std::move(proxy));
    }
  }

  return std::make_unique<ConcurrentProxy>(std::move(proxies));
}

intrinsic_proto::RuleSet RelaxCollisionMargins(
    const double relative_factor, const double absolute_factor,
    const intrinsic_proto::RuleSet& rule_set) {
  intrinsic_proto::RuleSet relaxed_rule_set = rule_set;
  for (intrinsic_proto::Rule& rule : *relaxed_rule_set.mutable_rules()) {
    if (rule.action().has_margin()) {
      const double initial_margin = rule.action().margin().hard_margin();
      const double relative_relaxed = relative_factor * initial_margin;
      const double absolute_relaxed =
          std::max(0.0, initial_margin - absolute_factor);
      const double relaxed_margin =
          std::min(relative_relaxed, absolute_relaxed);
      rule.mutable_action()->mutable_margin()->set_hard_margin(relaxed_margin);
    }
  }
  return relaxed_rule_set;
}
}  // namespace

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>>
CreateKinematicsProxyWithConfig(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    std::optional<
        intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
        constraints_proto,
    const intrinsic_proto::RuleSet& rule_set, bool disable_collision_checking,
    std::optional<int> maybe_concurrent_thread_count) {
  if (maybe_concurrent_thread_count.has_value()) {
    return CreateConcurrentKinematicsProxy(
        *maybe_concurrent_thread_count, object_world, robot,
        collision_checker_config, constraints_proto, rule_set,
        disable_collision_checking);
  }

  const World& world = object_world.GetEntityWorld();

  std::vector<std::unique_ptr<ConstraintInterface>> constraints;
  if (constraints_proto.has_value()) {
    INTR_ASSIGN_OR_RETURN(constraints,
                          GetUniformGeometricConstraintsFromProto(
                              object_world, constraints_proto.value()));
  }

  return CreateKinematicsProxyWithConfig(
      world, robot.GetRobotEntityId(), collision_checker_config,
      std::move(constraints), rule_set, disable_collision_checking);
}

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> CreateKinematicsProxy(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    std::optional<
        intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
        constraints_proto,
    const intrinsic_proto::RuleSet& rule_set, bool disable_collision_checking,
    std::optional<int> maybe_concurrent_thread_count) {
  return CreateKinematicsProxyWithConfig(
      object_world, robot, /*collision_checker_config=*/{}, constraints_proto,
      rule_set, disable_collision_checking, maybe_concurrent_thread_count);
}

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> CreateKinematicsProxy(
    const World& world, const WorldObjectName& robot_name,
    std::optional<
        intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
        constraints_proto,
    const intrinsic_proto::RuleSet& rule_set, bool disable_collision_checking) {
  INTR_ASSIGN_OR_RETURN(auto object_world,
                        object_world::ObjectWorld::CreateView(world));
  INTR_ASSIGN_OR_RETURN(auto* robot,
                        object_world->GetKinematicObject(robot_name));
  return CreateKinematicsProxy(*object_world, *robot, constraints_proto,
                               rule_set, disable_collision_checking);
}

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>>
CreateKinematicsProxyWithConfig(
    const World& world, RobotCollectionsEntityId robot_id,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    std::vector<std::unique_ptr<ConstraintInterface>> constraints,
    const intrinsic_proto::RuleSet& rule_set, bool disable_collision_checking) {
  INTR_ASSIGN_OR_RETURN(const bool is_kinematic_chain,
                        world.IsChainKinematicChain(robot_id));
  if (!is_kinematic_chain) {
    return DofViewProxy::Create(world, robot_id, std::move(constraints),
                                rule_set, collision_checker_config,
                                disable_collision_checking);
  }

  INTR_ASSIGN_OR_RETURN(LinkEntityId base_link_id, world.GetBaseLink(robot_id));

  INTR_ASSIGN_OR_RETURN(AttachmentEntityId tip_id,
                        world.GetFinalEntityOfRobotKinematicChain(robot_id));

  return WorldKinematicsSystemProxy::Create(
      world, PhysicalEntityId(base_link_id), PhysicalEntityId(tip_id.value()),
      kUseVariableLimits, std::move(constraints), rule_set,
      collision_checker_config, disable_collision_checking);
}

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> CreateKinematicsProxy(
    const World& world, RobotCollectionsEntityId robot_id,
    std::vector<std::unique_ptr<ConstraintInterface>> constraints,
    const intrinsic_proto::RuleSet& rule_set, bool disable_collision_checking) {
  return CreateKinematicsProxyWithConfig(
      world, robot_id, /*collision_checker_config=*/{}, std::move(constraints),
      rule_set, disable_collision_checking);
}

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>>
CreateKinematicsProxyWithConfig(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot, const PathSegment& path_segment,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    std::optional<
        intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
        constraints_proto) {
  if (!path_segment.collision_rule_set.has_value()) {
    return CreateKinematicsProxyWithConfig(
        object_world, robot, collision_checker_config, constraints_proto,
        /*rule_set=*/intrinsic_proto::RuleSet(),
        /*disable_collision_checking=*/true);
  }
  intrinsic_proto::RuleSet rule_set_proto;
  rule_set_proto.mutable_rules()->Assign(
      path_segment.collision_rule_set.value().begin(),
      path_segment.collision_rule_set.value().end());
  return CreateKinematicsProxyWithConfig(object_world, robot,
                                         collision_checker_config,
                                         constraints_proto, rule_set_proto,
                                         /*disable_collision_checking=*/false);
}

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>> CreateKinematicsProxy(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot, const PathSegment& path_segment,
    std::optional<
        intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
        constraints_proto) {
  return CreateKinematicsProxyWithConfig(object_world, robot, path_segment,
                                         /*collision_checker_config=*/{},
                                         std::move(constraints_proto));
}

ConstraintManifoldProjector CreateConstraintManifoldProjector(
    const KinematicsSystemProxy& proxy, size_t max_projection_steps,
    double max_constraint_error_norm) {
  return [&proxy, max_projection_steps,
          max_constraint_error_norm](const eigenmath::VectorXd& configuration) {
    return proxy.ProjectOntoConstraintManifolds(
        configuration, max_projection_steps, max_constraint_error_norm);
  };
}

absl::StatusOr<std::unique_ptr<KinematicsSystemProxy>>
CreateProxyWithRelaxedMargins(
    const object_world::ObjectWorld& object_world,
    const object_world::KinematicObject& robot,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    std::optional<
        intrinsic_proto::motion_planning::v1::UniformGeometricConstraint>
        constraints_proto,
    const intrinsic_proto::RuleSet& rule_set, bool disable_collision_checking,
    const double relative_factor, const double absolute_factor,
    std::optional<int> maybe_concurrent_thread_count) {
  INTR_ASSIGN_OR_RETURN(const intrinsic_proto::RuleSet world_rule_set,
                        object_world.GetDefaultCollisionSettings());

  const intrinsic_proto::RuleSet world_rule_set_with_relaxed_margins =
      RelaxCollisionMargins(relative_factor, absolute_factor, world_rule_set);

  World world_with_relaxed_margins = object_world.GetEntityWorld().Clone();
  INTR_RETURN_IF_ERROR(world_with_relaxed_margins.SetDefaultRuleSet(
      world_rule_set_with_relaxed_margins));

  // TODO(b/479881290): can we avoid creating this ObjectWorld here?
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<const object_world::ObjectWorld>
          object_world_with_relaxed_margins,
      object_world::ObjectWorld::CreateView(world_with_relaxed_margins));

  const intrinsic_proto::RuleSet motion_rule_set_with_relaxed_margins =
      RelaxCollisionMargins(relative_factor, absolute_factor, rule_set);

  // TODO(b/470440149): the World will get cloned again inside
  // CreateKinematicsProxy. Maybe try to avoid the extra clone?
  return CreateKinematicsProxyWithConfig(
      *object_world_with_relaxed_margins, robot, collision_checker_config,
      constraints_proto, motion_rule_set_with_relaxed_margins,
      /*disable_collision_checking=*/disable_collision_checking,
      maybe_concurrent_thread_count);
}

}  // namespace intrinsic
