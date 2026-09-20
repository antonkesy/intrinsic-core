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

#include "intrinsic/world/component/robot_component.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/wrappers.pb.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/proto/cart_space.pb.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/util/aggregate_type.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/proto/generic_action.pb.h"
#include "intrinsic/world/proto/robot_component.pb.h"
#include "intrinsic/world/robot_payload/robot_payload.h"

namespace intrinsic {
namespace {

using IconSimDevice = intrinsic_proto::world::RobotComponent::IconSimDevice;
using IconSimPluginSpec =
    intrinsic_proto::world::RobotComponent::IconSimPluginSpec;
using MultiCameraPluginSpec =
    intrinsic_proto::world::RobotComponent::MultiCameraPluginSpec;
using GenericActionPluginSpec =
    intrinsic_proto::world::generic_action::GenericActionPluginSpec;

class RobotComponentImpl : public RobotComponent {
 public:
  RobotComponentImpl() = default;
  explicit RobotComponentImpl(
      WorldHashMap<AttachmentEntityId,
                   WorldHashMap<AttachmentEntityId, std::string>>
          ik_solvers,
      WorldHashMap<std::string, eigenmath::VectorXd> named_configurations,
      std::vector<IconSimDevice> icon_sim_devices,
      std::optional<IconSimPluginSpec> icon_sim_plugin_spec,
      std::optional<MultiCameraPluginSpec> multi_camera_plugin_spec,
      std::optional<GenericActionPluginSpec> generic_action_plugin_spec,
      CartesianLimits cartesian_limits,
      std::optional<RobotPayload> mounted_payload);

  absl::StatusOr<intrinsic_proto::world::RobotComponent> ToProto()
      const override;
  std::unique_ptr<RobotComponent> Clone() const override;

  absl::StatusOr<std::unique_ptr<DofKinematicView>> CreateDofKinematicView()
      override;
  absl::StatusOr<std::unique_ptr<const DofKinematicView>>
  CreateDofKinematicView() const override;
  WorldHashSet<std::pair<AttachmentEntityId, AttachmentEntityId>>
  GetSolvableFrames() const override;
  absl::StatusOr<std::string> GetSolverKeyForFrames(
      AttachmentEntityId base, AttachmentEntityId tip) const override;

  absl::Status AddSolvableFrames(AttachmentEntityId base,
                                 AttachmentEntityId tip,
                                 absl::string_view solver_key) override;
  absl::Status RemoveSolvableFrames(AttachmentEntityId base,
                                    AttachmentEntityId tip) override;

  absl::StatusOr<eigenmath::VectorXd> GetNamedConfiguration(
      absl::string_view named_configuration) const override;
  WorldHashMap<std::string, eigenmath::VectorXd> GetNamedConfigurations()
      const override;
  void SetNamedConfigurations(WorldHashMap<std::string, eigenmath::VectorXd>&&
                                  named_configuration) override;
  void SetNamedConfiguration(absl::string_view name,
                             eigenmath::VectorXd configuration) override;
  void RemoveNamedConfiguration(absl::string_view name) override;

  absl::StatusOr<std::vector<IconSimDevice>> GetIconSimDevices() const override;
  void SetIconSimDevices(const std::vector<IconSimDevice>& devices) override;
  absl::StatusOr<std::optional<IconSimPluginSpec>> GetIconSimPluginSpec()
      const override;
  void SetIconSimPluginSpec(const IconSimPluginSpec& spec) override;
  void RemoveIconSimPluginSpec() override;

  const CartesianLimits& GetCartesianLimits() const override;
  absl::Status SetCartesianLimits(const CartesianLimits& cart_limits) override;

  void SetMultiCameraPluginSpec(const MultiCameraPluginSpec& spec) override;
  absl::StatusOr<std::optional<MultiCameraPluginSpec>>
  GetMultiCameraPluginSpec() const override;
  void RemoveMultiCameraPluginSpec() override;

  void SetGenericActionPluginSpec(
      std::optional<GenericActionPluginSpec> spec) override;
  absl::StatusOr<std::optional<GenericActionPluginSpec>>
  GetGenericActionPluginSpec() const override;

  absl::Status UpdateFromProto(
      const intrinsic_proto::world::RobotComponent& proto) override;

  absl::Status RekeyIds(
      const WorldHashMap<EntityId, EntityId>& id_mapping) override;

  const std::optional<RobotPayload>& GetMountedPayload() const override;

  std::optional<RobotPayload>& GetMountedPayload() override;

  absl::Status SetMountedPayload(std::optional<RobotPayload> payload) override;

  bool AreKinematicsUpdated() const override;

  void SetAreKinematicsUpdated(bool are_kinematics_updated) override;

 private:
  // 2D map from [base_id][tip_id] to IK solver keys.
  WorldHashMap<AttachmentEntityId,
               WorldHashMap<AttachmentEntityId, std::string>>
      ik_solvers_;
  WorldHashMap<std::string, eigenmath::VectorXd> named_configurations_;
  std::vector<intrinsic_proto::world::RobotComponent::IconSimDevice>
      icon_sim_devices_;
  std::optional<intrinsic_proto::world::RobotComponent::IconSimPluginSpec>
      icon_sim_plugin_spec_;
  std::optional<intrinsic_proto::world::RobotComponent::MultiCameraPluginSpec>
      multi_camera_plugin_spec_;
  std::optional<intrinsic_proto::world::generic_action::GenericActionPluginSpec>
      generic_action_plugin_spec_;
  CartesianLimits cartesian_limits_;
  std::optional<RobotPayload> mounted_payload_;
  bool are_kinematics_updated_ = false;
};

RobotComponentImpl::RobotComponentImpl(
    WorldHashMap<AttachmentEntityId,
                 WorldHashMap<AttachmentEntityId, std::string>>
        ik_solvers,
    WorldHashMap<std::string, eigenmath::VectorXd> named_configurations,
    std::vector<intrinsic_proto::world::RobotComponent::IconSimDevice>
        icon_sim_devices,
    std::optional<intrinsic_proto::world::RobotComponent::IconSimPluginSpec>
        icon_sim_plugin_spec,
    std::optional<intrinsic_proto::world::RobotComponent::MultiCameraPluginSpec>
        multi_camera_plugin_spec,
    std::optional<
        intrinsic_proto::world::generic_action::GenericActionPluginSpec>
        generic_action_plugin_spec,
    CartesianLimits cartesian_limits,
    std::optional<RobotPayload> mounted_payload)
    : ik_solvers_(std::move(ik_solvers)),
      named_configurations_(std::move(named_configurations)),
      icon_sim_devices_(std::move(icon_sim_devices)),
      icon_sim_plugin_spec_(std::move(icon_sim_plugin_spec)),
      multi_camera_plugin_spec_(std::move(multi_camera_plugin_spec)),
      generic_action_plugin_spec_(std::move(generic_action_plugin_spec)),
      cartesian_limits_(std::move(cartesian_limits)),
      mounted_payload_(std::move(mounted_payload)) {}

absl::StatusOr<intrinsic_proto::world::RobotComponent>
RobotComponentImpl::ToProto() const {
  intrinsic_proto::world::RobotComponent proto;
  *proto.mutable_cartesian_limits() =
      intrinsic::icon::ToProto(cartesian_limits_);

  for (const auto& [base, tip_map] : ik_solvers_) {
    for (const auto& [tip, solver] : tip_map) {
      auto* ik_solver_proto = proto.add_ik_solvers();
      ik_solver_proto->set_kinematic_solver_key(solver);
      ik_solver_proto->set_base_uid(base.value());
      ik_solver_proto->set_tip_uid(tip.value());
    }
  }

  for (const auto& [name, configuration] : named_configurations_) {
    auto* named_config_proto = proto.add_named_dof_configurations();
    named_config_proto->set_name(name);
    VectorXdToRepeatedDouble(configuration,
                             named_config_proto->mutable_dof_values());
  }

  // Sort the unordered repeated fields so EqualsProto() can be used in testing.
  std::sort(
      proto.mutable_ik_solvers()->begin(), proto.mutable_ik_solvers()->end(),
      [](const intrinsic_proto::world::RobotComponent::IkSolver& one,
         const intrinsic_proto::world::RobotComponent::IkSolver& two) -> bool {
        if (one.base_uid() != two.base_uid()) {
          return one.base_uid() < two.base_uid();
        }
        return one.tip_uid() < two.tip_uid();
      });

  std::sort(
      proto.mutable_named_dof_configurations()->begin(),
      proto.mutable_named_dof_configurations()->end(),
      [](const intrinsic_proto::world::RobotComponent::NamedDofConfiguration&
             one,
         const intrinsic_proto::world::RobotComponent::NamedDofConfiguration&
             two) -> bool { return one.name() < two.name(); });

  for (const auto& device : icon_sim_devices_) {
    *proto.add_icon_sim_devices() = device;
  }

  if (icon_sim_plugin_spec_) {
    *proto.mutable_icon_sim_plugin() = *icon_sim_plugin_spec_;
  }

  if (multi_camera_plugin_spec_) {
    *proto.mutable_multi_camera_plugin() = *multi_camera_plugin_spec_;
  }

  if (generic_action_plugin_spec_) {
    *proto.mutable_generic_action_plugin() = *generic_action_plugin_spec_;
  }

  if (mounted_payload_) {
    *proto.mutable_mounted_payload() = intrinsic::ToProto(*mounted_payload_);
  }

  if (are_kinematics_updated_) {
    proto.set_are_kinematics_updated(are_kinematics_updated_);
  }

  return proto;
}

std::unique_ptr<RobotComponent> RobotComponentImpl::Clone() const {
  auto ret = std::make_unique<RobotComponentImpl>();
  ret->ik_solvers_ = ik_solvers_;
  ret->named_configurations_ = named_configurations_;
  ret->icon_sim_devices_ = icon_sim_devices_;
  ret->icon_sim_plugin_spec_ = icon_sim_plugin_spec_;
  ret->cartesian_limits_ = cartesian_limits_;
  ret->multi_camera_plugin_spec_ = multi_camera_plugin_spec_;
  ret->generic_action_plugin_spec_ = generic_action_plugin_spec_;
  ret->mounted_payload_ = mounted_payload_;
  ret->are_kinematics_updated_ = are_kinematics_updated_;
  return ret;
}

absl::StatusOr<std::unique_ptr<DofKinematicView>>
RobotComponentImpl::CreateDofKinematicView() {
  LOG(FATAL) << "Not implemented yet (stoyang)";
  return absl::UnimplementedError("Not implemented yet (stoyang)");
}

absl::StatusOr<std::unique_ptr<const DofKinematicView>>
RobotComponentImpl::CreateDofKinematicView() const {
  LOG(FATAL) << "Not implemented yet (stoyang)";
  return absl::UnimplementedError("Not implemented yet (stoyang)");
}

WorldHashSet<std::pair<AttachmentEntityId, AttachmentEntityId>>
RobotComponentImpl::GetSolvableFrames() const {
  WorldHashSet<std::pair<AttachmentEntityId, AttachmentEntityId>> ret;
  for (const auto& [base, tip_map] : ik_solvers_) {
    for (auto tip : std::views::keys(tip_map)) {
      ret.insert(std::make_pair(base, tip));
    }
  }
  return ret;
}

absl::StatusOr<std::string> RobotComponentImpl::GetSolverKeyForFrames(
    AttachmentEntityId base, AttachmentEntityId tip) const {
  auto base_iter = ik_solvers_.find(base);
  if (base_iter != ik_solvers_.end()) {
    const auto& tip_map = base_iter->second;
    auto tip_iter = tip_map.find(tip);
    if (tip_iter != tip_map.end()) {
      return tip_iter->second;
    }
  }
  return intrinsic::NotFoundErrorBuilder()
         << "no solver found from " << base.value() << " to " << tip.value();
}

absl::Status RobotComponentImpl::AddSolvableFrames(
    AttachmentEntityId base, AttachmentEntityId tip,
    absl::string_view solver_key) {
  // A base of kInvalidEntityId is used enable a compatibility mode that was
  // used by EntityRobots
  // (intrinsic/world/aspects/entity_robots.cc;rcl=364923968) to
  // replicate the behavior of the original Robots Aspect implementation.
  if (base == kInvalidEntityId) {
    ik_solvers_[base][tip] = solver_key;
    return absl::OkStatus();
  }

  if (base == tip) {
    return absl::InvalidArgumentError("base and tip cannot be equal");
  }
  ik_solvers_[base][tip] = solver_key;
  return absl::OkStatus();
}

absl::Status RobotComponentImpl::RemoveSolvableFrames(AttachmentEntityId base,
                                                      AttachmentEntityId tip) {
  auto base_iter = ik_solvers_.find(base);
  if (base_iter != ik_solvers_.end()) {
    auto& tip_map = base_iter->second;
    if (tip_map.erase(tip) > 0) {
      return absl::OkStatus();
    }
  }
  return intrinsic::NotFoundErrorBuilder()
         << "no solver found from " << base.value() << " to " << tip.value();
}

absl::StatusOr<eigenmath::VectorXd> RobotComponentImpl::GetNamedConfiguration(
    absl::string_view named_configuration) const {
  auto itr = named_configurations_.find(named_configuration);
  if (itr == named_configurations_.end()) {
    return absl::NotFoundError(absl::StrCat("Unknown named configuration '",
                                            named_configuration, "'"));
  }

  return itr->second;
}

WorldHashMap<std::string, eigenmath::VectorXd>
RobotComponentImpl::GetNamedConfigurations() const {
  return named_configurations_;
}

void RobotComponentImpl::SetNamedConfigurations(
    WorldHashMap<std::string, eigenmath::VectorXd>&& named_configuration) {
  named_configurations_ = named_configuration;
}

void RobotComponentImpl::SetNamedConfiguration(
    absl::string_view name, eigenmath::VectorXd configuration) {
  named_configurations_[name] = configuration;
}

void RobotComponentImpl::RemoveNamedConfiguration(absl::string_view name) {
  named_configurations_.erase(name);
}

absl::StatusOr<WorldHashMap<std::string, eigenmath::VectorXd>>
ParseConfigurations(const intrinsic_proto::world::RobotComponent& proto) {
  WorldHashMap<std::string, eigenmath::VectorXd> named_configurations;
  for (const auto& named_config : proto.named_dof_configurations()) {
    eigenmath::VectorXd configuration =
        RepeatedDoubleToVectorXd(named_config.dof_values());

    if (!named_configurations.emplace(named_config.name(), configuration)
             .second) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "Repeated named configuration in proto: "
             << named_config.name();
    }
  }

  return named_configurations;
}

absl::StatusOr<WorldHashMap<AttachmentEntityId,
                            WorldHashMap<AttachmentEntityId, std::string>>>
ParseIkSolvers(const intrinsic_proto::world::RobotComponent& proto) {
  WorldHashMap<AttachmentEntityId,
               WorldHashMap<AttachmentEntityId, std::string>>
      ik_solvers;
  for (const auto& ik_solver_proto : proto.ik_solvers()) {
    ik_solvers[AttachmentEntityId(ik_solver_proto.base_uid())]
              [AttachmentEntityId(ik_solver_proto.tip_uid())] =
                  ik_solver_proto.kinematic_solver_key();
  }

  return std::move(ik_solvers);
}

absl::Status RobotComponentImpl::UpdateFromProto(
    const intrinsic_proto::world::RobotComponent& proto) {
  INTR_ASSIGN_OR_RETURN(auto named_configurations, ParseConfigurations(proto));
  INTR_ASSIGN_OR_RETURN(auto ik_solvers, ParseIkSolvers(proto));

  CartesianLimits cartesian_limits = CartesianLimits::Unlimited();
  if (proto.has_cartesian_limits()) {
    INTR_ASSIGN_OR_RETURN(cartesian_limits,
                          intrinsic::icon::FromProto(proto.cartesian_limits()));
  }

  std::optional<RobotPayload> mounted_payload = std::nullopt;
  if (proto.has_mounted_payload()) {
    INTR_ASSIGN_OR_RETURN(mounted_payload,
                          intrinsic::FromProto(proto.mounted_payload()));
  }

  // Once we know that both maps have parsed correctly we then update our fields
  ik_solvers_ = ik_solvers;
  named_configurations_ = named_configurations;
  cartesian_limits_ = cartesian_limits;
  mounted_payload_ = mounted_payload;

  if (proto.has_icon_sim_plugin()) {
    icon_sim_plugin_spec_ = proto.icon_sim_plugin();
  }
  if (proto.has_multi_camera_plugin()) {
    multi_camera_plugin_spec_ = proto.multi_camera_plugin();
  }

  if (proto.has_generic_action_plugin()) {
    generic_action_plugin_spec_ = proto.generic_action_plugin();
  }

  are_kinematics_updated_ = proto.are_kinematics_updated();

  return absl::OkStatus();
}

absl::Status RobotComponentImpl::RekeyIds(
    const WorldHashMap<EntityId, EntityId>& id_mapping) {
  // Save old ik_solvers_, then clear to make sure we don't get confused by
  // clashes between old and new IDs.
  WorldHashMap<AttachmentEntityId,
               WorldHashMap<AttachmentEntityId, std::string>>
      ik_solvers_old = ik_solvers_;
  ik_solvers_.clear();
  for (const auto& [base, tip_to_solver_name] : ik_solvers_old) {
    AttachmentEntityId new_base(kInvalidEntityId);
    if (base != kInvalidEntityId) {
      auto new_base_it = id_mapping.find(base);
      if (new_base_it == id_mapping.end()) {
        return absl::NotFoundError(
            absl::StrCat("Can't remap solver base frame ", base.value()));
      }
      new_base = AttachmentEntityId(new_base_it->second);
    }
    for (const auto& [tip, solver_key] : tip_to_solver_name) {
      AttachmentEntityId new_tip(kInvalidEntityId);
      if (tip != kInvalidEntityId) {
        auto new_tip_it = id_mapping.find(tip);
        if (new_tip_it == id_mapping.end()) {
          return absl::NotFoundError(
              absl::StrCat("Can't remap solver tip frame ", tip.value()));
        }
        new_tip = AttachmentEntityId(new_tip_it->second);
      }
      INTR_RETURN_IF_ERROR(AddSolvableFrames(new_base, new_tip, solver_key));
    }
  }

  return absl::OkStatus();
}

absl::StatusOr<std::vector<IconSimDevice>>
RobotComponentImpl::GetIconSimDevices() const {
  return icon_sim_devices_;
}

void RobotComponentImpl::SetIconSimDevices(
    const std::vector<IconSimDevice>& devices) {
  icon_sim_devices_ = devices;
}

absl::StatusOr<std::optional<IconSimPluginSpec>>
RobotComponentImpl::GetIconSimPluginSpec() const {
  return icon_sim_plugin_spec_;
}

void RobotComponentImpl::SetIconSimPluginSpec(const IconSimPluginSpec& spec) {
  icon_sim_plugin_spec_ = spec;
}

void RobotComponentImpl::RemoveIconSimPluginSpec() {
  icon_sim_plugin_spec_ = std::nullopt;
}

const CartesianLimits& RobotComponentImpl::GetCartesianLimits() const {
  return cartesian_limits_;
}

absl::Status RobotComponentImpl::SetCartesianLimits(
    const CartesianLimits& cart_limits) {
  cartesian_limits_ = cart_limits;
  return absl::OkStatus();
}

void RobotComponentImpl::SetMultiCameraPluginSpec(
    const MultiCameraPluginSpec& spec) {
  multi_camera_plugin_spec_ = spec;
}

absl::StatusOr<std::optional<MultiCameraPluginSpec>>
RobotComponentImpl::GetMultiCameraPluginSpec() const {
  return multi_camera_plugin_spec_;
}

void RobotComponentImpl::RemoveMultiCameraPluginSpec() {
  multi_camera_plugin_spec_ = std::nullopt;
}

void RobotComponentImpl::SetGenericActionPluginSpec(
    std::optional<GenericActionPluginSpec> spec) {
  generic_action_plugin_spec_ = std::move(spec);
}

absl::StatusOr<std::optional<GenericActionPluginSpec>>
RobotComponentImpl::GetGenericActionPluginSpec() const {
  return generic_action_plugin_spec_;
}

absl::Status RobotComponentImpl::SetMountedPayload(
    std::optional<RobotPayload> payload) {
  mounted_payload_ = std::move(payload);
  return absl::OkStatus();
}

std::optional<RobotPayload>& RobotComponentImpl::GetMountedPayload() {
  return mounted_payload_;
}

const std::optional<RobotPayload>& RobotComponentImpl::GetMountedPayload()
    const {
  return mounted_payload_;
}

bool RobotComponentImpl::AreKinematicsUpdated() const {
  return are_kinematics_updated_;
}

void RobotComponentImpl::SetAreKinematicsUpdated(bool are_kinematics_updated) {
  are_kinematics_updated_ = are_kinematics_updated;
}

}  // namespace

std::unique_ptr<RobotComponent> RobotComponent::Create() {
  return std::make_unique<RobotComponentImpl>();
}

absl::StatusOr<std::unique_ptr<RobotComponent>> RobotComponent::FromProto(
    const intrinsic_proto::world::RobotComponent& proto) {
  INTR_ASSIGN_OR_RETURN(auto named_configurations, ParseConfigurations(proto));
  INTR_ASSIGN_OR_RETURN(auto ik_solvers, ParseIkSolvers(proto));

  CartesianLimits cartesian_limits;
  if (proto.has_cartesian_limits()) {
    INTR_ASSIGN_OR_RETURN(cartesian_limits,
                          intrinsic::icon::FromProto(proto.cartesian_limits()));
  }

  std::vector<IconSimDevice> icon_sim_devices;
  for (const auto& device : proto.icon_sim_devices()) {
    icon_sim_devices.push_back(device);
  }

  std::optional<IconSimPluginSpec> icon_sim_plugin_spec;
  if (proto.has_icon_sim_plugin()) {
    icon_sim_plugin_spec = proto.icon_sim_plugin();
  }

  std::optional<MultiCameraPluginSpec> multi_camera_plugin_spec;
  if (proto.has_multi_camera_plugin()) {
    multi_camera_plugin_spec = proto.multi_camera_plugin();
  }

  std::optional<GenericActionPluginSpec> generic_action_plugin_spec;
  if (proto.has_generic_action_plugin()) {
    generic_action_plugin_spec = proto.generic_action_plugin();
  }

  std::optional<RobotPayload> mounted_payload_spec;
  if (proto.has_mounted_payload()) {
    INTR_ASSIGN_OR_RETURN(mounted_payload_spec,
                          intrinsic::FromProto(proto.mounted_payload()));
  }

  auto robot_component = std::make_unique<RobotComponentImpl>(
      std::move(ik_solvers), std::move(named_configurations),
      std::move(icon_sim_devices), std::move(icon_sim_plugin_spec),
      std::move(multi_camera_plugin_spec),
      std::move(generic_action_plugin_spec), std::move(cartesian_limits),
      std::move(mounted_payload_spec));

  robot_component->SetAreKinematicsUpdated(proto.are_kinematics_updated());

  return std::move(robot_component);
}

}  // namespace intrinsic
