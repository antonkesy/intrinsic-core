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

#include "intrinsic/simulation/gazebo/plugins/grippers/fixed_joint_gripper_plugin.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "absl/synchronization/mutex.h"
#include "google/protobuf/text_format.h"
#include "gz/sim/Entity.hh"
#include "gz/sim/Types.hh"
#include "intrinsic/hardware/gripper/eoat/gripper_config.pb.h"
#include "intrinsic/simulation/gazebo/plugins/gpio/gpio_service.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/fixed_joint_gripper.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/gpio_gripper_plugin_connection.h"
#include "intrinsic/simulation/gazebo/plugins/grippers/gripper.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "sdf/Element.hh"

namespace intrinsic {
namespace simulation {

using ::gz::sim::Entity;
using ::gz::sim::kNullEntity;
using ::intrinsic_proto::simulation::gazebo::GripperCommand;
using ::intrinsic_proto::simulation::gazebo::GripperStatus;

namespace {
template <typename T>
absl::StatusOr<T> CheckAndLoadFromSDF(const std::string& element_name,
                                      sdf::Element* sdf) {
  auto plugin_name = sdf->GetName();
  if (!sdf->HasElement(element_name)) {
    return InvalidArgumentErrorBuilder()
           << element_name << " element missing for " << plugin_name;
  }

  T value;
  bool success;
  std::tie(value, success) = sdf->Get<T>(element_name, value);
  if (!success) {
    return InvalidArgumentErrorBuilder()
           << element_name << " element specified is wrong type for "
           << plugin_name;
  }

  LOG(INFO) << "Found " << element_name << " element with value " << value
            << " for " << plugin_name;
  return value;
}

constexpr int kVerboseLogIntervalSeconds = 5;

}  // namespace

// FixedJointGripperPlugin utilizes an underlying FixedJointGripper which
// leverages allowed ECM manipulations in Gazebo to implement the
// FixedJointGripper. The plugin System has three execution points in the
// simulation cycle.

FixedJointGripperPlugin::~FixedJointGripperPlugin() {
  DestroyGPIOGripperConnection();
}
void FixedJointGripperPlugin::Configure(
    const ::gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    ::gz::sim::EntityComponentManager& ecm, ::gz::sim::EventManager& eventMgr) {
  plugin_name_ = sdf->Get<std::string>("name");
  parent_entity_ = entity;

  sdf::ElementPtr plugin_sdf = sdf->Clone();
  INTR_RETURN_IF_ERROR(LoadSDF(plugin_sdf.get()))
      .With([plugin_sdf, this](intrinsic::StatusBuilder builder) {
        builder.LogError() << ". Plugin sdf: " << plugin_sdf->ToString("");
        absl::MutexLock lock(initialization_status_mutex_);
        initialization_status_ = builder;
        return;
      });

  if (suction_gripper_config_.has_value()) {
    InitGPIOGripperConnection(*suction_gripper_config_);
  }

  INTR_ASSIGN_OR_RETURN(
      fixed_joint_gripper_,
      FixedJointGripper::Create(entity, plugin_name_, gripper_link_name_,
                                grasp_debounce_time_seconds_, ecm),
      _.LogError().With([this](const absl::Status& status) {
        absl::MutexLock lock(initialization_status_mutex_);
        initialization_status_ = status;
        return;
      }));

  if (!dio_status_topic_.empty()) {
    dio_status_pub_ = node_.Advertise<::gz::msgs::UInt32>(dio_status_topic_);
    if (!dio_status_pub_) {
      absl::MutexLock lock(initialization_status_mutex_);
      initialization_status_ = InternalErrorBuilder().LogError()
                               << "Failed to advertise dio status on topic '"
                               << dio_status_topic_ << "' for " << plugin_name_;
      return;
    }
  }
  if (!status_topic_.empty()) {
    status_pub_ = node_.Advertise<GripperStatus>(status_topic_);
    if (!status_pub_) {
      absl::MutexLock lock(initialization_status_mutex_);
      initialization_status_ =
          InternalErrorBuilder().LogError()
          << "Failed to advertise gripper status on topic '" << status_topic_
          << "' for " << plugin_name_;
      return;
    }
  }

  if (!dio_command_topic_.empty()) {
    bool success = node_.Subscribe(
        dio_command_topic_, &FixedJointGripperPlugin::OnDioCommand, this);
    if (!success) {
      absl::MutexLock lock(initialization_status_mutex_);
      initialization_status_ = InternalErrorBuilder().LogError()
                               << "Failed to subscribe to dio command topic '"
                               << dio_command_topic_ << "' for "
                               << plugin_name_;
      return;
    }
  }
  if (!command_topic_.empty()) {
    bool success = node_.Subscribe(command_topic_,
                                   &FixedJointGripperPlugin::OnCommand, this);
    if (!success) {
      absl::MutexLock lock(initialization_status_mutex_);
      initialization_status_ = InternalErrorBuilder().LogError()
                               << "Failed to subscribe to command topic '"
                               << command_topic_ << "' for " << plugin_name_;
      return;
    }
  }

  // Configure succeeded.
  absl::MutexLock lock(initialization_status_mutex_);
  initialization_status_ = absl::OkStatus();
}

void FixedJointGripperPlugin::InitGPIOGripperConnection(
    const intrinsic_proto::eoat::SuctionGripperConfig& suction_gripper_config) {
  QCHECK_EQ(gpio_connection_, nullptr)
      << "Cannot initialize gpio gripper connection twice for plugin "
      << plugin_name_ << ".";

  auto set_command = [this](const GripperCommand& command) -> absl::Status {
    return OnGpioCommand(command);
  };
  auto get_status = [this]() -> GripperStatus {
    return (fixed_joint_gripper_ == nullptr)
               ? GripperStatus{}
               : fixed_joint_gripper_->GetStatus();
  };

  gpio_connection_handle_ =
      absl::Substitute("$0[$1]", plugin_name_, parent_entity_);
  gpio_connection_ = std::make_unique<GPIOGripperPluginConnection>(
      suction_gripper_config, gpio_connection_handle_, set_command, get_status);
  auto& gpio_service = GPIOService::StartSingleton();
  if (auto register_status =
          gpio_service.RegisterPluginConnection(gpio_connection_.get());
      !register_status.ok()) {
    LOG(ERROR) << "Failed to register gpio gripper connection for plugin "
               << plugin_name_ << " with status " << register_status;
  } else {
    LOG(INFO)
        << "Successfully registered gpio gripper connection with gripper id "
        << gpio_connection_handle_;
  }
}

void FixedJointGripperPlugin::DestroyGPIOGripperConnection() {
  if (gpio_connection_ != nullptr) {
    auto& gpio_service = GPIOService::StartSingleton();
    if (!gpio_service.UnregisterPluginConnection(gpio_connection_handle_)
             .ok()) {
      LOG(ERROR) << "Failed to unregister gpio gripper connection for plugin "
                 << plugin_name_;
    } else {
      LOG(INFO) << "Successfully unregistered gpio gripper connection. ";
    }
    gpio_connection_.reset();
  }
}

void FixedJointGripperPlugin::PreUpdate(
    const ::gz::sim::UpdateInfo& info, ::gz::sim::EntityComponentManager& ecm) {
  {
    absl::MutexLock lock(initialization_status_mutex_);
    if (!initialization_status_.ok()) {
      return;
    }
  }
  fixed_joint_gripper_->ProcessCommand(ecm);
  PublishStatus();
}

// This updates grasped_object_link_entity_ at every timestep
void FixedJointGripperPlugin::PostUpdate(
    const ::gz::sim::UpdateInfo& info,
    const ::gz::sim::EntityComponentManager& ecm) {
  {
    absl::MutexLock lock(initialization_status_mutex_);
    if (!initialization_status_.ok()) {
      return;
    }
  }
  fixed_joint_gripper_->UpdateGraspedObject(ecm);
}

absl::Status FixedJointGripperPlugin::LoadSDF(sdf::Element* sdf) {
  if (sdf->HasElement(kDioCommandElementName)) {
    INTR_ASSIGN_OR_RETURN(dio_command_topic_, CheckAndLoadFromSDF<std::string>(
                                                  kDioCommandElementName, sdf));
  }
  if (sdf->HasElement(kCommandElementName)) {
    INTR_ASSIGN_OR_RETURN(command_topic_, CheckAndLoadFromSDF<std::string>(
                                              kCommandElementName, sdf));
  }
  if (dio_command_topic_.empty() && command_topic_.empty()) {
    LOG(WARNING) << absl::Substitute(
        "Neither $0 nor $1 was provided for FixedJointGripperPlugin named "
        "$2, this fixed joint gripper will not be controllable via pubsub.",
        kDioCommandElementName, kCommandElementName, plugin_name_);
  }

  if (sdf->HasElement(kStatusElementName)) {
    INTR_ASSIGN_OR_RETURN(status_topic_, CheckAndLoadFromSDF<std::string>(
                                             kStatusElementName, sdf));
  }
  if (sdf->HasElement(kDioStatusElementName)) {
    INTR_ASSIGN_OR_RETURN(dio_status_topic_, CheckAndLoadFromSDF<std::string>(
                                                 kDioStatusElementName, sdf));
  }
  if (dio_status_topic_.empty() && status_topic_.empty()) {
    LOG(WARNING) << absl::Substitute(
        "Neither $0 nor $1 was provided for FixedJointGripperPlugin named "
        "$2, "
        "this fixed joint gripper will not report its status.",
        kDioStatusElementName, kStatusElementName, plugin_name_);
  }

  std::vector<std::string> dio_elements{
      kDioCommandElementName,        kDioCommandGripElementName,
      kDioCommandReleaseElementName, kDioStatusElementName,
      kDioStatusAttachedElementName, kDioStatusDetachedElementName};
  auto has_element = [&](const std::string& element) {
    return sdf->HasElement(element);
  };
  if (std::any_of(dio_elements.begin(), dio_elements.end(), has_element) &&
      !std::all_of(dio_elements.begin(), dio_elements.end(), has_element)) {
    LOG(ERROR) << absl::Substitute(
        "Some but not all of [$0] are provided for FixedJointGripperPlugin "
        "named $1, this fixed joint gripper might expect DIO to work but DIO "
        "won't work properly.",
        absl::StrJoin(dio_elements, ", "), plugin_name_);
  }

  if (sdf->HasElement(kDioCommandGripElementName)) {
    INTR_ASSIGN_OR_RETURN(
        dio_command_grip_,
        CheckAndLoadFromSDF<int>(kDioCommandGripElementName, sdf));
  }
  if (sdf->HasElement(kDioCommandReleaseElementName)) {
    INTR_ASSIGN_OR_RETURN(
        dio_command_release_,
        CheckAndLoadFromSDF<int>(kDioCommandReleaseElementName, sdf));
  }

  if (sdf->HasElement(kDioStatusAttachedElementName)) {
    INTR_ASSIGN_OR_RETURN(
        dio_status_attached_,
        CheckAndLoadFromSDF<int>(kDioStatusAttachedElementName, sdf));
  }
  if (sdf->HasElement(kDioStatusDetachedElementName)) {
    INTR_ASSIGN_OR_RETURN(
        dio_status_detached_,
        CheckAndLoadFromSDF<int>(kDioStatusDetachedElementName, sdf));
  }

  INTR_ASSIGN_OR_RETURN(gripper_link_name_, CheckAndLoadFromSDF<std::string>(
                                                kGripperLinkElementName, sdf));

  if (sdf->HasElement(kSuctionGripperConfigElementName)) {
    INTR_ASSIGN_OR_RETURN(std::string suction_gripper_config,
                          CheckAndLoadFromSDF<std::string>(
                              kSuctionGripperConfigElementName, sdf));
    suction_gripper_config_.emplace();
    if (!::google::protobuf::TextFormat::ParseFromString(
            suction_gripper_config, &suction_gripper_config_.value())) {
      return InvalidArgumentErrorBuilder()
             << "Failed to parse plugin field "
             << kSuctionGripperConfigElementName
             << " as a intrinsic_proto::eoat::SuctionGripperConfig";
    }
  }
  auto debounce_time_parse_result = sdf->Get<double>(
      kGraspDebounceTimeSecondsElementName, kDefaultGraspDebounceTimeSeconds);
  if (!debounce_time_parse_result.second) {
    LOG(INFO) << "Using default value for seconds to detach = "
              << kDefaultGraspDebounceTimeSeconds << " for " << plugin_name_;
  }
  grasp_debounce_time_seconds_ = debounce_time_parse_result.first;

  return absl::OkStatus();
}

void FixedJointGripperPlugin::OnDioCommand(const ::gz::msgs::UInt32& command) {
  const int new_dio_command = command.data();
  const GripperCommand new_command = FromDioCommand(new_dio_command);
  OnCommand(new_command);
}

absl::Status FixedJointGripperPlugin::OnGpioCommand(
    const intrinsic_proto::simulation::gazebo::GripperCommand& command) {
  {
    absl::MutexLock lock(initialization_status_mutex_);
    if (absl::IsUnavailable(initialization_status_)) {
      return initialization_status_;
    } else if (!initialization_status_.ok()) {
      return FailedPreconditionErrorBuilder()
             << "FixedJointGripperPlugin initialization failed with: "
             << initialization_status_;
    }
  }
  OnCommand(command);
  return absl::OkStatus();
}

void FixedJointGripperPlugin::OnCommand(const GripperCommand& command) {
  {
    absl::MutexLock lock(initialization_status_mutex_);
    if (!initialization_status_.ok()) {
      return;
    }
  }
  fixed_joint_gripper_->OnCommand(command);
}

bool FixedJointGripperPlugin::DidReceiveCommandGrip() const {
  return fixed_joint_gripper_ ? fixed_joint_gripper_->GetCommand().command() ==
                                    GripperCommand::GRASP
                              : false;
}

bool FixedJointGripperPlugin::DidReceiveCommandRelease() const {
  return fixed_joint_gripper_ ? fixed_joint_gripper_->GetCommand().command() ==
                                    GripperCommand::RELEASE
                              : false;
}

bool FixedJointGripperPlugin::IsGrasping() const {
  return fixed_joint_gripper_ ? fixed_joint_gripper_->IsGrasping() : false;
}

::gz::sim::Entity FixedJointGripperPlugin::GraspedLink() const {
  return fixed_joint_gripper_ ? fixed_joint_gripper_->GraspedLink()
                              : kNullEntity;
}

GripperCommand FixedJointGripperPlugin::FromDioCommand(int dio_command) const {
  GripperCommand command;
  command.set_command(GripperCommand::UNKNOWN);
  if (dio_command == dio_command_release_) {
    command.set_command(GripperCommand::RELEASE);
  } else if (dio_command == dio_command_grip_) {
    command.set_command(GripperCommand::GRASP);
  }
  if (command.command() == GripperCommand::UNKNOWN) {
    LOG(ERROR) << "Received unknown dio command " << dio_command
               << ". Expected either release command " << dio_command_release_
               << " or grip command " << dio_command_grip_;
  }
  return command;
}

int FixedJointGripperPlugin::GetDioStatus() const {
  return IsGrasping() ? dio_status_attached_ : dio_status_detached_;
}

void FixedJointGripperPlugin::PublishStatus() {
  if (dio_status_pub_) {
    ::gz::msgs::UInt32 msg;
    msg.set_data(GetDioStatus());
    bool published = dio_status_pub_.Publish(msg);
    LOG_IF_EVERY_N_SEC(ERROR, !published, kVerboseLogIntervalSeconds)
        << "Failed to publish dio status for gripper " << plugin_name_;
  }
  if (status_pub_) {
    bool published = status_pub_.Publish(fixed_joint_gripper_->GetStatus());
    LOG_IF_EVERY_N_SEC(ERROR, !published, kVerboseLogIntervalSeconds)
        << "Failed to publish status for gripper " << plugin_name_;
  }
}

absl::Status FixedJointGripperPlugin::GetInitializationStatus() {
  absl::MutexLock lock(initialization_status_mutex_);
  return initialization_status_;
}

}  // namespace simulation
}  // namespace intrinsic
