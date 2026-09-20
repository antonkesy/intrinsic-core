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

#include "intrinsic/simulation/gazebo/plugins/gazebo_hwm.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status_matchers.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/notification.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/server_builder.h"
#include "gz/sim/Entity.hh"
#include "internal/testing.h"
#include "intrinsic/icon/control/mock_realtime_clock.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/icon_state_register.h"
#include "intrinsic/icon/hal/interfaces/icon_state.fbs.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/hal/proto/hardware_module_config.pb.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_matchers.h"
#include "intrinsic/simulation/gazebo/plugins/gravity_compensator.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_interface_data.h"
#include "intrinsic/util/thread/thread.h"

namespace intrinsic::simulation {
namespace {

using ::testing::Return;

constexpr absl::string_view kModuleName = "the_dread_gazebo";
constexpr char kJointGroup[] = "robobot";
constexpr size_t kNumJoints = 3;

TEST(GazeboHwmTest, StartsDeactivated) {
  std::string shared_memory_namespace = icon::UniqueMemoryNamespace();
  intrinsic_proto::icon::HardwareModuleConfig config;
  config.set_name(kModuleName);
  icon::ModuleConfig module_config(config, shared_memory_namespace,
                                   /*realtime_clock=*/nullptr);

  ASSERT_OK_AND_ASSIGN(
      auto shm_manager,
      icon::SharedMemoryManager::Create(shared_memory_namespace, kModuleName));
  auto interface_registry = icon::HardwareInterfaceRegistry(*shm_manager);
  ASSERT_OK_AND_ASSIGN(
      auto icon_state_interface,
      interface_registry.AdvertiseMutableInterface<intrinsic_fbs::IconState>(
          icon::kIconStateInterfaceName));
  grpc::ServerBuilder builder;
  icon::HardwareModuleInitContext init_context(interface_registry, builder,
                                               module_config);
  std::vector<std::unique_ptr<GravityCompensator>>
      position_cmd_gravity_compensators;
  std::vector<std::unique_ptr<GravityCompensator>>
      torque_cmd_gravity_compensators;
  for (size_t i = 0; i < kNumJoints; ++i) {
    position_cmd_gravity_compensators.push_back(nullptr);
    torque_cmd_gravity_compensators.push_back(nullptr);
  }
  absl::flat_hash_map<std::string,
                      hardware_interface_data::HardwareInterfaceData>
      hardware_interface_name_to_gazebo_data;
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_position_command",
      hardware_interface_data::NonStrictJointPositionCommandData{
          .joint_group_name = kJointGroup,
          .previous_setpoints = std::vector<double>(kNumJoints, 0.),
          .gravity_compensators = std::move(position_cmd_gravity_compensators),
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_position_state", hardware_interface_data::JointPositionStateData{
                                  .joint_group_name = kJointGroup,
                              });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_velocity_state", hardware_interface_data::JointVelocityStateData{
                                  .joint_group_name = kJointGroup,
                              });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_acceleration_state",
      hardware_interface_data::JointAccelerationStateData{
          .joint_group_name = kJointGroup,
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "process_wrench_command",
      hardware_interface_data::KinematicChainProcessWrenchCommandData{
          .joint_group_name = kJointGroup,
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_torque_command",
      hardware_interface_data::NonStrictJointTorqueCommandData{
          .joint_group_name = kJointGroup,
          .gravity_compensators = std::move(torque_cmd_gravity_compensators),
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_torque_state", hardware_interface_data::JointTorqueStateData{
                                .joint_group_name = kJointGroup,
                            });
  hardware_interface_name_to_gazebo_data.emplace(
      "payload_command",
      hardware_interface_data::KinematicChainPayloadCommandData{
          .joint_group_name = kJointGroup,
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "payload_state", hardware_interface_data::KinematicChainPayloadStateData{
                           .joint_group_name = kJointGroup,
                       });
  absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>
      joint_groups_by_name{
          {kJointGroup,
           std::vector<gz::sim::Entity>(kNumJoints, gz::sim::kNullEntity)}};
  GazeboHardwareModule module(
      GazeboHardwareModule::Config{.joint_groups_by_name = joint_groups_by_name,
                                   .hardware_interface_name_to_gazebo_data =
                                       hardware_interface_name_to_gazebo_data});
  EXPECT_OK(module.Init(init_context));
  // The two extra interfaces are safety_status and icon_state
  EXPECT_EQ(interface_registry.Size(),
            hardware_interface_name_to_gazebo_data.size() + 2);
  EXPECT_FALSE(module.IsActive());
  EXPECT_THAT(module.Activate(), icon::RealtimeIsOk());
  EXPECT_TRUE(module.IsActive());
}

TEST(GazeboHwmTest, InitRegistersJointInterfaces) {
  std::string shared_memory_namespace = icon::UniqueMemoryNamespace();
  intrinsic_proto::icon::HardwareModuleConfig config;
  config.set_name(kModuleName);
  icon::ModuleConfig module_config(config, shared_memory_namespace,
                                   /*realtime_clock=*/nullptr);

  ASSERT_OK_AND_ASSIGN(
      auto shm_manager,
      icon::SharedMemoryManager::Create(shared_memory_namespace, kModuleName));
  auto interface_registry = icon::HardwareInterfaceRegistry(*shm_manager);
  ASSERT_OK_AND_ASSIGN(
      auto icon_state_interface,
      interface_registry.AdvertiseMutableInterface<intrinsic_fbs::IconState>(
          icon::kIconStateInterfaceName));
  grpc::ServerBuilder builder;
  icon::HardwareModuleInitContext init_context(interface_registry, builder,
                                               module_config);
  std::vector<std::unique_ptr<GravityCompensator>>
      position_cmd_gravity_compensators;
  std::vector<std::unique_ptr<GravityCompensator>>
      torque_cmd_gravity_compensators;
  for (size_t i = 0; i < kNumJoints; ++i) {
    position_cmd_gravity_compensators.push_back(nullptr);
    torque_cmd_gravity_compensators.push_back(nullptr);
  }
  absl::flat_hash_map<std::string,
                      hardware_interface_data::HardwareInterfaceData>
      hardware_interface_name_to_gazebo_data;
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_position_command",
      hardware_interface_data::NonStrictJointPositionCommandData{
          .joint_group_name = kJointGroup,
          .previous_setpoints = std::vector<double>(kNumJoints, 0.),
          .gravity_compensators = std::move(position_cmd_gravity_compensators),
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_position_state", hardware_interface_data::JointPositionStateData{
                                  .joint_group_name = kJointGroup,
                              });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_velocity_state", hardware_interface_data::JointVelocityStateData{
                                  .joint_group_name = kJointGroup,
                              });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_acceleration_state",
      hardware_interface_data::JointAccelerationStateData{
          .joint_group_name = kJointGroup,
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "process_wrench_command",
      hardware_interface_data::KinematicChainProcessWrenchCommandData{
          .joint_group_name = kJointGroup,
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_torque_command",
      hardware_interface_data::NonStrictJointTorqueCommandData{
          .joint_group_name = kJointGroup,
          .gravity_compensators = std::move(torque_cmd_gravity_compensators),
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "joint_torque_state", hardware_interface_data::JointTorqueStateData{
                                .joint_group_name = kJointGroup,
                            });
  hardware_interface_name_to_gazebo_data.emplace(
      "payload_command",
      hardware_interface_data::KinematicChainPayloadCommandData{
          .joint_group_name = kJointGroup,
      });
  hardware_interface_name_to_gazebo_data.emplace(
      "payload_state", hardware_interface_data::KinematicChainPayloadStateData{
                           .joint_group_name = kJointGroup,
                       });
  absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>
      joint_groups_by_name{
          {kJointGroup,
           std::vector<gz::sim::Entity>(kNumJoints, gz::sim::kNullEntity)}};
  GazeboHardwareModule module(
      GazeboHardwareModule::Config{.joint_groups_by_name = joint_groups_by_name,
                                   .hardware_interface_name_to_gazebo_data =
                                       hardware_interface_name_to_gazebo_data});
  EXPECT_OK(module.Init(init_context));
  // The two extra interfaces are safety_status and icon_state
  EXPECT_EQ(interface_registry.Size(),
            hardware_interface_name_to_gazebo_data.size() + 2);
}

TEST(GazeboHwmTest, TickDoesNotBlockWithoutRealtimeClock) {
  intrinsic_proto::icon::HardwareModuleConfig config;
  config.set_name(kModuleName);
  std::string memory_namespace = icon::UniqueMemoryNamespace();
  icon::ModuleConfig module_config(
      /*config=*/config,
      /*shared_memory_namespace=*/memory_namespace,
      /*realtime_clock=*/nullptr);

  ASSERT_OK_AND_ASSIGN(auto shm_manager, icon::SharedMemoryManager::Create(
                                             memory_namespace, kModuleName));
  auto interface_registry = icon::HardwareInterfaceRegistry(*shm_manager);
  ASSERT_OK_AND_ASSIGN(
      auto icon_state_interface,
      interface_registry.AdvertiseMutableInterface<intrinsic_fbs::IconState>(
          icon::kIconStateInterfaceName));
  grpc::ServerBuilder builder;
  icon::HardwareModuleInitContext init_context(interface_registry, builder,
                                               module_config);
  absl::flat_hash_map<std::string,
                      hardware_interface_data::HardwareInterfaceData>
      empty_hardware_interface_data_map;
  absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>
      empty_joint_groups_by_name;
  GazeboHardwareModule module(GazeboHardwareModule::Config{
      .joint_groups_by_name = empty_joint_groups_by_name,
      .hardware_interface_name_to_gazebo_data =
          empty_hardware_interface_data_map,
  });
  EXPECT_OK(module.Init(init_context));
  EXPECT_OK(module.Prepare());
  EXPECT_THAT(module.Activate(), icon::RealtimeIsOk());

  Time sim_time = Clock::Now();
  EXPECT_THAT(module.RequestIconTick(sim_time), icon::RealtimeIsOk());
  EXPECT_THAT(module.ReadStatus(), icon::RealtimeIsOk());
  EXPECT_THAT(module.ApplyCommand(), icon::RealtimeIsOk());
  EXPECT_THAT(module.WaitForIconTicksToFinish(), icon::RealtimeIsOk());
}

TEST(GazeboHwmTest, ReadStatusBlocksUntilRequestIconTickWithoutRealtimeClock) {
  intrinsic_proto::icon::HardwareModuleConfig config;
  config.set_name(kModuleName);
  std::string memory_namespace = icon::UniqueMemoryNamespace();
  icon::ModuleConfig module_config(
      /*config=*/config,
      /*shared_memory_namespace=*/memory_namespace,
      /*realtime_clock=*/nullptr);

  ASSERT_OK_AND_ASSIGN(auto shm_manager, icon::SharedMemoryManager::Create(
                                             memory_namespace, kModuleName));
  auto interface_registry = icon::HardwareInterfaceRegistry(*shm_manager);
  ASSERT_OK_AND_ASSIGN(
      auto icon_state_interface,
      interface_registry.AdvertiseMutableInterface<intrinsic_fbs::IconState>(
          icon::kIconStateInterfaceName));
  grpc::ServerBuilder builder;
  icon::HardwareModuleInitContext init_context(interface_registry, builder,
                                               module_config);
  absl::flat_hash_map<std::string,
                      hardware_interface_data::HardwareInterfaceData>
      empty_hardware_interface_data_map;
  absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>
      empty_joint_groups_by_name;
  GazeboHardwareModule module(GazeboHardwareModule::Config{
      .joint_groups_by_name = empty_joint_groups_by_name,
      .hardware_interface_name_to_gazebo_data =
          empty_hardware_interface_data_map,
  });
  EXPECT_OK(module.Init(init_context));
  EXPECT_OK(module.Prepare());
  EXPECT_THAT(module.Activate(), icon::RealtimeIsOk());

  absl::Notification read_status_done;
  icon::RealtimeStatus read_status_result;
  intrinsic::Thread read_status{[&]() {
    read_status_result = module.ReadStatus();
    read_status_done.Notify();
  }};
  // Sleep long enough for the thread to start up.
  // In fact, this is long enough that ReadStatus() should return, if it wasn't
  // blocked.
  absl::SleepFor(absl::Milliseconds(500));
  EXPECT_FALSE(read_status_done.HasBeenNotified());
  Time sim_time = Clock::Now();
  EXPECT_THAT(module.RequestIconTick(sim_time), icon::RealtimeIsOk());
  // Wait until the thread is done
  read_status_done.WaitForNotification();
  read_status.join();
  EXPECT_THAT(read_status_result, icon::RealtimeIsOk());
}

TEST(GazeboHwmTest,
     WaitForIconTicksToFinishBlocksUntilExpectedNumberOfReadStatusCalls) {
  intrinsic_proto::icon::HardwareModuleConfig config;
  config.set_name(kModuleName);
  std::string memory_namespace = icon::UniqueMemoryNamespace();
  icon::ModuleConfig module_config(
      /*config=*/config,
      /*shared_memory_namespace=*/memory_namespace,
      /*realtime_clock=*/nullptr);

  ASSERT_OK_AND_ASSIGN(auto shm_manager, icon::SharedMemoryManager::Create(
                                             memory_namespace, kModuleName));
  auto interface_registry = icon::HardwareInterfaceRegistry(*shm_manager);
  ASSERT_OK_AND_ASSIGN(
      auto icon_state_interface,
      interface_registry.AdvertiseMutableInterface<intrinsic_fbs::IconState>(
          icon::kIconStateInterfaceName));
  grpc::ServerBuilder builder;
  icon::HardwareModuleInitContext init_context(interface_registry, builder,
                                               module_config);
  absl::flat_hash_map<std::string,
                      hardware_interface_data::HardwareInterfaceData>
      empty_hardware_interface_data_map;
  absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>
      empty_joint_groups_by_name;
  GazeboHardwareModule module(GazeboHardwareModule::Config{
      .joint_groups_by_name = empty_joint_groups_by_name,
      .hardware_interface_name_to_gazebo_data =
          empty_hardware_interface_data_map,
  });
  EXPECT_OK(module.Init(init_context));
  EXPECT_OK(module.Prepare());
  EXPECT_THAT(module.Activate(), icon::RealtimeIsOk());

  // Request multiple ICON ticks.
  EXPECT_THAT(module.RequestIconTick(Clock::Now()), icon::RealtimeIsOk());
  EXPECT_THAT(module.RequestIconTick(Clock::Now()), icon::RealtimeIsOk());

  absl::Notification wait_for_ticks_done;
  icon::RealtimeStatus wait_for_ticks_result;
  intrinsic::Thread wait_for_ticks{[&]() {
    wait_for_ticks_result = module.WaitForIconTicksToFinish();
    wait_for_ticks_done.Notify();
  }};
  // Sleep long enough for the thread to start up.
  // In fact, this is long enough that WaitForIconTicksToFinish() should return,
  // if it wasn't blocked.
  absl::SleepFor(absl::Milliseconds(500));
  EXPECT_FALSE(wait_for_ticks_done.HasBeenNotified());
  EXPECT_THAT(module.ReadStatus(), icon::RealtimeIsOk());
  // We're expecting two calls to ReadStatus, so the thread should still be
  // blocked after the first call.
  absl::SleepFor(absl::Milliseconds(500));
  EXPECT_FALSE(wait_for_ticks_done.HasBeenNotified());
  // This should unblock WaitForIconTicksToFinish()
  EXPECT_THAT(module.ReadStatus(), icon::RealtimeIsOk());
  // Wait until the thread is done
  wait_for_ticks_done.WaitForNotification();
  wait_for_ticks.join();
  EXPECT_THAT(wait_for_ticks_result, icon::RealtimeIsOk());
}

TEST(GazeboHwmTest, TickCallsRealtimeClockMethods) {
  intrinsic_proto::icon::HardwareModuleConfig config;
  config.set_name(kModuleName);
  std::string memory_namespace = icon::UniqueMemoryNamespace();
  icon::MockRealtimeClock clock;
  EXPECT_CALL(clock, TickBlockingWithDeadline)
      .WillOnce(Return(icon::OkStatus()));
  icon::ModuleConfig module_config(
      /*config=*/config,
      /*shared_memory_namespace=*/memory_namespace,
      /*realtime_clock=*/&clock);

  ASSERT_OK_AND_ASSIGN(auto shm_manager, icon::SharedMemoryManager::Create(
                                             memory_namespace, kModuleName));
  auto interface_registry = icon::HardwareInterfaceRegistry(*shm_manager);
  ASSERT_OK_AND_ASSIGN(
      auto icon_state_interface,
      interface_registry.AdvertiseMutableInterface<intrinsic_fbs::IconState>(
          icon::kIconStateInterfaceName));

  grpc::ServerBuilder builder;
  icon::HardwareModuleInitContext init_context(interface_registry, builder,
                                               module_config);
  absl::flat_hash_map<std::string,
                      hardware_interface_data::HardwareInterfaceData>
      empty_hardware_interface_data_map;
  absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>
      empty_joint_groups_by_name;
  GazeboHardwareModule module(GazeboHardwareModule::Config{
      .joint_groups_by_name = empty_joint_groups_by_name,
      .hardware_interface_name_to_gazebo_data =
          empty_hardware_interface_data_map,
  });
  EXPECT_OK(module.Init(init_context));
  EXPECT_OK(module.Prepare());
  EXPECT_THAT(module.Activate(), icon::RealtimeIsOk());

  auto module_cleanup =
      absl::MakeCleanup([&]() { EXPECT_OK(module.Shutdown()); });
  Time sim_time = Clock::Now();
  EXPECT_THAT(module.RequestIconTick(sim_time), icon::RealtimeIsOk());
  EXPECT_THAT(module.ReadStatus(), icon::RealtimeIsOk());
  EXPECT_THAT(module.ApplyCommand(), icon::RealtimeIsOk());
  EXPECT_THAT(module.WaitForIconTicksToFinish(), icon::RealtimeIsOk());
}

}  // namespace
}  // namespace intrinsic::simulation
