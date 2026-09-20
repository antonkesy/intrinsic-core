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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_TESTING_FAKE_REALTIME_PART_H_
#define INTRINSIC_ICON_CONTROL_PARTS_TESTING_FAKE_REALTIME_PART_H_

#include <array>
#include <functional>
#include <string>
#include <tuple>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/container/fixed_array.h"
#include "absl/log/check.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/parts/realtime_part_property_access.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// A FakeRealtimePart makes the supplied PartInterfaces available to Actions and
// implements the RealtimePartStateMachine TransitionHandler interface using
// FakeTransitionHandler.
// Optionally, you can supply callbacks that are called in the ReadStatus() /
// ApplyCommand() methods. Those callbacks get mutable references to the
// PartInterfaces, which allows them to implement more complex behavior than
// just setting them up once beforehand.
template <class... InterfaceTs>
class FakeRealtimePart : public RealtimePartInterface {
 public:
  using GetStateCallback = std::function<RealtimeOperationalStatus()>;
  using ReadStatusCallback = std::function<RealtimeStatus(
      ReadStatusParameters params, InterfaceTs&... interfaces)>;
  using ApplyCommandCallback = std::function<RealtimeStatus(
      ApplyCommandParameters params, InterfaceTs&... interfaces)>;

  // Creates a FakeRealtimePart with the given name, interfaces and callbacks.
  // Note that the optional callback parameters prevent class template argument
  // deduction, so you'll always have to supply the types of all interfaces.
  explicit FakeRealtimePart(
      InterfaceTs&&... interfaces,
      ReadStatusCallback read_status_callback = &AlwaysOkReadStatus,
      ApplyCommandCallback apply_command_callback = &AlwaysOkApplyCommand,
      GetStateCallback get_state_callback = &AlwaysEnabledState)
      : interfaces_(std::forward<InterfaceTs>(interfaces)...),
        read_status_(std::move(read_status_callback)),
        apply_command_(std::move(apply_command_callback)),
        get_state_(std::move(get_state_callback)) {
    RegisterInterfacesOrDie();
  }

  // This is used as a default for both the ReadStatus and ApplyCommand
  // callbacks.
  static RealtimeStatus AlwaysOkReadStatus(ReadStatusParameters params,
                                           InterfaceTs&... interfaces) {
    return OkStatus();
  }
  static RealtimeStatus AlwaysOkApplyCommand(ApplyCommandParameters params,
                                             InterfaceTs&... interfaces) {
    return OkStatus();
  }
  static RealtimeOperationalStatus AlwaysEnabledState() {
    return {.state = RealtimeOperationalState::kEnabled};
  }

  // Sets the group of hardware the part depends on.
  // By default, dependency to `operational_hardware` is alreadyset.
  void SetHardwareDependencies(HardwareGroupSet hardware_dependencies) {
    hardware_dependencies_ = hardware_dependencies;
  }

  HardwareGroupSet GetHardwareDependencies() const override {
    return hardware_dependencies_;
  }

  // Returns a reference to one of the FakeRealtimePart's interfaces, which can
  // be used to set fake interfaces' return values.
  template <class T>
  T& GetInterface() {
    return std::get<T>(interfaces_);
  }

  // Gets an interface for read-only access.
  template <class T>
  const T& GetInterface() const {
    return std::get<T>(interfaces_);
  }

  RealtimeStatusOr<RealtimeOperationalStatus> GetOperationalStatus()
      const override {
    return get_state_();
  }

  RealtimeStatus ReadStatus(ReadStatusParameters params) override {
    return read_status_(params, std::get<InterfaceTs>(interfaces_)...);
  }

  // Use this in tests where you
  // * need to interact with a part directly (cannot use ActionTestHelper)
  // * don't care about the actual parameters (f.i. part properties) of the part
  RealtimeStatus ReadStatusWithoutParamsTestOnly() {
    absl::FixedArray<PartPropertyValue> empty_part_properties_{};
    RealtimePartPropertyAccess empty_realtime_part_property_access_{
        empty_part_properties_, empty_part_properties_};
    return read_status_(
        {.part_properties = empty_realtime_part_property_access_},
        std::get<InterfaceTs>(interfaces_)...);
  }

  RealtimeStatus ApplyCommand(ApplyCommandParameters params) override {
    return apply_command_(params, std::get<InterfaceTs>(interfaces_)...);
  }

  // Use this in tests where you
  // * need to interact with a part directly (cannot use ActionTestHelper)
  // * don't care about the actual parameters (f.i. part properties) of the part
  RealtimeStatus ApplyCommandWithoutParamsTestOnly() {
    absl::FixedArray<PartPropertyValue> empty_part_properties_{};
    RealtimePartPropertyAccess empty_realtime_part_property_access_{
        empty_part_properties_, empty_part_properties_};
    return apply_command_(
        {.part_properties = empty_realtime_part_property_access_},
        std::get<InterfaceTs>(interfaces_)...);
  }

  FeatureInterfaceRegistry& GetFeatureInterfaces() override {
    return interface_registry_;
  }
  const FeatureInterfaceRegistry& GetFeatureInterfaces() const override {
    return interface_registry_;
  }

  void SetReadStatusCallback(ReadStatusCallback read_status_callback) {
    read_status_ = std::move(read_status_callback);
  }

  void SetApplyCommandCallback(ApplyCommandCallback apply_command_callback) {
    apply_command_ = std::move(apply_command_callback);
  }

  void SetGetStateCallback(GetStateCallback get_state_callback) {
    get_state_ = std::move(get_state_callback);
  }

 private:
  void RegisterInterfacesOrDie() {
    // Register each interface in interface_registry_ and save the result in an
    // array.
    std::array<RealtimeStatus, sizeof...(InterfaceTs)>
        register_interface_statuses{
            interface_registry_.RegisterAsCompatibleInterfaces(
                &std::get<InterfaceTs>(interfaces_))...};
    // A failure here is a programmer error that warrants a crash - the user
    // tried to register multiple implementations of the same PartInterface.
    CHECK(absl::c_all_of(
        register_interface_statuses,
        [](const RealtimeStatus& status) { return status.ok(); }))
        << "Failed to register interfaces, likely because you tried to use "
           "multiple implementations of the same PartInterface with "
           "FakeRealtimePart";
  }

  std::string name_;
  FeatureInterfaceRegistry interface_registry_;
  std::tuple<InterfaceTs...> interfaces_;
  ReadStatusCallback read_status_;
  ApplyCommandCallback apply_command_;
  HardwareGroupSet hardware_dependencies_{.operational_hardware = true};
  GetStateCallback get_state_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_TESTING_FAKE_REALTIME_PART_H_
