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

#ifndef INTRINSIC_ICON_CC_CLIENT_TESTING_RTCL_CONTROLLER_CHANNEL_FAKE_H_
#define INTRINSIC_ICON_CC_CLIENT_TESTING_RTCL_CONTROLLER_CHANNEL_FAKE_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "grpcpp/channel.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/cc_client/testing/channel_fake.h"
#include "intrinsic/icon/cc_client/testing/rtcl_controller_fake_robot_connection.h"
#include "intrinsic/icon/cc_client/testing/rtcl_controller_fake_session.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/proto/v1/service.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/testing/in_process_server.h"
#include "intrinsic/util/grpc/channel_interface.h"

namespace intrinsic::icon {

// A ChannelFake class that replicates the behavior of RtclController, including
// support for multi-part Actions.
//
// This class requires much less configuration than RtclChannelFake, since it
// simply uses the existing Action factory registry. Its build target links in
// all of the default action types, so if you do *not* want them to be available
// to the system under test, you must call
//
//  GetGlobalRtclActionFactoryRegistry().ClearForTestingOnly();
class RtclControllerChannelFake final : public ChannelInterface {
 public:
  static constexpr absl::string_view kDefaultServerName = "fake_icon_server";

  // Builds an RtclControllerChannelFake with the given configuration and
  // expectations.
  class Builder {
   public:
    Builder()
        : mode_(RtclControllerFakeSession::ValidationMode::kNice),
          options_(InProcessApplicationLayerServer::DefaultOptions()) {}
    // `mode` determines the behavior of the ChannelFake in case of unexpected
    // actions. kStrict fails on any deviation from expectations, while kNice
    // silently carries on.
    explicit Builder(RtclControllerFakeSession::ValidationMode mode,
                     double control_frequency_hz = 1000,
                     InProcessApplicationLayerServer::Options options =
                         InProcessApplicationLayerServer::DefaultOptions())
        : mode_(mode), options_(options) {}

    // Sets the server name of the ChannelFake.
    // If this is never called, the server name defaults to
    // `kDefaultServerName`.
    Builder& WithServerName(absl::string_view server_name);

    Builder& WithInitialRobotStatus(
        const intrinsic_proto::icon::v1::GetStatusResponse& robot_status);

    // If this is called (and StartFaulted() is NOT called afterwards), then the
    // ChannelFake starts in the Faulted state.
    Builder& StartFaulted();

    // If this is called (and StartFaulted() is NOT called afterwards), then the
    // ChannelFake starts in the Disabled state. Skill tests should not need to
    // use this, since ICON
    // * automatically enables whenever possible
    // * delays starting a session until it's enabled
    //
    // Use this only to validate interactions with ICON that do not use a
    // session (i.e. that can encounter ICON in the brief periods where it is
    // currently moving from disabled to enabled).
    Builder& StartDisabled();

    // Specifies the behaviors for the next session. Subsequent calls create
    // session expectations for additional sessions.
    Builder& OnNextSession(const SessionWill& session_behaviors);

    // `part_configs` contains the configuration protos that will be available
    // to action factories.
    //
    // Returns AlreadyExistsError if multiple elements of `part_configs` have
    // the same name.
    absl::StatusOr<std::shared_ptr<RtclControllerChannelFake>> Build(
        absl::Span<const intrinsic_proto::icon::v1::PartConfig> part_configs);

   private:
    std::string server_name_ =
        std::string(RtclControllerChannelFake::kDefaultServerName);
    RtclControllerFakeSession::ValidationMode mode_;
    std::vector<SessionWill> session_expectations_;
    intrinsic_proto::icon::v1::GetStatusResponse initial_robot_status_;
    OperationalState start_state_ = OperationalState::kEnabled;
    InProcessApplicationLayerServer::Options options_;
    absl::flat_hash_map<std::string, size_t> part_name_to_index_map_;
  };

  std::shared_ptr<grpc::Channel> GetChannel() const override;

  // Set the robot status returned by subsequent calls to `Client::GetStatus`.
  void SetRobotStatus(
      const intrinsic_proto::icon::v1::GetStatusResponse& robot_status);

  // Returns the current map of part properties. Use this to validate that an
  // ICON client has updated the part properties that you expect it to.
  // Thread-safe.
  PartPropertyMap GetPartPropertiesTestOnly() const;

  // Overwrites the current part property map. Use this to initialize part
  // properties – `part_properties` determines which parts and properties
  // "exist" to the ICON API, and which type each property has.
  // Thread-safe.
  void SetPartPropertiesTestOnly(const PartPropertyMap& part_properties);

  // Returns the number of times that the ICON service has been (re)started.
  // This happens when a client calls RestartServer(), or when a client calls
  // ClearFaults() after ICON encountered a fatal fault.
  int NumIconServiceRestarts() const;

 protected:
  RtclControllerChannelFake(
      std::unique_ptr<RtclControllerFakeRobotConnection> robot_connection,
      InProcessApplicationLayerServer::Options options);

 private:
  std::unique_ptr<RtclControllerFakeRobotConnection> robot_connection_;
  std::unique_ptr<InProcessApplicationLayerServer> server_;
};
}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CC_CLIENT_TESTING_RTCL_CONTROLLER_CHANNEL_FAKE_H_
