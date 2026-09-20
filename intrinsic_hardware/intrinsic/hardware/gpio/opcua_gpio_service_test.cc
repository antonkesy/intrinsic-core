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

#include "intrinsic/hardware/gpio/opcua_gpio_service.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/client_context.h"
#include "grpcpp/server.h"
#include "internal/testing.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/hardware/gpio/gpio_client.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/opcua/opcua_server.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "open62541/common.h"
#include "open62541/types.h"
#include "protobuf-matchers/protocol-buffer-matchers.h"

using ::absl_testing::IsOkAndHolds;
using ::absl_testing::StatusIs;
using ::intrinsic_proto::gpio::v1::SignalValue;
using ::intrinsic_proto::gpio::v1::SignalValueSet;
using ::protobuf_matchers::EqualsProto;
using ::testing::HasSubstr;

namespace intrinsic::opcua {

static SignalValue false_value() {
  SignalValue value;
  value.set_bool_value(false);
  return value;
}

static SignalValue true_value() {
  SignalValue value;
  value.set_bool_value(true);
  return value;
}

static absl::flat_hash_set<std::string> GetSignalNames(
    const ::intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse& resp) {
  absl::flat_hash_set<std::string> names;
  for (const auto& signal_desc : resp.signal_descriptions()) {
    names.insert(signal_desc.signal_name());
  }
  return names;
}

// Overview of the different entities involved in the `OpcuaGpioServiceTest`
// suite. Opcua server and Opcua GPIO server are configured and started in the
// `SetUp` while the Opcua GPIO client is started in the individual tests.
//
//  +---------------+          +---------------+          +-----------------+
//  |               |          |               |          |                 |
//  |    OPCUA      |          |   OPCUA GPIO  |          |   OPCUA GPIO    |
//  |    Server     |<---------+     Server    |<---------+     Client      |
//  | (port: 4840)  |          | (port: 17219) |          |                 |
//  |               |          |               |          |                 |
//  +---------------+          +---------------+          +-----------------+
//
class OpcuaGpioServiceTest : public ::testing::Test {
 public:
  void SetUp() override {
    SpinUpOpcuaServer();
    CreateOpcuaGpioService();
  }

  void TearDown() override { ShutDownOpcuaServer(); }

  void SpinUpOpcuaServer() {
    opcua_server_ = std::make_unique<::intrinsic::opcua::OpcuaServer>();

    // Starts an opcua server with scalar values in namespace=1
    auto [status, root_node] = opcua_server_->AddObjectNode("root_node");
    ASSERT_EQ(status, UA_STATUSCODE_GOOD);
    EXPECT_EQ(root_node.namespaceIndex, 0);
    EXPECT_EQ(root_node.identifierType, UA_NODEIDTYPE_STRING);

    // Adds read-write boolean scalar variables.
    for (auto v : {1, 2, 3}) {
      auto [status_v, node_v] = opcua_server_->AddBooleanVariableNode(
          {.node_id = absl::StrCat("bool_var", v),
           .display_name = absl::StrCat("bool_var", v),
           .ns_index = 1,
           .parent_node = root_node},
          false);
      ASSERT_EQ(status_v, UA_STATUSCODE_GOOD);
      EXPECT_EQ(node_v.namespaceIndex, 1);
      EXPECT_EQ(node_v.identifierType, UA_NODEIDTYPE_STRING);
    }

    // Adds read-only boolean scalar variables.
    for (auto v : {1}) {
      auto [status_v, node_v] = opcua_server_->AddBooleanVariableNode(
          {.node_id = absl::StrCat("bool_read_only", v),
           .display_name = absl::StrCat("bool_read_only", v),
           .ns_index = 1,
           .parent_node = root_node,
           .access_level_mask = UA_ACCESSLEVELMASK_READ},
          false);
      ASSERT_EQ(status_v, UA_STATUSCODE_GOOD);
      EXPECT_EQ(node_v.namespaceIndex, 1);
      EXPECT_EQ(node_v.identifierType, UA_NODEIDTYPE_STRING);
    }

    // Adds write-only boolean scalar variables.
    for (auto v : {1}) {
      auto [status_v, node_v] = opcua_server_->AddBooleanVariableNode(
          {.node_id = absl::StrCat("bool_write_only", v),
           .display_name = absl::StrCat("bool_write_only", v),
           .ns_index = 1,
           .parent_node = root_node,
           .access_level_mask = UA_ACCESSLEVELMASK_WRITE},
          false);
      ASSERT_EQ(status_v, UA_STATUSCODE_GOOD);
      EXPECT_EQ(node_v.namespaceIndex, 1);
      EXPECT_EQ(node_v.identifierType, UA_NODEIDTYPE_STRING);
    }

    // Adds scalar variables with different string identifier and display names.
    for (auto v : {"foo", "bar"}) {
      auto [status_v, node_v] = opcua_server_->AddBooleanVariableNode(
          {.node_id = absl::StrCat("longer_string_id_", v),
           .display_name = absl::StrCat("shorter_display_", v),
           .ns_index = 1,
           .parent_node = root_node},
          false);
      ASSERT_EQ(status_v, UA_STATUSCODE_GOOD);
      EXPECT_EQ(node_v.namespaceIndex, 1);
      EXPECT_EQ(node_v.identifierType, UA_NODEIDTYPE_STRING);
    }

    // Adds an array variable node containing boolean values.
    {
      auto [status_v, node_v] = opcua_server_->AddArrayVariableNode(
          {.node_id = "array_bool3",
           .display_name = "array_bool3",
           .ns_index = 1,
           .parent_node = root_node},
          std::vector<bool>{true, false, true});
      ASSERT_EQ(status_v, UA_STATUSCODE_GOOD);
      EXPECT_EQ(node_v.namespaceIndex, 1);
      EXPECT_EQ(node_v.identifierType, UA_NODEIDTYPE_STRING);
    }

    ASSERT_EQ(opcua_server_->StartServerAsync(), UA_STATUSCODE_GOOD);
  }

  void CreateOpcuaGpioService() {
    // Starts an opcua gpio server (basically, an opcua client) and connects to
    // the opcua server so that it can access all the signals.
    intrinsic_proto::gpio::OpcuaGpioServiceConfig opcua_gpio_srv_config;
    opcua_gpio_srv_config.set_opcua_server_address(
        absl::StrCat("opc.tcp://localhost:", kOpcuaServerPort));
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=bool_var1");
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=bool_var2");
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=bool_var3");
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=bool_read_only1");
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=bool_write_only1");
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=longer_string_id_foo");
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=longer_string_id_bar");
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=array_bool3");

    opcua_gpio_service_ =
        intrinsic::MakeOpcuaGPIOService("plc", opcua_gpio_srv_config);
    ASSERT_OK(opcua_gpio_service_);

    opcua_gpio_server_ = intrinsic::CreateServer(kOpcuaGpioServicePort,
                                                 {opcua_gpio_service_->get()});
    ASSERT_OK(opcua_gpio_server_);
  }

  void ShutDownOpcuaServer() { opcua_server_->StopServerAndWait(); }

  const int kOpcuaGpioServicePort = 17219;

 private:
  const int kOpcuaServerPort = 4840;
  std::unique_ptr<::intrinsic::opcua::OpcuaServer> opcua_server_;
  absl::StatusOr<
      std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::Service>>
      opcua_gpio_service_;
  absl::StatusOr<std::unique_ptr<::grpc::Server>> opcua_gpio_server_;
};

TEST_F(OpcuaGpioServiceTest, SimpleClientGetSignalDescriptionsRead) {
  const auto deadline =
      absl::Now() + intrinsic::connect::kGrpcClientConnectDefaultTimeout;
  auto channel = intrinsic::connect::CreateClientChannel(
      absl::StrCat("localhost:", kOpcuaGpioServicePort), deadline);
  EXPECT_OK(channel);
  auto client = intrinsic_proto::gpio::v1::GPIOService::NewStub(*channel);

  // Get Signal Descriptions
  {
    grpc::ClientContext context;
    ::intrinsic_proto::gpio::v1::GetSignalDescriptionsRequest request;
    ::intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse response;
    ASSERT_OK(ToAbslStatus(
        client->GetSignalDescriptions(&context, request, &response)));
    EXPECT_EQ(GetSignalNames(response),
              absl::flat_hash_set<std::string>(
                  {"bool_var1", "bool_var2", "bool_var3", "bool_read_only1",
                   "bool_write_only1", "shorter_display_foo",
                   "shorter_display_bar", "array_bool3"}));
  }

  // Read value for signals that have the same display name and string
  // identifier.
  {
    grpc::ClientContext context;
    ::intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("bool_var1");
    request.add_signal_names("bool_var2");
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse response;
    EXPECT_OK(ToAbslStatus(client->ReadSignals(&context, request, &response)));
  }

  // Read value using display names.
  {
    grpc::ClientContext context;
    ::intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("shorter_display_foo");
    request.add_signal_names("shorter_display_bar");
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse response;
    EXPECT_OK(ToAbslStatus(client->ReadSignals(&context, request, &response)));

    // The response should contain the display names.
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse expected_response;
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"shorter_display_foo", false_value()});
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"shorter_display_bar", false_value()});
    EXPECT_THAT(response.signal_values(),
                EqualsProto(expected_response.signal_values()));
  }

  // Read Signals using string identifier names.
  {
    grpc::ClientContext context;
    ::intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("longer_string_id_foo");
    request.add_signal_names("longer_string_id_bar");
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse response;
    ASSERT_OK(ToAbslStatus(client->ReadSignals(&context, request, &response)));

    // The response should contain the longer string names and not the display
    // names.
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse expected_response;
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"longer_string_id_foo", false_value()});
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"longer_string_id_bar", false_value()});
    EXPECT_THAT(response.signal_values(),
                EqualsProto(expected_response.signal_values()));
  }

  // Read Signals using xml node identifiers.
  {
    grpc::ClientContext context;
    ::intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("ns=1;s=longer_string_id_foo");
    request.add_signal_names("ns=1;s=longer_string_id_bar");
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse response;
    ASSERT_OK(ToAbslStatus(client->ReadSignals(&context, request, &response)));

    // The response should contain the xml node identifiers.
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse expected_response;
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"ns=1;s=longer_string_id_foo", false_value()});
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"ns=1;s=longer_string_id_bar", false_value()});
    EXPECT_THAT(response.signal_values(),
                EqualsProto(expected_response.signal_values()));
  }

  // Read the array signal with an index.
  {
    grpc::ClientContext context;
    ::intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("array_bool3.2");
    request.add_signal_names("array_bool3.1");
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse response;
    EXPECT_OK(ToAbslStatus(client->ReadSignals(&context, request, &response)));

    // The response should contain the display names.
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse expected_response;
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"array_bool3.2", true_value()});
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"array_bool3.1", false_value()});
    EXPECT_THAT(response.signal_values(),
                EqualsProto(expected_response.signal_values()));
  }
}

TEST_F(OpcuaGpioServiceTest, GpioClientReadWrite) {
  // Creates a GPIO client with the list of signals that it can write to. Note
  // that `bool_var3` is not present in this list.
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "bool_var2", "array_bool3"});

  {
    ASSERT_OK_AND_ASSIGN(auto known_signals, client.GetSignalDescriptions());

    // We expect to get all the signal names back and not just the ones we plan
    // to write to.
    EXPECT_EQ(GetSignalNames(known_signals),
              absl::flat_hash_set<std::string>(
                  {"bool_var1", "bool_var2", "bool_var3", "bool_read_only1",
                   "bool_write_only1", "shorter_display_foo",
                   "shorter_display_bar", "array_bool3"}));
    for (const auto& signal_desc : known_signals.signal_descriptions()) {
      // Signal names and descriptions are the same in the current opcua server.
      EXPECT_EQ(signal_desc.signal_name(), signal_desc.description());

      // Checks access is parsed correctly.
      if (signal_desc.signal_name() == "bool_read_only1") {
        EXPECT_FALSE(signal_desc.can_write());
        EXPECT_TRUE(signal_desc.can_read());
      } else if (signal_desc.signal_name() == "bool_write_only1") {
        EXPECT_TRUE(signal_desc.can_write());
        EXPECT_FALSE(signal_desc.can_read());
      } else {
        EXPECT_TRUE(signal_desc.can_write());
        EXPECT_TRUE(signal_desc.can_read());
      }
    }
  }

  // Sets both the signals to true
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});
    desired_signals.mutable_values()->insert({"bool_var2", true_value()});

    EXPECT_OK(client.Write(desired_signals));
  }

  // Signals should now be read as true
  {
    intrinsic_proto::gpio::v1::SignalValueSet request;
    request.mutable_values()->insert({"bool_var1", true_value()});
    request.mutable_values()->insert({"bool_var2", true_value()});

    EXPECT_THAT(client.ReadAndMatch(request), IsOkAndHolds(true));
  }

  // Write different values to signals
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", false_value()});
    desired_signals.mutable_values()->insert({"bool_var2", true_value()});

    EXPECT_OK(client.Write(desired_signals));
  }

  // Verify that read signal values match with expected values
  {
    ::intrinsic_proto::gpio::v1::SignalValueSet request;
    request.mutable_values()->insert({"bool_var1", false_value()});
    request.mutable_values()->insert({"bool_var2", true_value()});

    EXPECT_THAT(client.ReadAndMatch(request), IsOkAndHolds(true));
  }

  // Verify that writing to an scalar and array signals works.
  {
    ::intrinsic_proto::gpio::v1::SignalValueSet request;
    request.mutable_values()->insert({"bool_var1", false_value()});
    request.mutable_values()->insert({"array_bool3.0", true_value()});
    request.mutable_values()->insert({"array_bool3.2", true_value()});

    ASSERT_OK(client.Write(request));
  }

  // Sleeps for non-trivial amount of time to verify that rpc calls don't
  // timeout
  absl::SleepFor(absl::Seconds(5));

  // Write to only one signal now
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});

    EXPECT_OK(client.Write(desired_signals));
  }

  // Verify that read signal values match with expected values
  {
    ::intrinsic_proto::gpio::v1::SignalValueSet request;
    request.mutable_values()->insert({"bool_var1", true_value()});
    request.mutable_values()->insert({"bool_var2", true_value()});

    EXPECT_THAT(client.ReadAndMatch(request), IsOkAndHolds(true));
  }

  // Can't manipulate a signal that was not initially passed in GPIOClient
  // constructor
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var3", true_value()});

    EXPECT_THAT(client.Write(desired_signals),
                StatusIs(absl::StatusCode::kNotFound));
  }

  // Can't read a signal that does not exist on opcua server
  {
    ::intrinsic_proto::gpio::v1::SignalValueSet request;
    request.mutable_values()->insert({"bool_varFOOBAR", true_value()});

    EXPECT_THAT(client.ReadAndMatch(request),
                StatusIs(absl::StatusCode::kNotFound));
  }

  // But, it is possible to read a signal that was not already passed to
  // GPIOClient constructor
  {
    ::intrinsic_proto::gpio::v1::SignalValueSet request;
    request.mutable_values()->insert({"bool_var3", true_value()});
    EXPECT_OK(client.ReadAndMatch(request));
  }
}

TEST_F(OpcuaGpioServiceTest, GpioClientReadWriteToArrayIndex) {
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "array_bool3.0" /* only array index 0 */});

  // Write to the index 0 of the array signal.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"array_bool3.0", true_value()});
    EXPECT_OK(client.Write(desired_signals));
  }

  // Read the array signal to verify that the index 0 was written to.
  {
    ::intrinsic_proto::gpio::v1::SignalValueSet request;
    request.mutable_values()->insert({"array_bool3.0", true_value()});
    EXPECT_THAT(client.ReadAndMatch(request), IsOkAndHolds(true));
  }

  // Writing to a different index for the same node also works but shouldn't.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"array_bool3.1", true_value()});
    EXPECT_OK(client.Write(desired_signals));
  }
}

TEST_F(OpcuaGpioServiceTest, GpioClientWriteOnlySignals) {
  // Create a GPIO client with the list of signals that it can write to.
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort), {"bool_write_only1"});

  // Write to a write-only signal.
  {
    ::intrinsic_proto::gpio::v1::SignalValueSet request;
    request.mutable_values()->insert({"bool_write_only1", true_value()});
    EXPECT_OK(client.Write(request));
  }

  // Can't read a write-only signal.
  {
    intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("bool_write_only1");
    EXPECT_THAT(client.Read(request),
                StatusIs(absl::StatusCode::kPermissionDenied));
  }
}

TEST_F(OpcuaGpioServiceTest, UseMixedNamesForReadWrite) {
  // Creates a GPIO client with a mixed of node display names and node string
  // identifiers.
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"shorter_display_foo", "longer_string_id_bar"});

  // Creates set for different names for each of the signals.
  const auto signal_names_foo = absl::flat_hash_set<std::string>(
      {"shorter_display_foo", "longer_string_id_foo"});
  const auto signal_names_bar = absl::flat_hash_set<std::string>(
      {"shorter_display_bar", "longer_string_id_bar"});

  // Write: any combination of longer and shorter names should work.
  {
    for (const auto& foo : signal_names_foo) {
      for (const auto& bar : signal_names_bar) {
        intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
        desired_signals.mutable_values()->insert({foo, true_value()});
        desired_signals.mutable_values()->insert({bar, true_value()});
        EXPECT_OK(client.Write(desired_signals));
      }
    }
  }

  // Read: any combination of longer and shorter names should work.
  {
    for (const auto& foo : signal_names_foo) {
      for (const auto& bar : signal_names_bar) {
        ::intrinsic_proto::gpio::v1::ReadSignalsRequest request;
        request.add_signal_names(foo);
        request.add_signal_names(bar);

        // The response should contain the names that we queried for.
        ::intrinsic_proto::gpio::v1::ReadSignalsResponse expected_response;
        expected_response.mutable_signal_values()->mutable_values()->insert(
            {foo, true_value()});
        expected_response.mutable_signal_values()->mutable_values()->insert(
            {bar, true_value()});
        auto response = client.Read(request);
        ASSERT_OK(response);
        EXPECT_THAT(response->signal_values(),
                    EqualsProto(expected_response.signal_values()));
      }
    }
  }

  // ReadAndMatch: any combination of longer and shorter names should work.
  {
    for (const auto& foo : signal_names_foo) {
      for (const auto& bar : signal_names_bar) {
        intrinsic_proto::gpio::v1::SignalValueSet request;
        request.mutable_values()->insert({foo, true_value()});
        request.mutable_values()->insert({bar, true_value()});

        EXPECT_THAT(client.ReadAndMatch(request), IsOkAndHolds(true));
      }
    }
  }
}

TEST_F(OpcuaGpioServiceTest, GpioClientReadWriteInvalidSignalsToClaim) {
  // Creates client with invalid signal.
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "bool_var_invalid_signal"});

  // Claiming write signals should fail even if desired signal is valid.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});

    EXPECT_THAT(client.Write(desired_signals),
                StatusIs(absl::StatusCode::kNotFound));
  }

  // Claiming write signals show fail again.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert(
        {"some_random_signal", true_value()});

    EXPECT_THAT(client.Write(desired_signals),
                StatusIs(absl::StatusCode::kNotFound));
  }
}

TEST_F(OpcuaGpioServiceTest, GpioClientReadWriteInvalidSignalType) {
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "bool_var2"});

  // This should succeed.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});

    EXPECT_OK(client.Write(desired_signals));
  }

  // This should fail because the signal value type does not match.
  {
    auto IntValue = [](int v) {
      intrinsic_proto::gpio::v1::SignalValue value;
      value.set_int_value(v);
      return value;
    };

    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert(
        {"some_random_signal", IntValue(121)});

    EXPECT_THAT(client.Write(desired_signals),
                StatusIs(absl::StatusCode::kNotFound));
  }
}

TEST_F(OpcuaGpioServiceTest, GpioClientWaitForValue) {
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "bool_var2", "array_bool3"});

  // First set `bool_var1` to true.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});
    EXPECT_OK(client.Write(desired_signals));
  }

  // Waiting for `bool_var1` to be false should timeout.
  {
    intrinsic_proto::gpio::v1::WaitForValueRequest request;
    request.mutable_all_of()->mutable_values()->insert(
        {"bool_var1", false_value()});

    EXPECT_THAT(client.WaitForValue(request, absl::Seconds(1)),
                StatusIs(absl::StatusCode::kDeadlineExceeded));
  }

  // This should succeed as `bool_var1` is true.
  {
    intrinsic_proto::gpio::v1::WaitForValueRequest request;
    request.mutable_all_of()->mutable_values()->insert(
        {"bool_var1", true_value()});

    EXPECT_OK(client.WaitForValue(request, absl::Seconds(1)));
  }

  // Waiting for `array_bool3.0` to be false should timeout.
  {
    intrinsic_proto::gpio::v1::WaitForValueRequest request;
    request.mutable_all_of()->mutable_values()->insert(
        {"array_bool3.0", false_value()});

    EXPECT_THAT(client.WaitForValue(request, absl::Seconds(1)),
                StatusIs(absl::StatusCode::kDeadlineExceeded));
  }

  // Now set `bool_var2` to true.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var2", true_value()});
    EXPECT_OK(client.Write(desired_signals));
  }

  // `any_of` with correct value for `bool_var1` should pass.
  {
    intrinsic_proto::gpio::v1::WaitForValueRequest request;
    request.mutable_any_of()->mutable_values()->insert(
        {"bool_var1", true_value()});  // should match
    request.mutable_any_of()->mutable_values()->insert(
        {"bool_var2", false_value()});  // should not match
    request.mutable_any_of()->mutable_values()->insert(
        {"array_bool3.0", false_value()});  // should not match

    const auto response = client.WaitForValue(request, absl::Seconds(1));
    ASSERT_OK(response);

    SignalValueSet expected_response;
    expected_response.mutable_values()->insert({"bool_var1", true_value()});
    expected_response.mutable_values()->insert({"bool_var2", true_value()});
    expected_response.mutable_values()->insert({"array_bool3.0", true_value()});

    EXPECT_THAT(response->values(), EqualsProto(expected_response));
  }

  // This `any_of` should fail.
  {
    intrinsic_proto::gpio::v1::WaitForValueRequest request;
    request.mutable_any_of()->mutable_values()->insert(
        {"bool_var1", false_value()});  // should not match
    request.mutable_any_of()->mutable_values()->insert(
        {"bool_var2", false_value()});  // should not match

    EXPECT_THAT(client.WaitForValue(request, absl::Seconds(1)),
                StatusIs(absl::StatusCode::kDeadlineExceeded));
  }

  // `all_of` with correct values for `bool_var1` and `bool_var2` should pass.
  {
    intrinsic_proto::gpio::v1::WaitForValueRequest request;
    request.mutable_all_of()->mutable_values()->insert(
        {"bool_var1", true_value()});  // should match
    request.mutable_all_of()->mutable_values()->insert(
        {"bool_var2", true_value()});  // should match
    request.mutable_all_of()->mutable_values()->insert(
        {"array_bool3.2", true_value()});  // should match

    const auto response = client.WaitForValue(request, absl::Seconds(1));
    ASSERT_OK(response);

    SignalValueSet expected_response;
    expected_response.mutable_values()->insert({"bool_var1", true_value()});
    expected_response.mutable_values()->insert({"bool_var2", true_value()});
    expected_response.mutable_values()->insert({"array_bool3.2", true_value()});

    EXPECT_THAT(response->values(), EqualsProto(expected_response));
  }

  // This `all_of` should fail.
  {
    intrinsic_proto::gpio::v1::WaitForValueRequest request;
    request.mutable_all_of()->mutable_values()->insert(
        {"bool_var1", true_value()});  // should match
    request.mutable_all_of()->mutable_values()->insert(
        {"bool_var2", false_value()});  // should not match

    EXPECT_THAT(client.WaitForValue(request, absl::Seconds(1)),
                StatusIs(absl::StatusCode::kDeadlineExceeded));
  }
}

TEST_F(OpcuaGpioServiceTest, GpioClientWaitForValueTimesOut) {
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort), {"bool_var1"});

  intrinsic_proto::gpio::v1::WaitForValueRequest request;
  request.mutable_all_of()->mutable_values()->insert(
      {"bool_var1", true_value()});

  EXPECT_THAT(client.WaitForValue(request, absl::ZeroDuration()),
              StatusIs(absl::StatusCode::kDeadlineExceeded));
}

TEST_F(OpcuaGpioServiceTest, GpioClientWriteBadSession) {
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "bool_var2"});

  // Shutdowns the opcua server to simulate the behavior of server closing the
  // client session.
  ShutDownOpcuaServer();

  // Uses valid signals so that opcua_gpio_service tries to open a new session
  // with the opcua server and gets a connection error.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});
    desired_signals.mutable_values()->insert({"bool_var2", true_value()});

    EXPECT_THAT(
        client.Write(desired_signals),
        StatusIs(absl::StatusCode::kInternal, HasSubstr("Connection error")));
  }

  // Retrying should fail again and there shouldn't be any crash due to stale
  // client context from previous failed call.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});

    EXPECT_THAT(
        client.Write(desired_signals),
        StatusIs(absl::StatusCode::kInternal, HasSubstr("Connection error")));
  }
}

TEST_F(OpcuaGpioServiceTest, GpioClientReconnectOnDroppedConnection) {
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "bool_var2"});

  // Write & Read should succeed as opcua server is running.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});
    ASSERT_OK(client.Write(desired_signals));

    intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("bool_var2");
    ASSERT_OK(client.Read(request));
  }

  // Shutdowns the opcua server to simulate the behavior of server closing the
  // client session.
  ShutDownOpcuaServer();

  // Read & Write should fail as opcua server is no longer running.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});
    EXPECT_THAT(client.Write(desired_signals),
                StatusIs(absl::StatusCode::kInternal,
                         HasSubstr("Writing to nodes failed with service "
                                   "result")));

    intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("bool_var2");
    EXPECT_THAT(client.Read(request),
                StatusIs(absl::StatusCode::kDeadlineExceeded));
  }

  // Spins up the opcua server again.
  SpinUpOpcuaServer();

  // Read & Write should succeed now. Internally UA_Client should attempt to
  // reconnect to the opcua server.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});
    EXPECT_OK(client.Write(desired_signals));

    intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("bool_var2");
    EXPECT_OK(client.Read(request));
  }
}

TEST_F(OpcuaGpioServiceTest, GpioClientWriteBadSessionAfterInitialConnection) {
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "bool_var2"});

  // First call should succeed in establishing a connection.
  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", true_value()});
    desired_signals.mutable_values()->insert({"bool_var2", true_value()});

    EXPECT_OK(client.Write(desired_signals));
  }

  // Shutdowns the opcua server to simulate client disconnect.
  ShutDownOpcuaServer();

  // Writes should fail now with a failure to write to nodes.
  {
    for (int i = 0; i < 3; i++) {
      intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
      desired_signals.mutable_values()->insert({"bool_var1", true_value()});

      EXPECT_THAT(client.Write(desired_signals),
                  StatusIs(absl::StatusCode::kInternal,
                           HasSubstr("Writing to nodes failed with service "
                                     "result")));
    }
  }

  // Reads should fail also.
  {
    for (int i = 0; i < 3; i++) {
      intrinsic_proto::gpio::v1::ReadSignalsRequest request;
      request.add_signal_names("bool_var1");
      EXPECT_THAT(client.Read(request),
                  StatusIs(absl::StatusCode::kDeadlineExceeded));
    }
  }
}
}  // namespace intrinsic::opcua
