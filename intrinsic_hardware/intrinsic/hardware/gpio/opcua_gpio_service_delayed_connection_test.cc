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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <tuple>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/server.h"
#include "internal/testing.h"
#include "intrinsic/hardware/gpio/gpio_client.h"
#include "intrinsic/hardware/gpio/gpio_service_proto_utils.h"
#include "intrinsic/hardware/gpio/opcua_gpio_service.h"
#include "intrinsic/hardware/gpio/opcua_gpio_service_config.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/hardware/opcua/opcua_server.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/grpc/grpc.h"
#include "open62541/types.h"

using ::absl_testing::StatusIs;
using ::intrinsic_proto::gpio::v1::SignalValue;
using ::testing::Pair;
using ::testing::UnorderedElementsAre;

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

// Overview of the different entities involved in this test suite:
// The OPCUA GPIO server is configured and started in the `SetUp` method while
// the OPCUA GPIO client is started as part of the test. The OPCUA server
// itself is started later, during the test, and the tests confirm that the
// OPCUA GPIO server can automatically connect to it after startup.
//
//  +---------------+          +---------------+          +-----------------+
//  |               |          |               |          |                 |
//  |    OPCUA      |          |   OPCUA GPIO  |          |   OPCUA GPIO    |
//  |    Server     |<-delayed-+     Server    |<---------+     Client      |
//  | (port: 4840)  |          | (port: 17219) |          |                 |
//  |               |          |               |          |                 |
//  +---------------+          +---------------+          +-----------------+
//
class StartWithoutOpcuaServerTest : public ::testing::Test {
 public:
  void SetUp() override { CreateOpcuaGpioService(); }

 protected:
  void SpinUpOpcuaServer() {
    opcua_server_ = std::make_unique<::intrinsic::opcua::OpcuaServer>();

    // Starts an opcua server with scalar values in namespace=1.
    auto [status, root_node] = opcua_server_->AddObjectNode("root_node");
    ASSERT_EQ(status, UA_STATUSCODE_GOOD);
    EXPECT_EQ(root_node.namespaceIndex, 0);
    EXPECT_EQ(root_node.identifierType, UA_NODEIDTYPE_STRING);

    // Adds read-write boolean scalar variables.
    for (auto v : {1, 2}) {
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

    ASSERT_EQ(opcua_server_->StartServerAsync(), UA_STATUSCODE_GOOD);
  }

  void ShutDownOpcuaServer() { opcua_server_->StopServerAndWait(); }

  void CreateOpcuaGpioService() {
    intrinsic_proto::gpio::OpcuaGpioServiceConfig opcua_gpio_srv_config;
    opcua_gpio_srv_config.set_opcua_server_address(
        absl::StrCat("opc.tcp://localhost:", kOpcuaServerPort));
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=bool_var1");
    opcua_gpio_srv_config.mutable_opcua_nodes()->add_node_id(
        "ns=1;s=bool_var2");

    opcua_gpio_service_ =
        intrinsic::MakeOpcuaGPIOService("plc", opcua_gpio_srv_config);
    ASSERT_OK(opcua_gpio_service_);

    opcua_gpio_server_ = intrinsic::CreateServer(kOpcuaGpioServicePort,
                                                 {opcua_gpio_service_->get()});
    ASSERT_OK(opcua_gpio_server_);
  }

  static const int kOpcuaServerPort = 4840;
  static const int kOpcuaGpioServicePort = 17219;

  std::unique_ptr<::intrinsic::opcua::OpcuaServer> opcua_server_;
  absl::StatusOr<
      std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::Service>>
      opcua_gpio_service_;
  absl::StatusOr<std::unique_ptr<::grpc::Server>> opcua_gpio_server_;
};

// Without the OPCUA server started, check that calls fail as expected.
TEST_F(StartWithoutOpcuaServerTest, DelayedConnectionToOpcuaServer) {
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "bool_var2"});

  // Shouldn't be able to retrieve the signal descriptions.
  EXPECT_THAT(client.GetSignalDescriptions(),
              StatusIs(absl::StatusCode::kInternal));

  // Reads and writes should fail.
  for (int i = 0; i < 3; i++) {
    {
      intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
      desired_signals.mutable_values()->insert({"bool_var1", true_value()});
      desired_signals.mutable_values()->insert({"bool_var2", true_value()});
      EXPECT_THAT(client.Write(desired_signals),
                  StatusIs(absl::StatusCode::kInternal));
    }
    {
      intrinsic_proto::gpio::v1::ReadSignalsRequest request;
      request.add_signal_names("bool_var1");
      EXPECT_THAT(client.Read(request), StatusIs(absl::StatusCode::kInternal));
    }
  }

  // Waiting for a value should timeout.
  {
    intrinsic_proto::gpio::v1::WaitForValueRequest request;
    request.mutable_all_of()->mutable_values()->insert(
        {"bool_var1", false_value()});
    EXPECT_THAT(client.WaitForValue(request, absl::Seconds(1)),
                StatusIs(absl::StatusCode::kDeadlineExceeded));
  }

  // Now, bring the server up.
  SpinUpOpcuaServer();

  // Give some time for the connection thread to connect.
  absl::SleepFor(absl::Seconds(2));

  // We can now call the various functions successfully.

  ASSERT_OK_AND_ASSIGN(auto known_signals, client.GetSignalDescriptions());
  EXPECT_EQ(known_signals.signal_descriptions_size(), 2);

  {
    intrinsic_proto::gpio::v1::SignalValueSet desired_signals;
    desired_signals.mutable_values()->insert({"bool_var1", false_value()});
    desired_signals.mutable_values()->insert({"bool_var2", false_value()});
    EXPECT_OK(client.Write(desired_signals));
  }
  {
    ::intrinsic_proto::gpio::v1::ReadSignalsResponse expected_response;
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"bool_var1", false_value()});
    expected_response.mutable_signal_values()->mutable_values()->insert(
        {"bool_var2", false_value()});

    intrinsic_proto::gpio::v1::ReadSignalsRequest request;
    request.add_signal_names("bool_var1");
    request.add_signal_names("bool_var2");
    auto response = client.Read(request);
    ASSERT_OK(response);
    EXPECT_EQ(response->signal_values(), expected_response.signal_values());
  }
  {
    intrinsic_proto::gpio::v1::WaitForValueRequest request;
    request.mutable_all_of()->mutable_values()->insert(
        {"bool_var1", false_value()});
    request.mutable_all_of()->mutable_values()->insert(
        {"bool_var2", false_value()});
    const auto response = client.WaitForValue(request, absl::Seconds(1));
    ASSERT_OK(response);
    EXPECT_THAT(response->values().values(),
                UnorderedElementsAre(Pair("bool_var1", false_value()),
                                     Pair("bool_var2", false_value())));
  }
}

TEST_F(StartWithoutOpcuaServerTest, ShutDownWithoutConnection) {
  auto client = ::intrinsic::gpio::GPIOClient(
      ConnectionParams::LocalPort(kOpcuaGpioServicePort),
      {"bool_var1", "bool_var2"});

  // Shouldn't be able to retrieve the signal descriptions.
  EXPECT_THAT(client.GetSignalDescriptions(),
              StatusIs(absl::StatusCode::kInternal));

  // Complete the test without connecting to the OPCUA server in order to
  // test the shutdown logic in the OPCUA GPIO server.
}

}  // namespace intrinsic::opcua
