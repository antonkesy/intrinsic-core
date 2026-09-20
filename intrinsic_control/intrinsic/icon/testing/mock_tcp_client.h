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

#ifndef INTRINSIC_ICON_TESTING_MOCK_TCP_CLIENT_H_
#define INTRINSIC_ICON_TESTING_MOCK_TCP_CLIENT_H_

#include <gmock/gmock.h>

#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/tcp_server_client.h"

namespace intrinsic::kuka::testing {

class MockTcpClient : public icon::TcpClient {
 public:
  MockTcpClient() : icon::TcpClient("localhost", 12345, "MockTcpClient") {}
  MOCK_METHOD(absl::Status, Connect, (absl::Duration timeout), (override));

  MOCK_METHOD(void, Disconnect, (), (override));

  MOCK_METHOD(icon::RealtimeStatus, Send,
              (absl::Span<const uint8_t> bytes, int send_flags), (override));

  MOCK_METHOD(icon::RealtimeStatus, Receive,
              (absl::Duration timeout, bool recv_until_buffer_full,
               int recv_flags, absl::Span<uint8_t> buffer, size_t& bytes_read),
              (override));

  MOCK_METHOD(bool, IsConnected, (), (override));
};

}  // namespace intrinsic::kuka::testing

#endif  // INTRINSIC_ICON_TESTING_MOCK_TCP_CLIENT_H_
