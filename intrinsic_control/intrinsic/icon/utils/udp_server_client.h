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

#ifndef INTRINSIC_ICON_UTILS_UDP_SERVER_CLIENT_H_
#define INTRINSIC_ICON_UTILS_UDP_SERVER_CLIENT_H_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/poll.h>

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <type_traits>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/strerror.h"

namespace intrinsic::icon {

// Base class for UDP client and server holding the common code shared between
// the two.
class UdpEndpoint {
 public:
  UdpEndpoint() = delete;

  // Destructor closes the connection.
  virtual ~UdpEndpoint() INTRINSIC_NON_REALTIME_ONLY;

  // This class is move only since the connection is automatically closed in the
  // destructor.
  UdpEndpoint(const UdpEndpoint& other) = delete;
  UdpEndpoint& operator=(UdpEndpoint& other) = delete;

  UdpEndpoint(UdpEndpoint&& other) noexcept;
  UdpEndpoint& operator=(UdpEndpoint&& other) noexcept;

  // Get the port currently used for communication.
  uint16_t GetPort() const { return port_; }

  // Get the address we are communicating over.
  absl::string_view GetAddress() const { return address_; }

  // Initialize the socket for communication.
  // This differs in between client and server.
  virtual absl::Status Connect() INTRINSIC_NON_REALTIME_ONLY = 0;

  // Close the connection.
  //
  // InternalError is returned in case the disconnect fails.
  absl::Status Disconnect() INTRINSIC_NON_REALTIME_ONLY;

  // Check whether the client has a valid connection.
  bool IsConnected() const { return socket_ >= 0; }

  // Send the given contents of the buffer via the socket.
  // This function is non-blocking. Before this function can be called, the user
  // must have called `Connect()` successfully before.
  // The implementation is different for client and server.
  virtual RealtimeStatusOr<int64_t> Send(absl::Span<const uint8_t> buffer)
      INTRINSIC_CHECK_REALTIME_SAFE = 0;

  // Receive to a buffer over UDP. If supported by the network interface the
  // receive hardware timestamp will be returned  in `receive_time`. When using
  // the loopback device it is expected that no timestamp is returned. In case
  // of success the number of bytes received will be returned. Before this
  // function can be called `Connect()` has to be called successfully.
  // The implementation is different for client and server.
  virtual RealtimeStatusOr<int64_t> Receive(
      absl::Span<uint8_t> buffer, absl::Duration timeout,
      std::optional<absl::Time>& receive_time)
      INTRINSIC_CHECK_REALTIME_SAFE = 0;

  // Convenient overload that ignores the receive time.
  RealtimeStatusOr<int64_t> Receive(absl::Span<uint8_t> buffer,
                                    absl::Duration timeout)
      INTRINSIC_CHECK_REALTIME_SAFE {
    std::optional<absl::Time> ignored_receive_time;
    return Receive(buffer, timeout, ignored_receive_time);
  }

 protected:
  // Creates an inactive UDP connection. In order to open the connection the
  // OpenSocket function has to be called.
  UdpEndpoint(absl::string_view address,
              uint16_t port) INTRINSIC_NON_REALTIME_ONLY;

  // Get the underlying socket file descriptor.
  int GetSocketFileDescriptor() const { return socket_; }

  // Underlying implementation for `Send()`. The `dest_addr` specifies the
  // address that the contents of the buffer should be sent to.
  //
  // Additionally to `Send()` returns `UnavailableError` in case sending fails.
  RealtimeStatusOr<int64_t> SendTo(absl::Span<const uint8_t> buffer,
                                   const ::sockaddr_in& dest_addr)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Send a specific packet over UDP.
  // This is a convenience function that will call the underlying `Send()`
  // function. The `dest_addr` specifies the address that the contents of the
  // buffer should be sent to. The caller has to make sure themselves that the
  // packet respects network order (e.g. by using the BigEndian<T> wrapper).
  //
  // Additionally to the `Send()` function it returns `InternalError` in case
  // insufficient bytes were sent.
  template <typename T>
    requires std::is_trivially_copyable_v<T>
  RealtimeStatus SendPacketTo(const T& packet, const ::sockaddr_in& dest_addr)
      INTRINSIC_CHECK_REALTIME_SAFE {
    constexpr auto packet_size{sizeof(T)};
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const auto sent_size,
        SendTo(absl::MakeConstSpan(reinterpret_cast<const uint8_t*>(&packet),
                                   sizeof(T)),
               dest_addr));
    if (sent_size != packet_size) {
      return icon::InternalError(RealtimeStatus::StrCat(
          "Insufficient bytes sent (expected ", packet_size, ", sent ",
          sent_size, "): ", icon::StrError(errno)));
    }
    return OkStatus();
  }

  // Receive to a buffer over UDP. If supported by the network interface the
  // receive hardware timestamp will be returned. When using the loopback
  // device it is expected that no timestamp is returned. The source addr that
  // the request has been received from will be returned in `src_addr` if a
  // valid pointer is provided. Before this function can be called `Connect()`
  // has to be called successfully.
  //
  // Returns a `DeadlineExceededError` in case the connection timed out. In case
  // no data could be read `UnavailableError` is returned and in case polling
  // fails or truncation is detected in `InternalError` is returned. In case of
  // success this returns the number of bytes received.
  RealtimeStatusOr<int64_t> ReceiveFrom(
      absl::Span<uint8_t> buffer, absl::Duration timeout,
      std::optional<absl::Time>& receive_time,
      ::sockaddr_in* src_addr = nullptr) INTRINSIC_CHECK_REALTIME_SAFE;

  // Convenience overload for reading a packet over UDP to **packed** struct.
  //
  // Additionally to the overload above it returns an `InternalError` in case
  // the received size and the size of the class mismatch.
  template <typename T>
    requires(std::is_trivially_copyable_v<T> &&
             std::is_default_constructible_v<T>)
  RealtimeStatusOr<T> ReceivePacketFrom(
      const absl::Duration timeout, std::optional<absl::Time>& receive_time,
      ::sockaddr_in* src_addr = nullptr) INTRINSIC_CHECK_REALTIME_SAFE {
    constexpr auto packet_size{sizeof(T)};
    T packet;
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const auto received_size,
        ReceiveFrom(
            absl::MakeSpan(reinterpret_cast<uint8_t*>(&packet), packet_size),
            timeout, receive_time, src_addr));
    if (received_size != packet_size) {
      return icon::InternalError(RealtimeStatus::StrCat(
          "Insufficient bytes received (expected ", packet_size, ", received ",
          received_size, "): ", icon::StrError(errno)));
    }
    return packet;
  }

  std::string address_;
  uint16_t port_;
  int socket_;
};

// UDP client implementation based on UDP endpoint.
class UdpClient : public UdpEndpoint {
 public:
  // Creates an inactive UDP connection. In order to open the connection the
  // OpenSocket function has to be called.
  UdpClient(absl::string_view address,
            uint16_t port) INTRINSIC_NON_REALTIME_ONLY;

  // Initializes and configures a UDP socket for IPv4 communication.
  //
  // `InvalidArgumentError` is returned in case no valid IP was provided and
  // `InternalError` is returned in case the connection fails or we fail to set
  // the socket options.
  absl::Status Connect() override INTRINSIC_NON_REALTIME_ONLY;

  RealtimeStatusOr<int64_t> Send(absl::Span<const uint8_t> buffer) override
      INTRINSIC_CHECK_REALTIME_SAFE {
    return SendTo(buffer, rc_addr_);
  }

  template <typename T>
  RealtimeStatus SendPacket(const T& packet) INTRINSIC_CHECK_REALTIME_SAFE {
    return SendPacketTo(packet, rc_addr_);
  }

  // Workaround for name hiding due to virtual method with the same name.
  using UdpEndpoint::Receive;

  RealtimeStatusOr<int64_t> Receive(absl::Span<uint8_t> buffer,
                                    absl::Duration timeout,
                                    std::optional<absl::Time>& receive_time)
      override INTRINSIC_CHECK_REALTIME_SAFE {
    return ReceiveFrom(buffer, timeout, receive_time);
  }

  template <typename T>
  RealtimeStatusOr<T> ReceivePacket(const absl::Duration timeout,
                                    std::optional<absl::Time>& receive_time)
      INTRINSIC_CHECK_REALTIME_SAFE {
    return ReceivePacketFrom<T>(timeout, receive_time);
  }

  template <typename T>
  RealtimeStatusOr<T> ReceivePacket(const absl::Duration timeout)
      INTRINSIC_CHECK_REALTIME_SAFE {
    std::optional<absl::Time> ignored_receive_time;
    return ReceivePacket<T>(timeout, ignored_receive_time);
  }

 private:
  ::sockaddr_in rc_addr_;
};

// A simple UDP server which can only have a single client.
// The method `Receive()` must be called before `Send()` since it registers the
// address of the client.
class UdpServer : public UdpEndpoint {
 public:
  UdpServer(absl::string_view local_host, uint16_t local_port,
            bool reuse_address = true) INTRINSIC_NON_REALTIME_ONLY;

  // Initializes the UDP server by creating and binding a socket.
  //
  // `InvalidArgumentError` is returned in case no valid IP was provided and
  // `InternalError` is returned in case the connection fails or we fail to set
  // the socket options.
  absl::Status Connect() override INTRINSIC_NON_REALTIME_ONLY;

  // Additionally to the errors from `SendTo()` it returns a
  // `FailedPreconditionError` in case no `Receive()` has been called to fill
  // the client address.
  RealtimeStatusOr<int64_t> Send(absl::Span<const uint8_t> buffer) override
      INTRINSIC_CHECK_REALTIME_SAFE {
    if (client_addr_.sin_port == 0) {
      return icon::FailedPreconditionError(
          "You need to call Receive() successfully before Send() to fill the "
          "client address.");
    }
    return SendTo(buffer, client_addr_);
  }

  template <typename T>
  RealtimeStatus SendPacket(const T& packet) INTRINSIC_CHECK_REALTIME_SAFE {
    return SendPacketTo(packet, client_addr_);
  }

  // Workaround for name hiding due to virtual method with the same name.
  using UdpEndpoint::Receive;

  RealtimeStatusOr<int64_t> Receive(absl::Span<uint8_t> buffer,
                                    absl::Duration timeout,
                                    std::optional<absl::Time>& receive_time)
      override INTRINSIC_CHECK_REALTIME_SAFE {
    return ReceiveFrom(buffer, timeout, receive_time, &client_addr_);
  }

  template <typename T>
  RealtimeStatusOr<T> ReceivePacket(const absl::Duration timeout,
                                    std::optional<absl::Time>& receive_time)
      INTRINSIC_CHECK_REALTIME_SAFE {
    return ReceivePacketFrom<T>(timeout, receive_time, &client_addr_);
  }

  template <typename T>
  RealtimeStatusOr<T> ReceivePacket(const absl::Duration timeout)
      INTRINSIC_CHECK_REALTIME_SAFE {
    std::optional<absl::Time> ignored_receive_time;
    return ReceivePacket<T>(timeout, ignored_receive_time);
  }

 protected:
  bool reuse_address_;
  ::sockaddr_in server_addr_;
  ::sockaddr_in client_addr_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_UTILS_UDP_SERVER_CLIENT_H_
