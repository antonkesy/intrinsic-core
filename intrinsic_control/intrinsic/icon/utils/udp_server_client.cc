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

#include "intrinsic/icon/utils/udp_server_client.h"

#include <arpa/inet.h>
#include <linux/errqueue.h>
#include <linux/net_tstamp.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/strerror.h"

namespace intrinsic::icon {

namespace {

absl::Status EnableTimestamping(int socket) {
  // Set the Socket Options to request timestamps of time of packet arrival
  constexpr int flags = SOF_TIMESTAMPING_SOFTWARE;

  if (auto ret = setsockopt(socket, SOL_SOCKET, SO_TIMESTAMPING, &flags,
                            sizeof(flags));
      ret < 0) {
    return absl::InternalError(
        absl::StrCat("Failed to configure UDP socket for receive-timestamps: ",
                     ::strerror(errno)));
  }
  return absl::OkStatus();
}
}  // namespace

UdpEndpoint::UdpEndpoint(const absl::string_view address, const uint16_t port)
    : address_{address}, port_{port}, socket_{-1} {};

UdpEndpoint::~UdpEndpoint() {
  [[maybe_unused]] const auto status{Disconnect()};
  return;
}

UdpEndpoint::UdpEndpoint(UdpEndpoint&& other) noexcept
    : address_{std::move(other.address_)},
      port_{std::exchange(other.port_, 0)},
      socket_{std::exchange(other.socket_, -1)} {}

UdpEndpoint& UdpEndpoint::operator=(UdpEndpoint&& other) noexcept {
  if (this != &other) {
    // Make sure to close our own socket before moving.
    [[maybe_unused]] auto status = Disconnect();

    address_ = std::move(other.address_);
    port_ = std::exchange(other.port_, 0);
    socket_ = std::exchange(other.socket_, -1);
  }
  return *this;
}

absl::Status UdpEndpoint::Disconnect() {
  if (socket_ >= 0) {
    const int err = ::close(socket_);
    const int old_socket = socket_;
    // No matter what the result of close is, we should not try to close the
    // socket again.
    socket_ = -1;
    if (err == -1) {
      return absl::InternalError(
          absl::StrCat("Failed to close UDP socket ", old_socket,
                       " with error: ", ::strerror(errno)));
    }
  }
  return absl::OkStatus();
}

RealtimeStatusOr<int64_t> UdpEndpoint::SendTo(absl::Span<const uint8_t> buffer,
                                              const ::sockaddr_in& dest_addr) {
  if (!IsConnected()) {
    return icon::FailedPreconditionError(
        "You need to call Connect() before calling SendTo()");
  }
  const int64_t sent_size{::sendto(
      socket_, buffer.data(), buffer.size(), 0,
      reinterpret_cast<const ::sockaddr*>(&dest_addr), sizeof(dest_addr))};
  if (sent_size == -1) {
    return icon::UnavailableError(RealtimeStatus::StrCat(
        "Failed to send to UDP connection: ", icon::StrError(errno)));
  }
  return sent_size;
}

RealtimeStatusOr<int64_t> UdpEndpoint::ReceiveFrom(
    absl::Span<uint8_t> buffer, const absl::Duration timeout,
    std::optional<absl::Time>& receive_time, ::sockaddr_in* src_addr) {
  if (!IsConnected()) {
    return icon::FailedPreconditionError(
        "You need to call Connect() before calling ReceiveFrom()");
  }
  // If a non-zero timeout is specified, wait until data is available on the
  // socket or return after timeout.
  if (timeout > absl::ZeroDuration()) {
    const ::timespec timeout_spec = absl::ToTimespec(timeout);
    ::pollfd pfd = {.fd = socket_, .events = POLLIN};
    // ppoll for better timeout resolution.
    const auto result{::ppoll(&pfd, 1, &timeout_spec, /*sigmask=*/nullptr)};
    if (result == 0) {
      return icon::DeadlineExceededError(RealtimeStatus::StrCat(
          "UDP read timed out: ", icon::StrError(errno)));
    }
    if (result < 0) {
      return icon::InternalError(RealtimeStatus::StrCat(
          "Polling on UDP socket failed: ", icon::StrError(errno)));
    }
  }

  auto const buffer_size{buffer.size()};
  // The buffer has to be aligned correctly.
  alignas(
      ::cmsghdr) char control_buffer[CMSG_SPACE(sizeof(::scm_timestamping))];
  ::iovec iov{.iov_base = buffer.data(), .iov_len = buffer_size};
  ::msghdr msg{.msg_name = nullptr,
               .msg_namelen = 0,
               .msg_iov = &iov,
               .msg_iovlen = 1,
               .msg_control = control_buffer,
               .msg_controllen = sizeof(control_buffer)};
  // Get the source addr if a valid pointer is provided.
  if (src_addr != nullptr) {
    msg.msg_name = src_addr;
    msg.msg_namelen = sizeof(*src_addr);
  }
  const int64_t received_size{
      ::recvmsg(socket_, &msg, MSG_DONTWAIT | MSG_TRUNC)};
  if (received_size < 0) {
    // Since the socket is non-blocking, we will get EAGAIN or EWOULDBLOCK
    // when no data is available.
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return icon::DeadlineExceededError(RealtimeStatus::StrCat(
          "Deadline exceeded waiting for UDP data: ", icon::StrError(errno)));
    }
    return icon::UnavailableError(RealtimeStatus::StrCat(
        "Failed to receive from UDP connection: ", icon::StrError(errno)));
  } else if (received_size > buffer_size) {
    return icon::InternalError(RealtimeStatus::StrCat(
        "Truncation detected: Provided buffer (", buffer_size,
        ") smaller than received data (", received_size, ")"));
  }

  // The following will timestamp the packet in case this is possible. On a
  // loopback this will not work.
  if (!(msg.msg_flags & MSG_CTRUNC)) {
    for (::cmsghdr* cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL;
         cmsg = CMSG_NXTHDR(&msg, cmsg)) {
      if (cmsg->cmsg_level == SOL_SOCKET &&
          cmsg->cmsg_type == SO_TIMESTAMPING) {
        if (cmsg->cmsg_len < CMSG_LEN(sizeof(::scm_timestamping))) {
          continue;
        }
        // The kernel returns an array of 3 timespecs:
        // [0] Software, [1] Deprecated, [2] Hardware
        // The software timestamp should be the timestamp taken in the kernel,
        // not the IRQ task. Therefore, it is not delayed by other running RT
        // tasks.
        // The timestamp are usually reported as CLOCK_REALTIME.
        ::scm_timestamping ts_arr;
        std::memcpy(&ts_arr, CMSG_DATA(cmsg), sizeof(ts_arr));
        if (ts_arr.ts[0].tv_sec != 0 || ts_arr.ts[0].tv_nsec != 0) {
          receive_time = absl::TimeFromTimespec(ts_arr.ts[0]);
        }
        break;
      }
    }
  }
  return received_size;
}

UdpClient::UdpClient(const absl::string_view address, const uint16_t port)
    : UdpEndpoint{address, port} {}

absl::Status UdpClient::Connect() {
  if (address_.empty()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "'", address_, "' is not a valid IP for UDP communication"));
  }
  socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_ < 0) {
    return absl::InternalError(
        absl::StrCat("Error opening UDP socket: ", ::strerror(errno)));
  }

  INTR_RETURN_IF_ERROR(EnableTimestamping(socket_));

  ::memset(&rc_addr_, 0, sizeof(rc_addr_));
  rc_addr_.sin_family = AF_INET;
  rc_addr_.sin_port = ::htons(port_);

  if (::inet_pton(AF_INET, address_.c_str(), &rc_addr_.sin_addr) != 1) {
    [[maybe_unused]] const auto status{Disconnect()};
    return absl::InvalidArgumentError(
        absl::StrCat("'", address_, "' is not a valid IPv4 address"));
  }
  constexpr int dont_route{1};
  if (::setsockopt(socket_, SOL_SOCKET, SO_DONTROUTE, &dont_route,
                   sizeof(dont_route)) == -1) {
    return absl::InternalError(absl::StrCat(
        "Error setting socket option 'SO_DONTROUTE': ", ::strerror(errno)));
  }
  constexpr int reuse{1};
  if (::setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(int)) ==
      -1) {
    return absl::InternalError(absl::StrCat(
        "Error setting socket option 'SO_REUSEADDR': ", ::strerror(errno)));
  }
  return absl::OkStatus();
}

UdpServer::UdpServer(const absl::string_view address, const uint16_t port,
                     const bool reuse_address)
    : UdpEndpoint{address, port}, reuse_address_{reuse_address} {
  ::memset(&client_addr_, 0, sizeof(client_addr_));
}

absl::Status UdpServer::Connect() {
  if (address_.empty()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "'", address_, "' is not a valid IP for UDP communication"));
  }
  socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_ < 0) {
    return absl::InternalError(
        absl::StrCat("Error opening UDP socket: ", ::strerror(errno)));
  }
  INTR_RETURN_IF_ERROR(EnableTimestamping(socket_));
  ::memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = ::htons(port_);

  if (::inet_pton(AF_INET, address_.c_str(), &server_addr_.sin_addr) != 1) {
    [[maybe_unused]] const auto status{Disconnect()};
    return absl::InvalidArgumentError(
        absl::StrCat("'", address_, "' is not a valid IPv4 address"));
  }

  if (reuse_address_) {
    constexpr int reuse{1};
    if (::setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) <
        0) {
      return absl::InternalError(absl::StrCat(
          "Error setting socket option 'SO_REUSEADDR': ", ::strerror(errno)));
    }
  }

  if (::bind(socket_, reinterpret_cast<::sockaddr*>(&server_addr_),
             sizeof(server_addr_)) < 0)
    return absl::InternalError(absl::StrCat("Error binding to socket ",
                                            address_, ":", port_, ": ",
                                            ::strerror(errno)));
  return absl::OkStatus();
}

}  // namespace intrinsic::icon
