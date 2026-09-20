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

#include "intrinsic/icon/utils/tcp_server_client.h"

#include <arpa/inet.h>
#include <endian.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>

#include "absl/cleanup/cleanup.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "absl/utility/utility.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/strerror.h"

namespace intrinsic::icon {

RealtimeStatus TcpConnection::Send(absl::Span<const uint8_t> bytes,
                                   int send_flags) {
  if (sockfd_ < 0) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        connection_name_,
        ": You need to call Connect() before calling Send()"));
  }
  auto res = send(sockfd_, (const char*)bytes.data(), bytes.size(), send_flags);
  if (res < 0) {
    return UnavailableError(RealtimeStatus::StrCat(
        connection_name_,
        ": Could not send TCP data: ", icon::StrError(errno)));
  }
  if (res == bytes.size()) {
    return OkStatus();
  } else {
    return UnavailableError(RealtimeStatus::StrCat(
        "Only ", res, " of ", bytes.size(), " were sent."));
  }
}

RealtimeStatusOr<bool> TcpConnection::ReceiveSinglePacket(
    absl::Time read_deadline, bool recv_until_buffer_full, int recv_flags,
    absl::Span<uint8_t> buffer, size_t& bytes_read) {
  if (bytes_read >= buffer.size()) {
    return InternalError(RealtimeStatus::StrCat(
        connection_name_, ": Offset ", bytes_read, " exceeds buffer size(",
        buffer.size(), "). This is a bug."));
  }
  ssize_t received = recv(sockfd_, buffer.data() + bytes_read,
                          buffer.size() - bytes_read, recv_flags);
  if (received == -1) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      // We have not read any data until socket timeout -> Retry until
      // `read_deadline`.
      if (absl::Now() > read_deadline) {
        return DeadlineExceededError(RealtimeStatus::StrCat(
            connection_name_, ": TCP receive call exceeded deadline"));
      }
      return false;
    }
    return UnavailableError(RealtimeStatus::StrCat(
        connection_name_, ": Receive error: ", icon::StrError(errno)));
  } else if (received > 0) {
    if (recv_until_buffer_full) {
      if (bytes_read + received > buffer.size()) {
        return InternalError(RealtimeStatus::StrCat(
            connection_name_, ": Offset ", bytes_read + received,
            " exceeds buffer size(", buffer.size(), "). This is a bug."));
      }
      // Return if buffer is fully filled.
      if (bytes_read + received == buffer.size()) {
        bytes_read = buffer.size();
        return true;
      }
      // Otherwise increase number of read bytes and continue.
      bytes_read += received;
      return false;

    } else {  // We return on first package received.
      bytes_read = received;
      return true;
    }
  } else {
    sockfd_ = -1;
    return CancelledError(RealtimeStatus::StrCat(
        connection_name_, ": The TCP connection was closed"));
  }
}

RealtimeStatus TcpConnection::Receive(absl::Duration timeout,
                                      bool recv_until_buffer_full,
                                      int recv_flags,
                                      absl::Span<uint8_t> buffer,
                                      size_t& bytes_read) {
  bytes_read = 0;
  if (sockfd_ < 0) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        connection_name_,
        ": You need to call Connect() before calling Receive()"));
  }
  absl::Time read_deadline = absl::Now() + timeout;
  (void)memset(buffer.data(), 0, buffer.size());
  while (true) {
    if (bytes_read > buffer.size()) {
      return icon::InternalError(RealtimeStatus::StrCat(
          connection_name_, ": Offset exceeds buffer size. This is a bug."));
    }
    const auto time_remaining = read_deadline - absl::Now();
    // Make sure we don't call setsockopt() with negative timeout.
    if (time_remaining <= absl::ZeroDuration()) {
      return icon::DeadlineExceededError(RealtimeStatus::StrCat(
          connection_name_, ": TCP receive call exceeded timeout: ",
          absl::ToDoubleSeconds(timeout), " seconds"));
    }
    const struct timeval receive_timeout = absl::ToTimeval(time_remaining);
    if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, (char*)&receive_timeout,
                   sizeof(receive_timeout)) < 0) {
      return InternalError(RealtimeStatus::StrCat(
          connection_name_, ": Failed to set socket timeout (",
          absl::ToDoubleSeconds(timeout),
          " seconds): ", icon::StrError(errno)));
    }
    auto done = ReceiveSinglePacket(read_deadline, recv_until_buffer_full,
                                    recv_flags, buffer, bytes_read);
    if (!done.ok()) {
      return done.status();
    }
    if (*done) {
      return OkStatus();
    }
  }
}

bool TcpConnection::IsConnected() {
  if (sockfd_ < 0) return false;
  int error;
  socklen_t error_len = sizeof(error);
  getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &error, &error_len);

  return error != ECONNRESET;
}

absl::StatusOr<std::string> TcpConnection::GetIP() const {
  sockaddr_in name;
  socklen_t len = sizeof(name);
  int res = ::getsockname(GetSocketFileDescriptor(), (sockaddr*)&name, &len);

  if (res < 0) {
    return absl::InternalError(
        absl::StrCat("Could not get local IP: ", strerror(errno)));
  }

  char buf[std::max(INET_ADDRSTRLEN, INET6_ADDRSTRLEN) +
           1];  // +1 for null terminator
  memset(buf, 0, sizeof(buf));
  const char* result_str = inet_ntop(AF_INET, &name.sin_addr, buf, sizeof(buf));
  if (result_str == nullptr) {
    return absl::InternalError(
        absl::StrCat("Could not get local IP: ", strerror(errno)));
  }
  return std::string(result_str);
}

absl::Status TcpClient::ConnectToAddress(int sockfd, const struct addrinfo* p,
                                         absl::Time deadline) {
  int flags = fcntl(sockfd, F_GETFL, 0);
  // Add non-blocking flag so that we can poll for connection and use our
  // timeout.
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
  auto res = connect(sockfd, p->ai_addr, p->ai_addrlen);
  if (res < 0) {
    if (errno == EINPROGRESS) {
      // Connection in progress
      struct pollfd pfd = {sockfd, POLLOUT, 0};
      auto time_remaining = deadline - absl::Now();
      if (time_remaining <= absl::ZeroDuration()) {
        return absl::UnavailableError(
            absl::Substitute("$0: Failed to connect to $1:$2",
                             GetConnectionName(), host_, port_));
      }
      res = poll(&pfd, 1, absl::ToInt64Milliseconds(time_remaining));
      // If >= 0, the socket is ready to be read/written.
      if (res > 0) {
        int so_error;
        socklen_t len = sizeof(so_error);
        // Check if there are any error on the socket.
        getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &so_error, &len);
        if (so_error == 0) {
          // Restore old flags (e.g. remove non-blocking flag).
          fcntl(sockfd, F_SETFL, flags);
          return absl::OkStatus();
        } else {
          return absl::InternalError(absl::Substitute(
              "$0: Failed to connect to $1:$2: $3", GetConnectionName(), host_,
              port_, strerror(so_error)));
        }
      } else if (res == 0) {
        return absl::UnavailableError(absl::Substitute(
            "$0: Failed to connect to address $1:$2 before timeout",
            GetConnectionName(), host_, port_));
      } else {
        return absl::InternalError(
            absl::Substitute("$0: Failed to poll socket: $1",
                             GetConnectionName(), strerror(errno)));
      }
    } else {
      return absl::InternalError(absl::Substitute(
          "$0: Failed to connect: $1", GetConnectionName(), strerror(errno)));
    }
  }
  // Restore old flags (e.g. remove non-blocking flag).
  fcntl(sockfd, F_SETFL, flags);
  return absl::OkStatus();
}

absl::Status TcpClient::Connect(absl::Duration timeout) {
  auto deadline = absl::Now() + timeout;
  const char* host_name = host_.empty() ? nullptr : host_.c_str();
  LOG(INFO) << "Connecting to " << host_name << ":" << port_
            << ", timeout: " << timeout;
  std::string service = std::to_string(port_);
  struct addrinfo hints, *result = nullptr;
  std::memset(&hints, 0, sizeof(hints));
  // Any address family is acceptable.
  hints.ai_family = AF_UNSPEC;
  // Only TCP connections.
  hints.ai_socktype = SOCK_STREAM;
  // The returned addresses should listen for connections and `accept()`
  // messages.
  hints.ai_flags = AI_PASSIVE;
  int sockfd = -1;
  SetSocketFileDescriptor(-1);
  auto start = absl::Now();
  // Loop until we find a valid address that we can connect to or we timeout.
  while ((absl::Now() - start) < timeout) {
    auto cleanup = absl::MakeCleanup([&] { freeaddrinfo(result); });
    if (getaddrinfo(host_name, service.c_str(), &hints, &result) != 0) {
      return absl::UnavailableError(
          absl::Substitute("$0: Failed to get address for $1:$2",
                           GetConnectionName(), host_, port_));
    }

    for (struct addrinfo* p = result; p != nullptr; p = p->ai_next) {
      sockfd = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);

      if (sockfd != -1) {
        auto status = ConnectToAddress(sockfd, p, deadline);
        if (status.ok()) {
          SetSocketFileDescriptor(sockfd);
          LOG(INFO) << GetConnectionName() << ": Connected to " << host_
                    << " on service " << service << ". ";
          return status;
        }
        close(sockfd);
      }
      if ((absl::Now() - start) >= timeout) {
        break;
      }
    }

    absl::SleepFor(absl::Milliseconds(100));
  }
  close(sockfd);
  return absl::UnavailableError(absl::Substitute(
      "$0: Failed to connect to address $1:$2 before timeout of $3",
      GetConnectionName(), host_, port_, timeout));
}

absl::Status TcpServer::OpenPort(size_t connection_queue_size) {
  if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    return UnavailableError(
        RealtimeStatus::StrCat("Failed to create socket: ", strerror(errno)));
  }

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(port_);

  if (bind(sockfd_, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
    return UnavailableError(absl::StrCat(connection_name_,
                                         ": Failed to bind to port ", port_,
                                         ": ", strerror(errno)));
  }

  if (listen(sockfd_, connection_queue_size) < 0) {
    return UnavailableError(absl::StrCat(
        connection_name_, ": Failed to listen: ", strerror(errno)));
  }

  if (int status =
          fcntl(sockfd_, F_SETFL, fcntl(sockfd_, F_GETFL, 0) | O_NONBLOCK);
      status < 0) {
    return absl::InternalError(absl::Substitute(
        "$0: Failed to set socket on port $1 to non-blocking: $2",
        connection_name_, port_, strerror(errno)));
  }

  return absl::OkStatus();
}

absl::StatusOr<TcpConnection> TcpServer::Accept() {
  struct sockaddr_in client_address;

  socklen_t client_address_size = sizeof(client_address);

  int newsockfd =
      accept(sockfd_, (struct sockaddr*)&client_address, &client_address_size);
  if (newsockfd < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return absl::UnavailableError("No new connection pending");
    } else {
      return absl::InternalError(
          absl::StrCat("Failed to accept new connection: ", strerror(errno)));
    }
  }

  LOG(INFO) << "Accepted client on port " << port_ << ": "
            << inet_ntoa(client_address.sin_addr) << ":"
            << ntohs(client_address.sin_port);
  std::string connection_name =
      absl::StrCat(connection_name_, "_", inet_ntoa(client_address.sin_addr),
                   ":", ntohs(client_address.sin_port));

  return absl::StatusOr<TcpConnection>(
      absl::in_place_t(), newsockfd, inet_ntoa(client_address.sin_addr),
      ntohs(client_address.sin_port), connection_name);
}

}  // namespace intrinsic::icon
