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

#ifndef INTRINSIC_ICON_UTILS_TCP_SERVER_CLIENT_H_
#define INTRINSIC_ICON_UTILS_TCP_SERVER_CLIENT_H_

#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Represents a single TCP connection regardless if server or client connection.
// This class cannot be copied since the destructor closes the connection. Use
// the move-functions instead.
// While the receive and send functions in this class are written realtime
// compatible, TCP itself is inherently not realtime safe due to retransmissions
// etc.
class TcpConnection {
 protected:
  TcpConnection& operator=(const TcpConnection&) = default;  // For mocking.
  TcpConnection() = default;                                 // For mocking.
  TcpConnection(const TcpConnection&) = default;             // For mocking.
  // Creates an inactive TCP connection. When using this,
  // `SetSocketFileDescriptor` needs to be called to set the file descriptor for
  // this connection.
  explicit TcpConnection(absl::string_view host, uint16_t port,
                         absl::string_view connection_name)
      : host_(host), port_(port), connection_name_(connection_name) {}

 public:
  // Creates a TCP connection. `sockfd` is the file descriptor for the socket
  // that is used to send and receive data.
  // `host`,`port` and `connection_name` are only used for debugging purposes.
  // `connection_name` should be a short (ideally <16 characters) and
  // descriptive name for this connection.
  TcpConnection(int sockfd, absl::string_view host, uint16_t port,
                absl::string_view connection_name)
      : host_(host),
        port_(port),
        sockfd_(sockfd),
        connection_name_(connection_name) {}
  TcpConnection(TcpConnection&& other)
      : host_(std::move(other.host_)),
        port_(std::move(other.port_)),
        connection_name_(std::move(other.connection_name_)) {
    // set it to -1 so that `other` does not close the socket in its
    // destructor.
    sockfd_ = std::exchange(other.sockfd_, -1);
  }
  TcpConnection& operator=(TcpConnection&& other) {
    host_ = std::move(other.host_);
    port_ = std::move(other.port_);
    connection_name_ = std::move(other.connection_name_);
    // set it to -1 so that `other` does not close the socket in its
    // destructor.
    sockfd_ = std::exchange(other.sockfd_, -1);
    return *this;
  }

  // Closes the connection.
  virtual ~TcpConnection() {
    if (sockfd_ != -1) {
      Disconnect();
    }
  }

  // Closes the connection.
  virtual void Disconnect() {
    close(sockfd_);
    sockfd_ = -1;
  }

  // File descriptor that is used for this connection.
  int GetSocketFileDescriptor() const { return sockfd_; }

  // Send the given data via the socket. The user must have called
  // `Connect()` successfully beforehand. `sendto_flags` will be forwarded to
  // the `send()` function. This function is non blocking.
  virtual RealtimeStatus Send(absl::Span<const uint8_t> bytes, int send_flags);

  // Attempts to read data from the TCP socket. Fills `buffer` with the
  // received data up to the size of `buffer`. Additional data will be read in
  // the next call to this function.
  //
  // The `buffer` will be fully filled with zeroes before reading data.
  // `bytes_read` will be updated with the number of bytes read. This will also
  // be filled, if the function failed. This means the number of bytes that
  // have been read already from the socket before the error occurred. Those
  // bytes need to be stored by the user, if they are still needed and combined
  // with the data received with future calls to `Receive()`.
  //
  // The user must have called `Connect()` successfully before.
  //
  // If `recv_until_buffer_full` is true, the function will attempt to read data
  // until the timeout is reached or the buffer is fully filled.
  // If `recv_until_buffer_full` is false, the function will return on reading
  // the next package and return its content (up to size of buffer).
  //
  // `recv_flags` are forwarded to the `recv()` function. Set to 0, if no
  // special flags are needed.
  //
  // Returns true if data has been received and stores the data in `buffer`. If
  // there is no data available until `timeout`, returns false.
  //
  // Returns UnavailableError on socket errors, FailedPreconditionError when the
  // connection was not opened or CancelledError when the connection was closed
  // by the other side.
  virtual RealtimeStatus Receive(absl::Duration timeout,
                                 bool recv_until_buffer_full, int recv_flags,
                                 absl::Span<uint8_t> buffer,
                                 size_t& bytes_read);

  // Convenience overload that attempts to read data from the TCP
  // socket. Fills `container` with the received data up to a size of
  // `container.size()`.
  // Resizes and fills `container` to the number of bytes read, also in
  // error cases where data was already read.
  // `container` must provide `resize()`, `capacity()`, `data()` and `size()`.
  // See `Receive()` for more details about other parameters and
  // functionality.
  template <typename Container>
  RealtimeStatus ReceiveContainer(absl::Duration timeout,
                                  bool recv_until_buffer_full, int recv_flags,
                                  Container& container) {
    container.resize(container.capacity());
    size_t size = 0;
    RealtimeStatus status =
        Receive(timeout, recv_until_buffer_full, recv_flags,
                absl::MakeSpan(container.data(), container.size()), size);
    container.resize(size);
    return status;
  }

  // Checks if the socket was opened and tries to check if the connection is
  // still ok (this part is not reliable). If the function returns false, the
  // connection is closed. If the function returns true, it could already be
  // closed by the other side.
  virtual bool IsConnected();

  // Human-readable connection name used for log messages and errors.
  absl::string_view GetConnectionName() const { return connection_name_; }

  uint16_t GetPort() const { return port_; }
  absl::string_view GetHost() const { return host_; }

  // Returns the IP address of the network interface that is used for this TCP
  // connection.
  absl::StatusOr<std::string> GetIP() const;

 protected:
  void SetSocketFileDescriptor(int sockfd) { sockfd_ = sockfd; }

 private:
  // Receives a single packet from the TCP socket. Returns true, if all data was
  // received. Returns false, if the data is incomplete and this function should
  // be called again.
  RealtimeStatusOr<bool> ReceiveSinglePacket(absl::Time read_deadline,
                                             bool recv_until_buffer_full,
                                             int recv_flags,
                                             absl::Span<uint8_t> buffer,
                                             size_t& bytes_read);

  std::string host_;
  uint16_t port_ = 0;
  int sockfd_ = -1;
  std::string connection_name_;
};

// Simple wrapper class for a TCP client connection.
class TcpClient : public TcpConnection {
 protected:
  TcpClient(const TcpClient&) = default;  // For mocking.
  TcpClient& operator=(const TcpClient&) = delete;
  TcpClient() = default;  // For mocking.

 public:
  // `connection_name` should be a short (ideally <16 characters) and
  // descriptive name for this connection.
  TcpClient(absl::string_view host, uint32_t port,
            absl::string_view connection_name)
      : TcpConnection(host, port, connection_name), host_(host), port_(port) {}

  TcpClient(TcpClient&&) = default;
  TcpClient& operator=(TcpClient&&) = delete;

  virtual absl::Status Connect(absl::Duration timeout);

 private:
  // Connects to the given address. `p` is a pointer to a struct addrinfo that
  // was created by `getaddrinfo()`. `deadline` is the time when the connection
  // attempt should be aborted.
  absl::Status ConnectToAddress(int sockfd, const struct addrinfo* p,
                                absl::Time deadline);
  const std::string host_;
  uint16_t port_ = 0;
};
class TcpServer {
 public:
  // The server will listens on `port` after calling `OpenPort()`.
  // `connection_name` should be a short (ideally <16 characters) and
  // descriptive name for this connection.
  TcpServer(uint32_t port, absl::string_view connection_name)
      : port_(port), connection_name_(connection_name) {}

  // Closes the server connection. Client connections remain open.
  ~TcpServer() {
    close(sockfd_);
    sockfd_ = -1;
  }

  // Open the server port and starts listening for incoming connections. The
  // listening happens in the background. `connection_queue_size` defines how
  // large the queue for pending connections is. This is independent from
  // established connections.
  absl::Status OpenPort(size_t connection_queue_size);

  // Checks if there is a connection pending from a client and accepts this
  // connection. If there is none, returns UnavailableError. If there was an
  // actual error, returns InternalError.
  absl::StatusOr<TcpConnection> Accept();

  uint32_t GetPort() const { return port_; }
  std::string GetConnectionName() const { return connection_name_; }

 private:
  uint16_t port_ = 0;
  int sockfd_ = -1;
  std::string connection_name_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_UTILS_TCP_SERVER_CLIENT_H_
