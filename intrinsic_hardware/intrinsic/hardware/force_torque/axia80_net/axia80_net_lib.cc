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

#include "intrinsic/hardware/force_torque/axia80_net/axia80_net_lib.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <optional>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/hardware/force_torque/axia80_net/axia80_net.pb.h"
#include "intrinsic/hardware/gripper/wsg32/async_buffer.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_interface_traits.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/hardware_module_registry.h"
#include "intrinsic/icon/hal/interfaces/force_sensor.fbs.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/force_torque_utils.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/malloc_guard.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/stop_token.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"

namespace intrinsic::ati {

using ::intrinsic::ati::internal::Response;
using ::intrinsic::icon::OkStatus;
using ::intrinsic::icon::RealtimeStatus;

namespace {

timespec diff_timespec(const timespec& t1, const timespec& t2) {
  timespec diff;
  if (t2.tv_nsec < t1.tv_nsec) {
    // Borrow from seconds
    diff.tv_sec = t2.tv_sec - t1.tv_sec - 1;
    diff.tv_nsec = (t2.tv_nsec + 1000000000) - t1.tv_nsec;
  } else {
    diff.tv_sec = t2.tv_sec - t1.tv_sec;
    diff.tv_nsec = t2.tv_nsec - t1.tv_nsec;
  }
  return diff;
}

constexpr uint16_t kHeader = 0x1234;
constexpr uint16_t kStartStreaming = 2;
constexpr uint16_t kStreamingPort = 49152;
constexpr int kRequestSizeBytes = 8;
constexpr int kResponseSizeBytes = 36;
// Default multiplier applied to context control period for setting timeouts.
constexpr double kTimeoutControlPeriodMultiplier = 2.0;
// Maximum duration to wait for initial streaming data during Prepare().
constexpr absl::Duration kPrepareDataTimeout = absl::Seconds(30);

struct Connection {
  int socket_handle;
  struct sockaddr_in addr;
  struct hostent* he;
  std::byte request[kRequestSizeBytes];
  Response resp;
  std::byte response[kResponseSizeBytes];
};

int ThreadLoop(intrinsic::StopToken stop_token,
               const ::intrinsic_proto::icon::Axia80NetConfig config,
               wsg32::AsyncBuffer<Response>* status_buffer) {
  // Effectively, disables the malloc guard by using the None reaction.
  // Necessary, since network socket initialization and recv calls may allocate
  // memory.
  SetThreadLocalMallocGuardReaction(icon::MallocGuardReaction::kNone);

  LOG(INFO) << "ThreadLoop starting.";
  Connection connection;

  // Set the ip address. This only needs to be done once, and there is no
  // recourse if we fail to parse.
  if (inet_pton(AF_INET, config.ip().c_str(), &connection.addr.sin_addr) != 1) {
    // Error code must be copied before calling anything else.
    int err_code = errno;
    LOG(ERROR) << "Failed to parse ip address: '" << config.ip()
               << "' Error: " << strerror(err_code);
    return -1;
  }
  connection.addr.sin_family = AF_INET;
  connection.addr.sin_port = htons(kStreamingPort);

  // Create the recv timeout.
  struct timeval tv;
  tv.tv_sec = (time_t)(config.timeout_on_socket_recv_us() / 1000000);
  tv.tv_usec = (suseconds_t)(config.timeout_on_socket_recv_us() % 1000000);

  // Main loop. Start connection and receive. Retry the connection on failure.
  bool socket_open = false;
  while (!stop_token.stop_requested()) {
    if (socket_open) {
      if (close(connection.socket_handle)) {
        // Error code must be copied before calling anything else.
        int err_code = errno;
        LOG(WARNING) << "Failed to close open file descriptor: "
                     << strerror(err_code);
      }
      socket_open = false;
    }

    // Create the socket.
    connection.socket_handle = socket(AF_INET, SOCK_DGRAM, 0);
    if (connection.socket_handle == -1) {
      // Error code must be copied before calling anything else.
      int err_code = errno;
      LOG(ERROR) << "Failed to create socket: " << strerror(err_code);
      return -1;
    }
    socket_open = true;

    // Socket is in standard blocking mode. Setting SO_RCVTIMEO causes recv()
    // to block in the kernel for up to tv duration (yielding CPU) and return
    // EAGAIN/EWOULDBLOCK if no packet arrives before the timeout.
    if (setsockopt(connection.socket_handle, SOL_SOCKET, SO_RCVTIMEO, &tv,
                   sizeof(tv))) {
      int err_code = errno;
      LOG(ERROR) << "Failed to set socket receive timeout: "
                 << strerror(err_code);
      return -1;
    }

    // Create the message to start streaming.
    *(uint16_t*)&connection.request[0] = htons(kHeader);
    *(uint16_t*)&connection.request[2] = htons(kStartStreaming);
    *(uint32_t*)&connection.request[4] = htonl(0);

    LOG(INFO) << "Connecting to device.";
    int err =
        connect(connection.socket_handle, (struct sockaddr*)&connection.addr,
                sizeof(connection.addr));
    if (err) {
      int err_code = errno;
      LOG(ERROR) << "Failed to connect: " << strerror(err_code)
                 << ". Will retry after sleeping.";
      absl::SleepFor(absl::Seconds(1));
      continue;
    }

    LOG(INFO) << "Sending the start streaming request.";
    err = send(connection.socket_handle, connection.request, kRequestSizeBytes,
               0);
    if (err == -1) {
      int err_code = errno;
      LOG(ERROR) << "Failed to send start streaming: " << strerror(err_code)
                 << ". Will retry after sleeping.";
      absl::SleepFor(absl::Seconds(1));
      continue;
    }

    LOG(INFO) << "Waiting for first response.";
    bool got_first_response = false;
    for (int i = 0; i < 500; i++) {
      // Receive.
      int b = recv(connection.socket_handle, connection.response,
                   kResponseSizeBytes, 0);
      if (b == -1) {
        int err_code = errno;
        if (err_code == EAGAIN || err_code == EWOULDBLOCK) {
          // Sleep briefly to yield CPU to other realtime processes while
          // waiting for device response. Otherwise this thread can block ICON /
          // other hardware modules (see b/527072101).
          absl::SleepFor(absl::Milliseconds(2));
          continue;
        }
        LOG(ERROR) << "Failed to recv: " << strerror(err_code);
        break;
      }
      // Wait till we get a full response.
      if (b != kResponseSizeBytes) {
        // Sleep briefly to yield CPU to other realtime processes while waiting
        // for device response.
        // Otherwise this thread can block ICON / other hardware modules (see
        // b/527072101).
        absl::SleepFor(absl::Milliseconds(2));
        continue;
      }
      got_first_response = true;
      break;
    }
    if (!got_first_response) {
      LOG(ERROR) << "Did not get an first response. Retrying connection.";
      continue;
    }

    // Inner loop. Receive, and break out on failure.
    // Note: Stale or missing data stream detection is handled by ReadStatus()
    // checking last_receive_time_monotonic against
    // timeout_on_stale_read_seconds. Blocking recv() on a UDP socket with
    // SO_RCVTIMEO yields CPU while waiting.
    LOG(INFO) << "Starting receiving loop.";
    while (!stop_token.stop_requested()) {
      // Receive.
      int b = recv(connection.socket_handle, connection.response,
                   kResponseSizeBytes, 0);
      if (b == -1) {
        int err_code = errno;
        // On a blocking socket with SO_RCVTIMEO set, EAGAIN / EWOULDBLOCK
        // indicates a receive timeout (no packet arrived within
        // timeout_on_socket_recv_us).
        if (err_code == EAGAIN || err_code == EWOULDBLOCK) {
          INTRINSIC_RT_LOG_THROTTLED(ERROR)
              << "Timeout on recv, will retry connection.";
          break;
        }
        INTRINSIC_RT_LOG_THROTTLED(ERROR)
            << "Failed to recv: " << strerror(err_code);
        return -1;
      }
      if (b != kResponseSizeBytes) {
        INTRINSIC_RT_LOG_THROTTLED(ERROR)
            << "Size mismatch of received message, expected "
            << kResponseSizeBytes << " bytes, got " << b
            << " bytes. Discarding.";
        continue;
      }

      // Copy data from the raw buffer to the output response buffer.
      auto* response = status_buffer->GetFreeBuffer();
      clock_gettime(CLOCK_MONOTONIC, &response->last_receive_time_monotonic);
      response->rdt_sequence = ntohl(*(uint32_t*)&connection.response[0]);
      response->ft_sequence = ntohl(*(uint32_t*)&connection.response[4]);
      response->status = ntohl(*(uint32_t*)&connection.response[8]);
      for (int i = 0; i < 6; i++) {
        response->ft_data[i] =
            ntohl(*(int32_t*)&connection.response[12 + i * 4]);
      }
      if (!status_buffer->CommitFreeBuffer()) {
        INTRINSIC_RT_LOG_THROTTLED(ERROR)
            << "Could not commit status buffer. This is unexpected.";
      }
    }
  }

  if (socket_open) {
    close(connection.socket_handle);
  }

  LOG(INFO) << "Exiting ThreadLoop.";
  return 0;
}

}  // namespace

absl::Status Axia80NetHwm::Init(
    intrinsic::icon::HardwareModuleInitContext& init_context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  intrinsic::icon::HardwareInterfaceRegistry& interface_registry =
      init_context.GetInterfaceRegistry();
  const intrinsic::icon::ModuleConfig& module_config =
      init_context.GetModuleConfig();

  // Check whether the module_config is asking this hwm to drive.
  // This hwm doesn't support this!
  if (module_config.GetRealtimeClock() != nullptr) {
    return absl::InvalidArgumentError(
        "Axia80NetHwm cannot drive clock, but was requested to.");
  }

  // Unpack any module specific configuration.
  INTR_ASSIGN_OR_RETURN(
      auto config,
      module_config.GetConfig<::intrinsic_proto::icon::Axia80NetConfig>());

  // Auto-configure timeouts based on the context's control period if left
  // unset.
  if (!config.has_timeout_on_stale_read_seconds() ||
      !config.has_timeout_on_socket_recv_us()) {
    INTR_ASSIGN_OR_RETURN(absl::Duration control_period,
                          module_config.GetControlPeriod());

    if (!config.has_timeout_on_stale_read_seconds()) {
      config.set_timeout_on_stale_read_seconds(
          kTimeoutControlPeriodMultiplier *
          absl::ToDoubleSeconds(control_period));
      LOG(INFO) << "Auto-configured timeout_on_stale_read_seconds to "
                << config.timeout_on_stale_read_seconds() << "s ("
                << kTimeoutControlPeriodMultiplier << "x control period).";
    }
    if (!config.has_timeout_on_socket_recv_us()) {
      config.set_timeout_on_socket_recv_us(
          static_cast<uint64_t>(kTimeoutControlPeriodMultiplier *
                                absl::ToInt64Microseconds(control_period)));
      LOG(INFO) << "Auto-configured timeout_on_socket_recv_us to "
                << config.timeout_on_socket_recv_us() << "us ("
                << kTimeoutControlPeriodMultiplier << "x control period).";
    }
  }

  if (config.has_timeout_on_stale_read_seconds()) {
    LOG(INFO) << "Using timeout_on_stale_read_seconds: "
              << config.timeout_on_stale_read_seconds() << "s.";
  }
  if (config.has_timeout_on_socket_recv_us()) {
    LOG(INFO) << "Using timeout_on_socket_recv_us: "
              << config.timeout_on_socket_recv_us() << "us.";
  }

  LOG(INFO) << "Hardware module '" << module_config.GetName() << "' with type '"
            << kHwModuleTypeName << "' initializing with config:\n"
            << config;

  // Initialized the shared memory that will be used to send statuses and
  // receive commands.
  INTR_ASSIGN_OR_RETURN(
      status_handle_,
      interface_registry
          .AdvertiseMutableInterface<intrinsic_fbs::ForceTorqueStatus>(
              "status"));
  INTR_ASSIGN_OR_RETURN(
      command_handle_,
      interface_registry.AdvertiseInterface<intrinsic_fbs::ForceTorqueCommand>(
          "command"));

  INTR_RETURN_IF_ERROR(InitInternal(
      config,
      module_config.GetIconThreadOptions()
          .SetRealtimeLowPriorityAndScheduler()
          .SetName(absl::StrCat("Axia80NetHwm-", module_config.GetName()))));

  return absl::OkStatus();
}

absl::Status Axia80NetHwm::InitInternal(
    const ::intrinsic_proto::icon::Axia80NetConfig& config,
    const ThreadOptions& thread_options) {
  config_ = config;
  thread_options_ = thread_options;

  if (config_.counts_per_force() == 0) {
    return absl::InvalidArgumentError(
        "Must specify counts_per_force as it is configured on your device.");
  }
  if (config_.counts_per_torque() == 0) {
    return absl::InvalidArgumentError(
        "Must specify counts_per_torque as it is configured on your device.");
  }

  if (config_.timeout_on_stale_read_seconds() <= 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("timeout_on_stale_read_seconds must be > 0, got: ",
                     config_.timeout_on_stale_read_seconds(), "s."));
  }

  if (config_.timeout_on_socket_recv_us() <= 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("timeout_on_socket_recv_us must be > 0, got: ",
                     config_.timeout_on_socket_recv_us(), "us."));
  }

  // Start thread (on realtime core) if not started already.
  if (!thread_.joinable()) {
    INTR_ASSIGN_OR_RETURN(
        thread_, CreateRealtimeCapableThread(thread_options_, ThreadLoop,
                                             config_, &status_buffer_));
  }

  LOG(INFO) << "Axia80NetHwm done init.";

  return absl::OkStatus();
}

absl::Status Axia80NetHwm::Prepare() {
  if (!thread_.joinable()) {
    INTR_ASSIGN_OR_RETURN(
        thread_, CreateRealtimeCapableThread(thread_options_, ThreadLoop,
                                             config_, &status_buffer_));
  }

  // Wait for the first data to come back from the hardware.
  const absl::Time deadline = absl::Now() + kPrepareDataTimeout;
  Response* buffered_status = nullptr;
  while (!status_buffer_.GetActiveBuffer(&buffered_status)) {
    if (absl::Now() > deadline) {
      return absl::DeadlineExceededError(absl::StrCat(
          "Axia80NetHwm: Timed out waiting for streaming data from device at ",
          config_.ip(), " after ", absl::FormatDuration(kPrepareDataTimeout)));
    }
    LOG(INFO) << "Waiting for device to be ready...";
    absl::SleepFor(absl::Seconds(0.5));
  }

  return absl::OkStatus();
}

RealtimeStatus Axia80NetHwm::Activate() { return OkStatus(); }

RealtimeStatus Axia80NetHwm::Deactivate() {
  enabled_ = false;
  return OkStatus();
}

absl::Status Axia80NetHwm::EnableMotion() {
  enabled_ = true;
  return absl::OkStatus();
}

absl::Status Axia80NetHwm::DisableMotion() {
  enabled_ = false;
  return absl::OkStatus();
}

absl::Status Axia80NetHwm::ClearFaults() { return absl::OkStatus(); }

absl::Status Axia80NetHwm::Shutdown() {
  if (thread_.joinable()) {
    thread_.request_stop();
    thread_.join();
  }
  return absl::OkStatus();
}

RealtimeStatus Axia80NetHwm::ReadStatus() {
  Response* buffered_status = nullptr;
  // Always process the buffer, even if it isn't updated.
  status_buffer_.GetActiveBuffer(&buffered_status);

  if (buffered_status == nullptr) {
    return ::intrinsic::icon::InternalError("Got a nullptr for status!");
  }

  // Check for stale data.
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  timespec diff =
      diff_timespec(buffered_status->last_receive_time_monotonic, now);
  double time_delta_sec = diff.tv_sec + 1e-9 * diff.tv_nsec;
  if (time_delta_sec > config_.timeout_on_stale_read_seconds()) {
    // Explicitly mark sensor as disabled since stale data is no longer valid
    // live measurement data.
    status_handle_->mutate_enabled(false);
    INTRINSIC_RT_LOG(ERROR)
        << "ReadStatus timeout! Last update time: " << time_delta_sec
        << " timeout: " << config_.timeout_on_stale_read_seconds();
    return ::intrinsic::icon::InternalError(
        ::intrinsic::icon::RealtimeStatus::StrCat(
            "Did not receive data within timeout of ",
            config_.timeout_on_stale_read_seconds(), "s."));
  }

  eigenmath::Vector6d wrench_raw;
  wrench_raw << buffered_status->ft_data[0] /
                    (double)config_.counts_per_force(),
      buffered_status->ft_data[1] / (double)config_.counts_per_force(),
      buffered_status->ft_data[2] / (double)config_.counts_per_force(),
      buffered_status->ft_data[3] / (double)config_.counts_per_torque(),
      buffered_status->ft_data[4] / (double)config_.counts_per_torque(),
      buffered_status->ft_data[5] / (double)config_.counts_per_torque();

  taring_data_.Update(wrench_raw);

  auto wrench_tared_or = taring_data_.GetTaredValue();
  if (wrench_tared_or) {
    status_handle_->mutable_wrench()->mutate_x((*wrench_tared_or)(0));
    status_handle_->mutable_wrench()->mutate_y((*wrench_tared_or)(1));
    status_handle_->mutable_wrench()->mutate_z((*wrench_tared_or)(2));
    status_handle_->mutable_wrench()->mutate_rx((*wrench_tared_or)(3));
    status_handle_->mutable_wrench()->mutate_ry((*wrench_tared_or)(4));
    status_handle_->mutable_wrench()->mutate_rz((*wrench_tared_or)(5));
  } else {
    // Send zeros while taring.
    status_handle_->mutable_wrench()->mutate_x(0);
    status_handle_->mutable_wrench()->mutate_y(0);
    status_handle_->mutable_wrench()->mutate_z(0);
    status_handle_->mutable_wrench()->mutate_rx(0);
    status_handle_->mutable_wrench()->mutate_ry(0);
    status_handle_->mutable_wrench()->mutate_rz(0);
  }

  status_handle_->mutate_enabled(enabled_);
  // TODO(keegang): watch the unsigned to signed conversion here.
  status_handle_->mutate_raw_status_code(buffered_status->status);
  // TODO(keegang): derive this from the raw_status_code.
  status_handle_->mutate_status_code(intrinsic_fbs::ForceSensorStatusCode::Ok);
  status_handle_->mutate_retare_completed(!taring_data_.TaringInProgress());

  return OkStatus();
}

RealtimeStatus Axia80NetHwm::ApplyCommand() {
  if (command_handle_->retare()) {
    INTRINSIC_RT_LOG(INFO) << "Got taring command for "
                           << command_handle_->num_taring_cycles()
                           << " cycles.";
    taring_data_.StartTare(command_handle_->num_taring_cycles());
  }
  return OkStatus();
}

}  // namespace intrinsic::ati

// Register the interfaces we use.
namespace intrinsic::icon {
namespace hardware_interface_traits {

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::ForceTorqueStatus,
                                 intrinsic_fbs::CreateFbsForceTorqueStatus,
                                 "intrinsic_fbs.ForceTorqueStatus")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::ForceTorqueCommand,
                                 intrinsic_fbs::CreateFbsForceTorqueCommand,
                                 "intrinsic_fbs.ForceTorqueCommand")

}  // namespace hardware_interface_traits
}  // namespace intrinsic::icon
REGISTER_HARDWARE_MODULE(::intrinsic::ati::Axia80NetHwm);
