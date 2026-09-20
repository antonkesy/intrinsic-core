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

#ifndef INTRINSIC_ICON_CONTROL_STREAMING_IO_REALTIME_H_
#define INTRINSIC_ICON_CONTROL_STREAMING_IO_REALTIME_H_

#include <any>
#include <atomic>
#include <cstring>
#include <type_traits>
#include <typeinfo>

#include "absl/container/fixed_array.h"
#include "absl/time/time.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/initialized_async_buffer.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/current_cycle.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// We pass this to the Sense() method of realtime Actions so they can interact
// with streaming inputs and outputs. All methods are non-blocking and do not
// allocate memory.
// This class is not thread safe, since we assume a single realtime thread.
class StreamingIoRealtimeAccess {
 public:
  // `storage` must outlive the StreamingIoRealtimeAccess object. This shouldn't
  // be a problem, since we expect these objects to be short-lived (roughly for
  // the duration of an Action's Sense() method).
  explicit StreamingIoRealtimeAccess(::intrinsic::Time current_time,
                                     RealtimeStreamingIoStorage& storage)
      : current_time_(current_time), storage_(storage) {}

  // Polls a streaming input.
  // Returns nullptr if nothing has been written to the streaming input for `id`
  // since the last call to PollInput() (or if nothing has been written at all).
  // Returns NotFoundError if there is no streaming input for `id`.
  // Returns InvalidArgumentError if there is a streaming input for `id` with
  // available data, but its type is not `RealtimeT`.
  template <typename RealtimeT>
  RealtimeStatusOr<const RealtimeT*> PollInput(StreamingInputId id);

  // Copies `output` into a buffer that is then made available to the
  // non-realtime thread.
  // NOTE: Because this copies data, be careful about large outputs!
  //
  // Returns NotFound if there is no streaming output in our
  // RealtimeStreamingIoStorage.
  // Returns InvalidArgument if there is a streaming output, but it has a type
  // other than RealtimeT.
  template <typename RealtimeT, typename = std::enable_if_t<
                                    std::is_trivially_copyable_v<RealtimeT>>>
  RealtimeStatus WriteOutput(const RealtimeT& output);

  // Copies `size` bytes starting at `output` into a
  // std::array<BufferT, max_size>, which is then made available to the
  // non-realtime thread.
  //
  // NOTE: Because this copies data, be careful about large outputs!
  //
  // NOTE: Most callers should be using WriteOutput above! This variant makes
  // special affordances for the C API, which can't confer type information
  // (including size) for streaming outputs at compile time.
  //
  // Returns NotFound if there is no streaming output in our
  // RealtimeStreamingIoStorage.
  // Returns InvalidArgument if there is a streaming output, but it has a type
  // other than std::array<BufferT, max_size>.
  // Returns ResourceExhausted if `size` > `max_size`.
  template <typename BufferT, size_t max_size,
            typename = std::enable_if_t<std::is_trivially_copyable_v<BufferT>>>
  RealtimeStatus WriteOutputRawBuffer(const BufferT* output, size_t size);

 private:
  ::intrinsic::Time current_time_;
  RealtimeStreamingIoStorage& storage_;
};

template <typename InputT>
RealtimeStatusOr<const InputT*> StreamingIoRealtimeAccess::PollInput(
    StreamingInputId id) {
  if (id.value() >= storage_.input_channels.size()) {
    return icon::NotFoundError(RealtimeStatus::StrCat(
        "No streaming input channel for StreamingInputId(", id.value(),
        ") of Action ", storage_.id.value()));
  }
  RealtimeStreamingInputChannel& channel = storage_.input_channels[id.value()];
  if (!channel.has_value->load()) {
    return nullptr;
  }

  std::any* any_ptr;
  // If this returns false, we've already read this streaming input value.
  // Return early.
  if (!channel.input_buffer->GetActiveBuffer(&any_ptr)) {
    return nullptr;
  }
  const InputT* value = std::any_cast<InputT>(any_ptr);
  if (value == nullptr) {
    const std::type_info& input_type_info = typeid(InputT);
    return icon::InvalidArgumentError(RealtimeStatus::StrCat(
        "Streaming input channel for StreamingInputId(", id.value(),
        ") of Action ", storage_.id.value(),
        " has a value, but that value has the wrong type. Expected '",
        input_type_info.name(), "', but value is a '", any_ptr->type().name(),
        "'."));
  }
  return value;
}

template <typename OutputT, typename>
RealtimeStatus StreamingIoRealtimeAccess::WriteOutput(const OutputT& output) {
  if (storage_.output_channel_buffer == nullptr) {
    return icon::NotFoundError(RealtimeStatus::StrCat(
        "No streaming output registered for Action ", storage_.id.value(),
        ". Check the factory / signature."));
  }

  // Get the local machine time once.
  uint64_t local_machine_time_ns = absl::ToUnixNanos(absl::Now());

  {
    // Copy to the output buffer.
    StreamingOutputChannel::PayloadAndTimestamp* output_buffer =
        storage_.output_channel_buffer->GetFreeBuffer();
    OutputT* output_t_buf = std::any_cast<OutputT>(&output_buffer->payload);
    if (output_t_buf == nullptr) {
      const std::type_info& output_type_info = typeid(OutputT);
      return icon::InvalidArgumentError(RealtimeStatus::StrCat(
          "Streaming output buffer for Action ", storage_.id.value(),
          " has the wrong type. Output value is a '", output_type_info.name(),
          "', buffer is a '", output_buffer->payload.type().name(), "'."));
    }
    std::memcpy(output_t_buf, &output, sizeof(*output_t_buf));
    output_buffer->timestamp_ns = intrinsic::toNSec<uint64_t>(current_time_);
    output_buffer->wall_clock_timestamp_ns = local_machine_time_ns;
    output_buffer->cycle_index = Cycle::GetCurrentCycle();
    storage_.output_channel_buffer->CommitFreeBuffer();
  }

  // Copy to the output queue.
  {
    StreamingOutputChannel::PayloadAndTimestamp* output_buffer =
        storage_.output_channel_queue->writer()->PrepareInsert();
    // output_buffer could be nullptr here which means the queue is full. If
    // this happens we'll just drop the update.
    if (output_buffer != nullptr) {
      OutputT* output_t_buf = std::any_cast<OutputT>(&output_buffer->payload);
      if (output_t_buf == nullptr) {
        const std::type_info& output_type_info = typeid(OutputT);
        return icon::InvalidArgumentError(RealtimeStatus::StrCat(
            "Streaming output buffer for Action ", storage_.id.value(),
            " has the wrong type. Output value is a '", output_type_info.name(),
            "', buffer is a '", output_buffer->payload.type().name(), "'."));
      }
      std::memcpy(output_t_buf, &output, sizeof(*output_t_buf));
      output_buffer->timestamp_ns = intrinsic::toNSec<uint64_t>(current_time_);
      output_buffer->wall_clock_timestamp_ns = local_machine_time_ns;
      output_buffer->cycle_index = Cycle::GetCurrentCycle();
      storage_.output_channel_queue->writer()->FinishInsert();
    }
  }

  return icon::OkStatus();
}

template <typename BufferT, size_t max_size, typename>
RealtimeStatus StreamingIoRealtimeAccess::WriteOutputRawBuffer(
    const BufferT* output, size_t size) {
  if (storage_.output_channel_buffer == nullptr) {
    return icon::NotFoundError(RealtimeStatus::StrCat(
        "No streaming output registered for Action ", storage_.id.value(),
        ". Check the factory / signature."));
  }
  if (size > max_size) {
    return icon::ResourceExhaustedError(RealtimeStatus::StrCat(
        "Trying to write ", size, " bytes to a buffer that only has ", max_size,
        " bytes."));
  }

  // Get the local machine time once.
  uint64_t local_machine_time_ns = absl::ToUnixNanos(absl::Now());

  // Copy to the output buffer.
  StreamingOutputChannel::PayloadAndTimestamp* output_buffer =
      storage_.output_channel_buffer->GetFreeBuffer();
  std::array<BufferT, max_size>* output_t_buf =
      std::any_cast<std::array<BufferT, max_size>>(&output_buffer->payload);
  if (output_t_buf == nullptr) {
    const std::type_info& output_type_info =
        typeid(std::array<BufferT, max_size>);
    return icon::InvalidArgumentError(RealtimeStatus::StrCat(
        "Streaming output buffer for Action ", storage_.id.value(),
        " has the wrong type. Output value is a '", output_type_info.name(),
        "', buffer is a '", output_buffer->payload.type().name(), "'."));
  }
  std::memcpy(output_t_buf->data(), output, size);
  output_buffer->timestamp_ns = intrinsic::toNSec<uint64_t>(current_time_);
  output_buffer->wall_clock_timestamp_ns = local_machine_time_ns;
  output_buffer->cycle_index = Cycle::GetCurrentCycle();
  storage_.output_channel_buffer->CommitFreeBuffer();

  // Copy to the output queue.
  output_buffer = storage_.output_channel_queue->writer()->PrepareInsert();
  // output_buffer could be nullptr here which means the queue is full. If this
  // happens we'll just drop the update.
  if (output_buffer != nullptr) {
    std::array<BufferT, max_size>* output_t_buf =
        std::any_cast<std::array<BufferT, max_size>>(&output_buffer->payload);
    if (output_t_buf == nullptr) {
      const std::type_info& output_type_info =
          typeid(std::array<BufferT, max_size>);
      return icon::InvalidArgumentError(RealtimeStatus::StrCat(
          "Streaming output buffer for Action ", storage_.id.value(),
          " has the wrong type. Output value is a '", output_type_info.name(),
          "', buffer is a '", output_buffer->payload.type().name(), "'."));
    }
    std::memcpy(output_t_buf->data(), output, size);
    output_buffer->timestamp_ns = intrinsic::toNSec<uint64_t>(current_time_);
    output_buffer->wall_clock_timestamp_ns = local_machine_time_ns;
    output_buffer->cycle_index = Cycle::GetCurrentCycle();
    storage_.output_channel_queue->writer()->FinishInsert();
  }

  return icon::OkStatus();
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_STREAMING_IO_REALTIME_H_
