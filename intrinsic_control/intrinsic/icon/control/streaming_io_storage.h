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

#ifndef INTRINSIC_ICON_CONTROL_STREAMING_IO_STORAGE_H_
#define INTRINSIC_ICON_CONTROL_STREAMING_IO_STORAGE_H_

#include <any>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "absl/container/fixed_array.h"
#include "absl/memory/memory.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/initialized_async_buffer.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/proto/streaming_output.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/platform/common/buffers/rt_queue.h"

namespace intrinsic::icon {

// Uses std::any for type erasure. The receiving (realtime) side only reads
// from the AsyncBuffer, so we don't need to worry about realtime safety. The
// sender can take as much time and memory as it needs to copy data into the
// AsyncBuffer.
struct StreamingInputChannel {
  std::string input_name = "";
  GenericStreamingInputParser input_parser = nullptr;
  // Provides some small measure of protection against using stale input values.
  // The realtime thread can set this to false when the corresponding Action is
  // started, and the non-realtime thread can set it to true when it sends a
  // streaming input.
  //
  // Of course, a sufficiently oblivious client can keep *sending* stale
  // streaming inputs, but there's no way to prevent that.
  //
  // This is only relevant for Action/Reaction graphs that have loops, i.e.
  // those that can run the same Action instance more than once. Different
  // Action instances each have their own StreamingInputChannel.
  std::atomic<bool> has_value = false;
  static_assert(std::atomic<bool>::is_always_lock_free,
                "StreamingInputChannel is not RT safe because "
                "std::atomic<bool> is not lock free!");
  AsyncBuffer<std::any> input_buffer;
};

// The realtime side *does* write to this one's AsyncBuffer, which means we
// must make sure that the std::any within already has the correct size, and
// that the output type is trivial (trivially copyable, plus has a trivial
// default ctor).
//
// This allows the realtime side to treat the std::any like a fixed-size
// buffer that it can just memcpy values into.
//
// NB: If the realtime side were to write a new value into the std::any,
// rather than memcpy into the existing one, it could cause spurious
// allocations. On the other hand, std::any provides some type safety in that
// any_cast returns a nullptr if the object within does not have the expected
// type.
//
// Since the only code interacting with the std::any directly is framework
// code that can be audited for inadvertent allocations, we err on the side of
// type safety (i.e. we use std::any). This lets us report more detailed
// error messages to the user, rather than accepting and trying to convert
// garbage data back into a proto.
class StreamingOutputChannel {
 public:
  template <typename T,
            typename = std::enable_if_t<std::is_trivially_copyable_v<T>>>
  static std::unique_ptr<StreamingOutputChannel> Create(
      GenericStreamingOutputConverter output_converter) {
    return absl::WrapUnique(
        new StreamingOutputChannel(std::move(output_converter), std::any(T())));
  }

  struct PayloadAndTimestamp {
    std::any payload;
    uint64_t timestamp_ns;
    uint64_t wall_clock_timestamp_ns;
    uint64_t cycle_index;
  };

  GenericStreamingOutputConverter output_converter_ = nullptr;
  // We use two separate data structures for output:
  //   * output_queue_ stores the oldest N unprocessed outputs. The queue is
  //     read by a non-realtime publishing thread that is responsible for
  //     publishing the outputs via dds.
  //   * output_buffer_ always holds the latest output value. It is used to
  //     respond to GetLatestStreamingOutput queries via an icon client.
  RealtimeQueue<PayloadAndTimestamp> output_queue_;
  InitializedAsyncBuffer<PayloadAndTimestamp> output_buffer_;

 private:
  explicit StreamingOutputChannel(
      GenericStreamingOutputConverter output_converter,
      std::any output_buffer_value)
      : output_converter_(std::move(output_converter)),
        output_buffer_(PayloadAndTimestamp{.payload = output_buffer_value,
                                           .timestamp_ns = 0}) {
    output_queue_.InitElements(
        [&output_buffer_value](PayloadAndTimestamp* data) {
          data->payload = output_buffer_value;
          data->timestamp_ns = 0;
        });
  }
};

class StreamingIoStorage {
 public:
  explicit StreamingIoStorage(
      const ActionInstanceId& id,
      const intrinsic_proto::icon::v1::ActionSignature& signature);

  const ActionInstanceId id_;
  const intrinsic_proto::icon::v1::ActionSignature signature_;
  // Initialized in the constructor to hold input channels (without parsers) for
  // each of the streaming inputs in `signature_`.
  absl::FixedArray<StreamingInputChannel> input_channels_;
  // unique_ptr rather than absl::optional, because StreamingOutputChanel is not
  // movable or copyable (because its AsyncBuffer is not).
  std::unique_ptr<StreamingOutputChannel> output_channel_;
};

// Holds pointers to the data for `StreamingInputChannel`. Must not outlive the
// corresponding `StreamingInputChannel`.
struct RealtimeStreamingInputChannel {
  std::atomic<bool>* has_value = nullptr;
  AsyncBuffer<std::any>* input_buffer = nullptr;
};

// Maybe not strictly necessary, but leaves us some room to tighten the
// interfaces later.
// For example, we could split the API of AsyncBuffer into AsyncBufferConsumer
// and AsyncBufferProducer, and only pass the corresponding side to the realtime
// thread.
struct RealtimeStreamingIoStorage {
  ActionInstanceId id;
  absl::FixedArray<RealtimeStreamingInputChannel> input_channels;
  // The output_channel_buffer will hold the latest update, and the
  // output_channel_queue will hold N oldest outputs. See the docs for
  // StreamingOutputChannel for more details.
  InitializedAsyncBuffer<StreamingOutputChannel::PayloadAndTimestamp>* const
      output_channel_buffer = nullptr;
  // We pass the whole queue so that the real time code has the ability to clear
  // the queue on reset. Other than that, the channel should only be used for
  // writing.
  RealtimeQueue<StreamingOutputChannel::PayloadAndTimestamp>*
      output_channel_queue = nullptr;
};

// Populates a RealtimeStreamingIoStorage struct with pointers to the values
// held by `storage`.
//
// `storage` must outlive the generated RealtimeStreamingIoStorage!
RealtimeStreamingIoStorage FromStreamingIoStorage(StreamingIoStorage& storage)
    INTRINSIC_NON_REALTIME_ONLY;

void ResetRealtimeStreamingIoStorage(RealtimeStreamingIoStorage& storage)
    INTRINSIC_CHECK_REALTIME_SAFE;

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_STREAMING_IO_STORAGE_H_
