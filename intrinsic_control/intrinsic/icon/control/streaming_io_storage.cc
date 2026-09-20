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

#include "intrinsic/icon/control/streaming_io_storage.h"

#include <atomic>
#include <cstddef>

#include "absl/container/fixed_array.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic::icon {

StreamingIoStorage::StreamingIoStorage(
    const ActionInstanceId& id,
    const intrinsic_proto::icon::v1::ActionSignature& signature)
    : id_(id),
      signature_(signature),
      input_channels_(signature_.streaming_input_infos().size()) {
  for (size_t i = 0; i < input_channels_.size(); ++i) {
    input_channels_[i].input_name =
        signature_.streaming_input_infos(i).parameter_name();
  }
}

RealtimeStreamingIoStorage FromStreamingIoStorage(StreamingIoStorage& storage) {
  RealtimeStreamingIoStorage rt_storage{
      .id = storage.id_,
      .input_channels = absl::FixedArray<RealtimeStreamingInputChannel>(
          storage.input_channels_.size()),
      .output_channel_buffer = storage.output_channel_ == nullptr
                                   ? nullptr
                                   : &storage.output_channel_->output_buffer_,
      .output_channel_queue = storage.output_channel_ == nullptr
                                  ? nullptr
                                  : &storage.output_channel_->output_queue_};
  for (size_t i = 0; i < storage.input_channels_.size(); ++i) {
    auto& input_channel = storage.input_channels_[i];
    rt_storage.input_channels[i] = RealtimeStreamingInputChannel{
        .has_value = &input_channel.has_value,
        .input_buffer = &input_channel.input_buffer};
  }
  return rt_storage;
}

void ResetRealtimeStreamingIoStorage(RealtimeStreamingIoStorage& storage) {
  for (RealtimeStreamingInputChannel& input_channel : storage.input_channels) {
    input_channel.has_value->store(false, std::memory_order_release);
  }
  if (storage.output_channel_buffer != nullptr) {
    storage.output_channel_buffer->Clear();
  }
}

}  // namespace intrinsic::icon
