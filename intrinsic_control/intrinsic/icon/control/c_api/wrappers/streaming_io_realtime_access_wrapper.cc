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

#include "intrinsic/icon/control/c_api/wrappers/streaming_io_realtime_access_wrapper.h"

#include <cstddef>
#include <cstdint>
#include <tuple>

#include "intrinsic/icon/control/c_api/c_action_factory_context.h"
#include "intrinsic/icon/control/c_api/c_realtime_status.h"
#include "intrinsic/icon/control/c_api/c_streaming_io_realtime_access.h"
#include "intrinsic/icon/control/c_api/convert_c_realtime_status.h"
#include "intrinsic/icon/control/c_api/wrappers/action_factory_context_wrapper.h"
#include "intrinsic/icon/control/streaming_io_realtime.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {
namespace {

StreamingIoRealtimeAccess* Unwrap(
    IntrinsicIconStreamingIoRealtimeAccess* self) {
  return reinterpret_cast<StreamingIoRealtimeAccess*>(self);
}

const IntrinsicIconStreamingInputType* PollInput(
    IntrinsicIconStreamingIoRealtimeAccess* self_wrapped, uint64_t input_id,
    IntrinsicIconRealtimeStatus* status_out) {
  StreamingIoRealtimeAccess* self = Unwrap(self_wrapped);
  // See
  // intrinsic/icon/control/c_api/wrappers/action_factory_context_wrapper.h
  // for how and why this type is used.
  RealtimeStatusOr<const WrappedCApiStreamingInputRealtimeType*> input =
      self->PollInput<WrappedCApiStreamingInputRealtimeType>(
          StreamingInputId(input_id));
  if (!input.ok()) {
    *status_out = FromRealtimeStatus(input.status());
    return nullptr;
  }

  // The streaming input exists, but may not have a value yet. Set `status_out`
  // to Ok, but check for nullptr.
  *status_out = FromRealtimeStatus(icon::OkStatus());
  if (input.value() == nullptr) {
    return nullptr;
  }
  // If there is a value, unpack it from the wrapper type. The pointer is
  // guaranteed to remain valid until the next call to PollInput.
  return input.value()->get();
}

IntrinsicIconRealtimeStatus WriteOutput(
    IntrinsicIconStreamingIoRealtimeAccess* self_wrapped,
    const IntrinsicIconStreamingOutputType* output, size_t size) {
  StreamingIoRealtimeAccess* self = Unwrap(self_wrapped);
  return FromRealtimeStatus(
      self->WriteOutputRawBuffer<
          WrappedCApiStreamingOutputRealtimeType::value_type,
          // tuple_size is different from sizeof(...) if value_type is not a
          // single byte, and the correct value to use here (number of elements
          // vs. number of bytes)!
          std::tuple_size_v<WrappedCApiStreamingOutputRealtimeType>>(
          reinterpret_cast<
              const WrappedCApiStreamingOutputRealtimeType::value_type*>(
              output),
          size));
}

}  // namespace

IntrinsicIconStreamingIoRealtimeAccess* Wrap(StreamingIoRealtimeAccess* self) {
  return reinterpret_cast<IntrinsicIconStreamingIoRealtimeAccess*>(self);
}

IntrinsicIconStreamingIoRealtimeAccessVtable
GetStreamingIoRealtimeAccessVtable() {
  return {
      .poll_input = &PollInput,
      .write_output = &WriteOutput,
  };
}

}  // namespace intrinsic::icon
