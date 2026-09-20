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

#include "intrinsic/icon/control/streaming_io_helpers.h"

#include <any>
#include <cstddef>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/proto/streaming_output.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::StatusOr<size_t> GetStreamingInputIndexAny(
    absl::string_view input_name, absl::string_view input_proto_type_name,
    StreamingIoStorage& io_storage) {
  const auto streaming_input_it = absl::c_find_if(
      io_storage.signature_.streaming_input_infos(),
      [&input_name](
          const intrinsic_proto::icon::v1::ActionSignature::ParameterInfo&
              streaming_input_info) {
        return streaming_input_info.parameter_name() == input_name;
      });
  if (streaming_input_it ==
      io_storage.signature_.streaming_input_infos().end()) {
    return absl::NotFoundError(absl::StrCat(
        "Action ", io_storage.id_.value(), " of type '",
        io_storage.signature_.action_type_name(),
        "' does not have a streaming input named '", input_name, "'"));
  }
  if (input_proto_type_name != streaming_input_it->value_message_type()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Streaming input '", input_name, "' of Action ", io_storage.id_.value(),
        " of type '", io_storage.signature_.action_type_name(),
        "' has the proto message type '",
        streaming_input_it->value_message_type(), "', but we expected '",
        input_proto_type_name, "'."));
  }
  auto input_channel_it = absl::c_find_if(
      io_storage.input_channels_,
      [&input_name](const StreamingInputChannel& input_channel) {
        return input_channel.input_name == input_name;
      });
  if (input_channel_it == io_storage.input_channels_.end()) {
    return absl::InternalError(
        absl::StrCat("No InputChannel object for streaming input named '",
                     input_name, "' of Action ", io_storage.id_.value(),
                     " of type '", io_storage.signature_.action_type_name(),
                     "'. This is a programming error in ICON!"));
  }
  return std::distance(io_storage.input_channels_.begin(), input_channel_it);
}

absl::Status WriteStreamingInputAny(absl::string_view input_name,
                                    const google::protobuf::Any& input_proto,
                                    StreamingIoStorage& io_storage) {
  // Work around a quirk of StrSplit, where it sometimes returns an empty list
  // instead of a list containing the empty string.
  std::string type_name = "";
  if (!input_proto.type_url().empty()) {
    std::vector<std::string> url_components =
        absl::StrSplit(input_proto.type_url(), '/');
    type_name = url_components.back();
  }
  INTR_ASSIGN_OR_RETURN(
      size_t input_channel_index,
      GetStreamingInputIndexAny(input_name, type_name, io_storage));
  StreamingInputChannel& input_channel =
      io_storage.input_channels_[input_channel_index];
  // Make sure that we have a parser for this input.
  if (input_channel.input_parser == nullptr) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Cannot write to streaming input '", input_name, "' of Action ",
        io_storage.id_.value(), " of type '",
        io_storage.signature_.action_type_name(), "': No parser registered."));
  }
  std::any* input_buffer = input_channel.input_buffer.GetFreeBuffer();
  INTR_ASSIGN_OR_RETURN(*input_buffer, input_channel.input_parser(input_proto));
  input_channel.input_buffer.CommitFreeBuffer();
  input_channel.has_value = true;
  return absl::OkStatus();
}

absl::StatusOr<StreamingOutputWithCycle> PollStreamingOutputAny(
    absl::Time deadline, StreamingIoStorage& io_storage) {
  if (!io_storage.signature_.has_streaming_output_info()) {
    return absl::NotFoundError(
        absl::StrCat("Action ", io_storage.id_.value(), " of type '",
                     io_storage.signature_.action_type_name(),
                     "' does not have a streaming output."));
  }
  // Make sure that we have an output converter.
  if (io_storage.output_channel_ == nullptr) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Cannot poll streaming output of Action ", io_storage.id_.value(),
        " of type '", io_storage.signature_.action_type_name(),
        "': No converter registered."));
  }
  std::optional<StreamingOutputChannel::PayloadAndTimestamp>
      realtime_streaming_output =
          io_storage.output_channel_->output_buffer_.GetCurrentValue(deadline);
  if (!realtime_streaming_output.has_value()) {
    return absl::DeadlineExceededError(
        absl::StrCat("Timed out waiting for streaming output of Action ",
                     io_storage.id_.value(), " of type ",
                     io_storage.signature_.action_type_name()));
  }
  StreamingOutputWithCycle output_with_cycle;
  output_with_cycle.output.set_timestamp_ns(
      realtime_streaming_output.value().timestamp_ns);
  output_with_cycle.output.set_wall_clock_timestamp_ns(
      realtime_streaming_output.value().wall_clock_timestamp_ns);
  output_with_cycle.cycle_index = realtime_streaming_output.value().cycle_index;
  INTR_ASSIGN_OR_RETURN(*output_with_cycle.output.mutable_payload(),
                        io_storage.output_channel_->output_converter_(
                            realtime_streaming_output.value().payload));
  return output_with_cycle;
}

absl::StatusOr<std::optional<StreamingOutputWithCycle>> GetNextStreamingOutput(
    StreamingIoStorage& io_storage) {
  if (!io_storage.signature_.has_streaming_output_info()) {
    return absl::NotFoundError(
        absl::StrCat("Action ", io_storage.id_.value(), " of type '",
                     io_storage.signature_.action_type_name(),
                     "' does not have a streaming output."));
  }
  // Make sure that we have an output converter.
  if (io_storage.output_channel_ == nullptr) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Cannot poll streaming output of Action ", io_storage.id_.value(),
        " of type '", io_storage.signature_.action_type_name(),
        "': No converter registered."));
  }
  std::optional<StreamingOutputChannel::PayloadAndTimestamp>
      realtime_streaming_output =
          io_storage.output_channel_->output_queue_.reader()->Pop();
  if (!realtime_streaming_output.has_value()) {
    return std::nullopt;
  }
  StreamingOutputWithCycle output_with_cycle;
  output_with_cycle.output.set_timestamp_ns(
      realtime_streaming_output.value().timestamp_ns);
  output_with_cycle.output.set_wall_clock_timestamp_ns(
      realtime_streaming_output.value().wall_clock_timestamp_ns);
  output_with_cycle.cycle_index = realtime_streaming_output.value().cycle_index;
  INTR_ASSIGN_OR_RETURN(*output_with_cycle.output.mutable_payload(),
                        io_storage.output_channel_->output_converter_(
                            realtime_streaming_output.value().payload));
  return output_with_cycle;
}

}  // namespace intrinsic::icon
