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

#include "intrinsic/icon/control/streaming_io_registry.h"

#include <stddef.h>

#include <any>
#include <array>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/streaming_io_helpers.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/util/status/annotate.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

StreamingIoRegistry::StreamingIoRegistry(StreamingIoStorage& storage)
    : storage_(storage) {}

absl::Status StreamingIoRegistry::AddStreamingOutputConverterAny(
    absl::string_view proto_type_name, size_t realtime_type_size,
    std::function<absl::StatusOr<::google::protobuf::Any>(
        const std::array<char, kMaxStreamingOutputSizeBytes>& streaming_output)>
        converter) {
  if (realtime_type_size > kMaxStreamingOutputSizeBytes) {
    return absl::InvalidArgumentError(
        absl::StrCat("realtime_type_size must not be larger than "
                     "kMaxStreamingOutputSizeBytes(",
                     kMaxStreamingOutputSizeBytes, ")"));
  }

  if (!storage_.signature_.has_streaming_output_info()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Cannot add streaming output converter for Action ",
        storage_.id_.value(), " of type '",
        storage_.signature_.action_type_name(),
        "', since this Action type does not export a streaming output."));
  }
  if (proto_type_name !=
      storage_.signature_.streaming_output_info().value_message_type()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Cannot register streaming output converter for Action ",
        storage_.id_.value(), " of type '",
        storage_.signature_.action_type_name(),
        "'. This Action type has a streaming output of type '",
        storage_.signature_.streaming_output_info().value_message_type(),
        "', but the converter function takes '", proto_type_name, "'."));
  }

  if (storage_.output_channel_ != nullptr) {
    return absl::AlreadyExistsError(
        absl::StrCat("Cannot register more than one streaming output converter "
                     "for Action ",
                     storage_.id_.value(), " of type '",
                     storage_.signature_.action_type_name(), "'."));
  }

  // Save Converter, and wrap to conform to generic interface.
  storage_.output_channel_ = StreamingOutputChannel::Create<
      std::array<char, kMaxStreamingOutputSizeBytes>>(
      [converter, action_id = storage_.id_,
       action_type_name = storage_.signature_.action_type_name(),
       streaming_output_type_name =
           storage_.signature_.streaming_output_info().value_message_type()](
          const std::any& streaming_output_any)
          -> absl::StatusOr<google::protobuf::Any> {
        const auto* streaming_output =
            std::any_cast<std::array<char, kMaxStreamingOutputSizeBytes>>(
                &streaming_output_any);
        if (streaming_output == nullptr) {
          return absl::InternalError(absl::StrCat(
              "Failed to cast streaming output for Action ", action_id.value(),
              " of type '", action_type_name,
              "' from std::any to converter type. This is a bug in ICON!"));
        }

        absl::StatusOr<::google::protobuf::Any> streaming_output_proto_or =
            converter(*streaming_output);
        if (!streaming_output_proto_or.ok()) {
          return AnnotateError(
              streaming_output_proto_or.status(),
              absl::StrCat("Converter for streaming output on Action ",
                           action_id.value(), " of type '", action_type_name,
                           "' failed."));
        }
        // Work around a quirk of StrSplit, where it sometimes returns an empty
        // list instead of a list containing the empty string.
        std::string type_name = "";
        if (!streaming_output_proto_or->type_url().empty()) {
          std::vector<std::string> url_components =
              absl::StrSplit(streaming_output_proto_or->type_url(), '/');
          type_name = url_components.back();
        }
        if (type_name != streaming_output_type_name) {
          return absl::InternalError(absl::StrCat(
              "Converter for streaming output on Action ", action_id.value(),
              " of type '", action_type_name, "' returned a proto of type '",
              type_name, "' instead of the expected '",
              streaming_output_type_name, "'"));
        }
        return *streaming_output_proto_or;
      });
  return absl::OkStatus();
}

absl::StatusOr<StreamingInputId>
StreamingIoRegistry::AddStreamingInputParserAny(
    absl::string_view input_name, absl::string_view proto_type_name,
    std::function<absl::StatusOr<std::any>(
        const ::google::protobuf::Any& streaming_input)>
        parser) {
  INTR_ASSIGN_OR_RETURN(
      size_t input_channel_index,
      GetStreamingInputIndexAny(input_name, proto_type_name, storage_));
  StreamingInputChannel& input_channel =
      storage_.input_channels_[input_channel_index];
  // Check that we don't already have a parser for this input.
  if (input_channel.input_parser != nullptr) {
    return absl::AlreadyExistsError(absl::StrCat(
        "Cannot register multiple input parsers for streaming input '",
        input_name, "' of Action ", storage_.id_.value(), " of type '",
        storage_.signature_.action_type_name(), "'."));
  }

  input_channel.input_parser = std::move(parser);

  // We've successfully saved the parser, so return the index of the
  // InputChannel as a StreamingInputId.
  return StreamingInputId(input_channel_index);
}

}  // namespace intrinsic::icon
