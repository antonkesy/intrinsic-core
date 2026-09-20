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

#ifndef INTRINSIC_ICON_CONTROL_STREAMING_IO_REGISTRY_H_
#define INTRINSIC_ICON_CONTROL_STREAMING_IO_REGISTRY_H_

#include <stddef.h>

#include <any>
#include <array>
#include <cstring>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/any.pb.h"
#include "google/protobuf/message.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// This is passed to an Action factory and provides a write-only interface to a
// `StreamingIoStorage` object. *Does not* take ownership of `storage`, and
// *must not* outlive `storage`.
class StreamingIoRegistry {
 public:
  // Streaming output values may not be larger than this. This limit reduces the
  // risk of an Action taking too much time to memcpy its output.
  static constexpr size_t kMaxStreamingOutputSizeBytes = 102400;

  explicit StreamingIoRegistry(StreamingIoStorage& storage);

  // RtclActions can use the StreamingInputId that this returns to access
  // streaming input values in realtime functions.
  //
  // Returns an error if there's already a streaming input parser registered for
  // `input_name`.
  template <typename ProtoT, typename RealtimeT,
            typename = std::enable_if_t<
                std::is_base_of_v<google::protobuf::Message, ProtoT>>>
  absl::StatusOr<StreamingInputId> AddStreamingInputParser(
      absl::string_view input_name,
      std::function<absl::StatusOr<RealtimeT>(const ProtoT& streaming_input)>
          parser);

  // A variant of AddStreamingInputParser for parsers that take
  // ::google::protobuf::Any and produce std::any. Most callers should be using
  // AddStreamingInputParser, which takes care of packing / unpacking the
  // output / input automatically.
  //
  // Returns an error if `proto_type_name` does not match the input message
  // type for `input_name` in our `signature_`.
  //
  // RtclActions can use the StreamingInputId that this returns to access
  // streaming input values in realtime functions.
  //
  // Returns an error if there's already a streaming input parser registered for
  // `input_name`.
  absl::StatusOr<StreamingInputId> AddStreamingInputParserAny(
      absl::string_view input_name, absl::string_view proto_type_name,
      std::function<absl::StatusOr<std::any>(
          const ::google::protobuf::Any& streaming_input)>
          parser);

  // Returns an error if ProtoT does not match the output message type in our
  // `signature_`.
  //
  // Returns an error if there is already a streaming output converter
  // registered.
  template <typename RealtimeT, typename ProtoT,
            typename = std::enable_if_t<
                std::is_base_of_v<google::protobuf::Message, ProtoT>>>
  absl::Status AddStreamingOutputConverter(
      std::function<absl::StatusOr<ProtoT>(const RealtimeT& streaming_output)>
          converter);

  // A variant of AddStreamingOutputConverter for converters for use with ICON's
  // custom Action plugin C API. Unlike AddStreamingOutputConverter() above,
  // converters for this function take a reference to a std::array<char,
  // kMaxStreamingOutputSizeBytes> and produce
  // ::google::protobuf::Any.
  //
  // Most callers should be using AddStreamingOutputConverter, which
  // * takes care of packing / unpacking the output / input automatically
  // * can use its template parameters to
  //   * provide a nicer converter API
  //   * avoid copying unnecessary bytes
  //
  // Returns an error if `proto_type_name` does not match the output message
  // type in our `signature_`.
  //
  // Returns an error if `realtime_type_size` (the size of the data packed into
  // the converter's std::any input value) is larger than
  // kMaxStreamingOutputSizeBytes.
  //
  // Returns an error if there is already a streaming output converter
  // registered.
  absl::Status AddStreamingOutputConverterAny(
      absl::string_view proto_type_name, size_t realtime_type_size,
      std::function<absl::StatusOr<::google::protobuf::Any>(
          const std::array<char, kMaxStreamingOutputSizeBytes>&
              streaming_output)>
          converter);

 private:
  StreamingIoStorage& storage_;
};

template <typename ProtoT, typename RealtimeT, typename>
absl::StatusOr<StreamingInputId> StreamingIoRegistry::AddStreamingInputParser(
    absl::string_view input_name,
    std::function<absl::StatusOr<RealtimeT>(const ProtoT& streaming_input)>
        parser) {
  // Wrap the parser to match the generic interface, and save in an
  // InputChannel.
  auto wrapped_parser = [parser, input_name, action_id = storage_.id_,
                         action_type_name =
                             storage_.signature_.action_type_name()](
                            const google::protobuf::Any& streaming_input)
      -> absl::StatusOr<std::any> {
    ProtoT streaming_input_proto;
    if (!streaming_input.UnpackTo(&streaming_input_proto)) {
      return absl::InvalidArgumentError(
          absl::StrCat("Failed to unpack streaming input '", input_name,
                       "' for Action ", action_id.value(), " of type '",
                       action_type_name, "'. Expected proto message type '",
                       streaming_input_proto.GetDescriptor()->full_name(),
                       "', but got '", streaming_input.GetTypeName(), "'"));
    }
    absl::StatusOr<RealtimeT> parser_result = parser(streaming_input_proto);
    if (!parser_result.ok()) {
      return absl::Status(
          parser_result.status().code(),
          absl::StrCat(parser_result.status().message(), "; ",
                       "Parser for streaming input '", input_name,
                       "' on Action ", action_id.value(), " of type '",
                       action_type_name, "' failed."));
    }
    return std::any(parser_result.value());
  };
  return AddStreamingInputParserAny(input_name,
                                    ProtoT::GetDescriptor()->full_name(),
                                    std::move(wrapped_parser));
}

template <typename RealtimeT, typename ProtoT, typename>
absl::Status StreamingIoRegistry::AddStreamingOutputConverter(
    std::function<absl::StatusOr<ProtoT>(const RealtimeT& streaming_output)>
        converter) {
  static_assert(
      sizeof(RealtimeT) <= kMaxStreamingOutputSizeBytes,
      "RealtimeT must not be larger than kMaxStreamingOutputSizeBytes");

  if (!storage_.signature_.has_streaming_output_info()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Cannot add streaming output converter for Action ",
        storage_.id_.value(), " of type '",
        storage_.signature_.action_type_name(),
        "', since this Action type does not export a streaming output."));
  }
  if (ProtoT::GetDescriptor()->full_name() !=
      storage_.signature_.streaming_output_info().value_message_type()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Cannot register streaming output converter for Action ",
        storage_.id_.value(), " of type '",
        storage_.signature_.action_type_name(),
        "'. This Action type has a streaming output of type '",
        storage_.signature_.streaming_output_info().value_message_type(),
        "', but the converter function takes '",
        ProtoT::GetDescriptor()->full_name(), "'."));
  }

  if (storage_.output_channel_ != nullptr) {
    return absl::AlreadyExistsError(
        absl::StrCat("Cannot register more than one streaming output converter "
                     "for Action ",
                     storage_.id_.value(), " of type '",
                     storage_.signature_.action_type_name(), "'."));
  }

  // Save Converter, and wrap to conform to generic interface.
  storage_.output_channel_ = StreamingOutputChannel::Create<RealtimeT>(
      [converter, action_id = storage_.id_,
       action_type_name = storage_.signature_.action_type_name()](
          const std::any& streaming_output_any)
          -> absl::StatusOr<google::protobuf::Any> {
        const RealtimeT* streaming_output =
            std::any_cast<RealtimeT>(&streaming_output_any);
        if (streaming_output == nullptr) {
          return absl::InternalError(absl::StrCat(
              "Failed to cast streaming output for Action ", action_id.value(),
              " of type '", action_type_name,
              "' from std::any to converter type. This is a bug in ICON!"));
        }

        absl::StatusOr<ProtoT> streaming_output_proto_or =
            converter(*streaming_output);
        if (!streaming_output_proto_or.ok()) {
          const absl::Status& s = streaming_output_proto_or.status();
          return absl::Status(
              s.code(),
              absl::StrCat(s.message(), "; ",
                           "Converter for streaming output on Action ",
                           action_id.value(), " of type '", action_type_name,
                           "' failed."));
        }
        google::protobuf::Any streaming_output_proto_any;
        if (!streaming_output_proto_any.PackFrom(
                streaming_output_proto_or.value())) {
          return absl::InternalError(absl::StrCat(
              "Failed to pack streaming output for Action ", action_id.value(),
              " of type '", action_type_name, "' into an Any proto."));
        }
        return streaming_output_proto_any;
      });
  return absl::OkStatus();
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_STREAMING_IO_REGISTRY_H_
