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

#include "intrinsic/icon/control/c_api/wrappers/action_factory_context_wrapper.h"

#include <any>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/c_api/c_action_factory_context.h"
#include "intrinsic/icon/control/c_api/c_realtime_status.h"
#include "intrinsic/icon/control/c_api/c_types.h"
#include "intrinsic/icon/control/c_api/convert_c_realtime_status.h"
#include "intrinsic/icon/control/c_api/wrappers/string_wrapper.h"
#include "intrinsic/icon/control/realtime_signal_types.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_registry.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

static_assert(
    kIntrinsicIconMaxStreamingOutputSizeBytes ==
        StreamingIoRegistry::kMaxStreamingOutputSizeBytes,
    "kIntrinsicIconMaxStreamingOutputSizeBytes "
    "(intrinsic/icon/control/c_api/c_action_factory_context.h) and "
    "::intrinsic::icon::StreamingIoRegistry::kMaxStreamingOutputSizeBytes "
    "(intrinsic/icon/control/streaming_io_registry.h) are out of "
    "sync!");

// 1. Unwraps `self` to an ActionFactoryContext*
// 2. Calls `ServerConfig()` on it
// 3. Serializes the resulting proto into a string
// 4. Returns the serialized proto as an IntrinsicIconString, which the caller
// takes
//    ownership of.
IntrinsicIconString* GetServerConfig(
    const IntrinsicIconActionFactoryContext* self) {
  intrinsic_proto::icon::v1::ServerConfig server_config =
      reinterpret_cast<const ActionFactoryContext*>(self)->ServerConfig();

  return Wrap(server_config.SerializeAsString());
}

// 1.  Unwraps `self` to an ActionFactoryContext*
// 2.  Calls `GetSlotInfo(slot_name)` on it
// 3a. On success, writes the SlotInfo data into `slot_info_out` and returns
//     OkStatus. The caller takes ownership of the proto string in
//     `part_info_out` and is responsible for cleaning it up!
// 3b. On error, returns the error status and does not touch `slot_info_out`. No
//     cleanup required for `slot_info_out` in this case!
IntrinsicIconRealtimeStatus GetSlotInfo(IntrinsicIconActionFactoryContext* self,
                                        IntrinsicIconStringView slot_name,
                                        IntrinsicIconSlotInfo* slot_info_out) {
  absl::StatusOr<SlotInfo> slot_info =
      reinterpret_cast<ActionFactoryContext*>(self)->GetSlotInfo(
          absl::string_view(slot_name.data, slot_name.size));
  if (!slot_info.ok()) {
    return FromAbslStatus(slot_info.status());
  }

  slot_info_out->realtime_slot_id = slot_info->slot_id.value();
  slot_info_out->part_config_buffer =
      Wrap(slot_info->config.SerializeAsString());

  return FromAbslStatus(OkStatus());
}

// 1.  Unwraps `self` to an ActionFactoryContext*
// 2.  Calls `GetRealtimeSignalId(signal_name)` on it
// 3a. On success, writes the RealtimeSignalId data into `signal_id_out` and
//     returns OkStatus.
// 3b. On error, returns the error status and does not touch `signal_id_out`.
IntrinsicIconRealtimeStatus GetRealtimeSignalId(
    IntrinsicIconActionFactoryContext* self,
    IntrinsicIconStringView signal_name, uint64_t* signal_id_out) {
  absl::StatusOr<RealtimeSignalId> signal_id =
      reinterpret_cast<ActionFactoryContext*>(self)->GetRealtimeSignalId(
          absl::string_view(signal_name.data, signal_name.size));
  if (!signal_id.ok()) {
    return FromAbslStatus(signal_id.status());
  }

  *signal_id_out = signal_id->value();

  return FromAbslStatus(OkStatus());
}

// Functor that calls `destroy` on a T instance, for use with smart pointers.
template <typename T>
struct DestroyFunctor {
  void operator()(T* self) {
    if (self != nullptr) destroy(self);
  }
  void (*destroy)(T* self);
};

// 1. Wraps `parser` into a lambda that conforms to the C++ StreamingInputParser
//    API.
// 2. Casts `self` to ActionFactoryContext*.
// 3. Calls `AddStreamingInputParserAny` on it.
// 4. Returns the Status of the call, and if Ok, writes the streaming input ID
//    to `streaming_input_id_out`.
IntrinsicIconRealtimeStatus AddStreamingInputParser(
    IntrinsicIconActionFactoryContext* self, IntrinsicIconStringView input_name,
    IntrinsicIconStringView input_proto_message_type_name,
    IntrinsicIconStreamingInputParserFnInstance parser,
    uint64_t* streaming_input_id_out) {
  // First, wrap the parser function pointer into a lambda that adapts from the
  // C API to the signature that
  // ActionFactoryContext::AddStreamingInputParserAny() expects.
  auto parser_wrapped =
      // Using shared_ptr instead of unique_ptr to hold the parser function
      // makes sure the lambda is copyable. Semantically, it's correct for any
      // copies to use the same parser function, and from a realtime standpoint,
      // the lambda is not copied or destroyed in the raeltime thread, so
      // there's no danger of shared_ptr's ctor or dtor blocking the realtime
      // thread.
      [parser_ptr = std::shared_ptr<IntrinsicIconStreamingInputParserFn>(
           parser.self,
           DestroyFunctor<IntrinsicIconStreamingInputParserFn>{
               .destroy = parser.destroy}),
       invoke = parser.invoke, destroy_input = parser.destroy_input](
          const ::google::protobuf::Any& input_any)
      -> absl::StatusOr<std::any> {
    // Prepare the parameters for the C API parser function.
    const std::string input_serialized = input_any.SerializeAsString();
    IntrinsicIconRealtimeStatus parse_status;
    // parsed_input is a shared_ptr, because absl::any can't hold a unique_ptr
    // (unique_ptr is not copyable).
    //
    // TODO(b/221390061): Consider adding a copy/clone function to the C API for
    // IntrinsicIconStreamingInputType, and using a custom wrapper class instead
    // of shared_ptr. However, we never modify any copies, and, shared_ptr or
    // not, would need to make sure that the destructor function is never called
    // in the realtime thread, so we probably don't gain much from another
    // wrapper type.
    WrappedCApiStreamingInputRealtimeType parsed_input(
        invoke(parser_ptr.get(), WrapView(input_serialized), &parse_status),
        DestroyFunctor<IntrinsicIconStreamingInputType>{.destroy =
                                                            destroy_input});
    // If this returns early, shared_ptr's dtor cleans up parsed_input, if the
    // parser returned a non-null pointer.
    INTR_RETURN_IF_ERROR(ToAbslStatus(parse_status));

    return std::any(std::move(parsed_input));
  };

  auto result =
      reinterpret_cast<ActionFactoryContext*>(self)->AddStreamingInputParserAny(
          absl::string_view(input_name.data, input_name.size),
          absl::string_view(input_proto_message_type_name.data,
                            input_proto_message_type_name.size),
          std::move(parser_wrapped));
  if (!result.ok()) {
    return FromAbslStatus(result.status());
  }
  *streaming_input_id_out = result->value();
  return FromAbslStatus(absl::OkStatus());
}

// 1. Wraps `converter` into a lambda that conforms to the C++
//    StreamingOutputConverter API.
// 2. Casts `self` to ActionFactoryContext*.
// 3. Calls `AddStreamingOutputConverterAny` on it.
// 4. Returns the Status of the call.
IntrinsicIconRealtimeStatus AddStreamingOutputConverter(
    IntrinsicIconActionFactoryContext* self,
    IntrinsicIconStringView output_proto_message_type_name,
    size_t realtime_type_size,
    IntrinsicIconStreamingOutputConverterFnInstance converter) {
  auto converter_wrapped =
      // Same reasoning as for StreamingInputParsers applies here: converter_ptr
      // must be a shared_ptr so that the resulting lambda is copyable, and it's
      // safe to use shared_ptr because callbacks are only copied/destroyes in
      // non-realtime.
      [converter_ptr = std::shared_ptr<IntrinsicIconStreamingOutputConverterFn>(
           converter.self,
           DestroyFunctor<IntrinsicIconStreamingOutputConverterFn>{
               .destroy = converter.destroy}),
       destroy_string = converter.destroy_string, invoke = converter.invoke,
       realtime_type_size](
          const WrappedCApiStreamingOutputRealtimeType& streaming_output)
      -> absl::StatusOr<::google::protobuf::Any> {
    // Check the size of the output. Non-Plugin Actions can do this at compile
    // time via templates, but for the C API, we must check at runtime.
    if (realtime_type_size > streaming_output.size()) {
      return absl::InvalidArgumentError(absl::StrCat(
          "realtime_type_size (", realtime_type_size,
          ") exceeds size of container (", streaming_output.size(), ")"));
    }
    IntrinsicIconRealtimeStatus result_status;
    // Automatically call destroy_string at when we leave this scope.
    std::unique_ptr<IntrinsicIconString, DestroyFunctor<IntrinsicIconString>>
        output_string(
            invoke(converter_ptr.get(),
                   reinterpret_cast<const IntrinsicIconStreamingOutputType*>(
                       streaming_output.data()),
                   realtime_type_size, &result_status),
            DestroyFunctor<IntrinsicIconString>{.destroy = destroy_string});
    INTR_RETURN_IF_ERROR(ToAbslStatus(result_status));
    ::google::protobuf::Any output_any;
    if (!output_any.ParseFromString(
            absl::string_view(output_string->data, output_string->size))) {
      return absl::InternalError(
          "Failed to parse result of streaming output parser.");
    }
    return output_any;
  };

  return FromAbslStatus(
      reinterpret_cast<ActionFactoryContext*>(self)
          ->AddStreamingOutputConverterAny(
              absl::string_view(output_proto_message_type_name.data,
                                output_proto_message_type_name.size),
              realtime_type_size, std::move(converter_wrapped)));
}

}  // namespace

IntrinsicIconActionFactoryContext* Wrap(
    ActionFactoryContext* action_factory_context) {
  return reinterpret_cast<IntrinsicIconActionFactoryContext*>(
      action_factory_context);
}

IntrinsicIconActionFactoryContextVtable GetActionFactoryContextVtable() {
  return {
      .destroy_string = DestroyString,
      .server_config = GetServerConfig,
      .get_slot_info = GetSlotInfo,
      .get_realtime_signal_id = GetRealtimeSignalId,
      .add_streaming_input_parser = AddStreamingInputParser,
      .add_streaming_output_converter = AddStreamingOutputConverter,
  };
}

}  // namespace intrinsic::icon
