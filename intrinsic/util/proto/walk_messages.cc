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

#include "intrinsic/util/proto/walk_messages.h"

#include <vector>

#include "absl/functional/function_ref.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace {

bool IsMapWithNonMessageValues(const google::protobuf::FieldDescriptor* field) {
  return field->is_map() && field->message_type()->map_value()->type() !=
                                google::protobuf::FieldDescriptor::TYPE_MESSAGE;
}

}  // namespace

absl::Status RecursivelyWalkMessage(
    google::protobuf::Message& message,
    absl::FunctionRef<absl::StatusOr<bool>(google::protobuf::Message&)>
        function) {
  INTR_ASSIGN_OR_RETURN(const bool should_enter, function(message));
  if (!should_enter) {
    return absl::OkStatus();
  }

  const google::protobuf::Reflection* reflection = message.GetReflection();
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  reflection->ListFields(message, &fields);
  for (const google::protobuf::FieldDescriptor* field : fields) {
    // If the field has presence and is not set, skip it.
    if (field->has_presence() && !reflection->HasField(message, field)) {
      continue;
    }
    // Skip non-message types.
    if (field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      continue;
    }
    // Skip maps with non-message values.
    if (IsMapWithNonMessageValues(field)) {
      continue;
    }

    // Note that is_repeated() also returns true for maps.
    if (field->is_repeated()) {
      for (int i = 0; i < reflection->FieldSize(message, field); ++i) {
        google::protobuf::Message* entry =
            reflection->MutableRepeatedMessage(&message, field, i);
        INTR_RETURN_IF_ERROR(RecursivelyWalkMessage(*entry, function));
      }
    } else if (field->type() ==
               google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      INTR_RETURN_IF_ERROR(RecursivelyWalkMessage(
          *reflection->MutableMessage(&message, field), function));
    }
  }
  return absl::OkStatus();
}

}  // namespace intrinsic
