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

#ifndef INTRINSIC_UTIL_PROTO_WALK_MESSAGES_H_
#define INTRINSIC_UTIL_PROTO_WALK_MESSAGES_H_

#include "absl/functional/function_ref.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"

namespace intrinsic {

// Recursively walks through a proto message, executing a function for each
// message it finds. The provided function returns whether to enter into the
// message recursively. The input message may be mutated.
absl::Status RecursivelyWalkMessage(
    google::protobuf::Message& message,
    absl::FunctionRef<absl::StatusOr<bool>(google::protobuf::Message&)>
        function);

}  // namespace intrinsic

#endif  // INTRINSIC_UTIL_PROTO_WALK_MESSAGES_H_
