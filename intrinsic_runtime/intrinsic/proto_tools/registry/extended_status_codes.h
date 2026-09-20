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

#ifndef INTRINSIC_PROTO_TOOLS_REGISTRY_EXTENDED_STATUS_CODES_H_
#define INTRINSIC_PROTO_TOOLS_REGISTRY_EXTENDED_STATUS_CODES_H_

#include <string_view>

namespace intrinsic::proto_registry {

constexpr std::string_view kExtendedStatusComponent =
    "ai.intrinsic.proto_registry";
constexpr std::string_view kExtendedStatusFile =
    "intrinsic_runtime/intrinsic/proto_tools/registry/"
    "status_specs_message.binarypb";

}  // namespace intrinsic::proto_registry

#endif  // INTRINSIC_PROTO_TOOLS_REGISTRY_EXTENDED_STATUS_CODES_H_
