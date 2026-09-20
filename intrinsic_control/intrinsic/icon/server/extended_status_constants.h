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

#ifndef INTRINSIC_ICON_SERVER_EXTENDED_STATUS_CONSTANTS_H_
#define INTRINSIC_ICON_SERVER_EXTENDED_STATUS_CONSTANTS_H_

#include "absl/strings/string_view.h"

// This header defines constants for extended status codes and messages used by
// the ICON server.
//
// Use kExtendedStatusComponent and kStatusSpecs to initialize extended status
// specs in any ICON process, such as test fixture, using the
// `intrinsic::InitExtendedStatusSpecs` function.
namespace intrinsic::icon {

constexpr absl::string_view kExtendedStatusComponent = "ai.intrinsic.icon";

// TODO(b/378051945) Migrate to defining this in the resource manifests when
// possible.
constexpr absl::string_view kStatusSpecs = R"pb(
  status_info: { code: 10001 title: "Failed to poll reactions" }
  status_info: { code: 10002 title: "Failed to handle open session request" }
)pb";

enum ExtendedStatusCodes {
  kFailedToPollReactions = 10001,
  kFailedToHandleOpenSessionRequest = 10002,
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_SERVER_EXTENDED_STATUS_CONSTANTS_H_
