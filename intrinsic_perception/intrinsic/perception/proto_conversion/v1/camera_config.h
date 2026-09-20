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

#ifndef INTRINSIC_PERCEPTION_PROTO_CONVERSION_V1_CAMERA_CONFIG_H_
#define INTRINSIC_PERCEPTION_PROTO_CONVERSION_V1_CAMERA_CONFIG_H_

#include <utility>

#include "absl/status/statusor.h"
#include "intrinsic/perception/cameras/camera_config.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/proto/v1/camera_config.pb.h"

namespace intrinsic_proto::perception::v1 {

absl::StatusOr<std::pair<intrinsic::perception::CameraConfig,
                         intrinsic::perception::CameraIdentifier>>
FromProto(const CameraConfig& camera_config);

CameraConfig ToProto(
    const intrinsic::perception::CameraConfig& camera_config,
    const intrinsic::perception::CameraIdentifier& camera_identifier);

}  // namespace intrinsic_proto::perception::v1

// Convenience ToProto aliases to allow argument-dependent lookup. Directly use
// the versioned namespace alternatives above instead when dealing with
// different versions at the same time.
namespace intrinsic::perception {

using ::intrinsic_proto::perception::v1::ToProto;

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_PROTO_CONVERSION_V1_CAMERA_CONFIG_H_
