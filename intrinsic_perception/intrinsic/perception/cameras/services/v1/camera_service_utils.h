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

#ifndef INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_SERVICE_UTILS_H_
#define INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_SERVICE_UTILS_H_

#include <string_view>

#include "absl/status/status.h"
#include "intrinsic/perception/cameras/capture_result.h"
#include "intrinsic/perception/proto/v1/camera_service.pb.h"
#include "intrinsic/perception/proto/v1/capture_result.pb.h"
#include "intrinsic/platform/pubsub/pubsub.h"

namespace intrinsic::perception {

constexpr std::string_view kCameraServiceLoggerPrefix = "/camera_service/";

absl::Status StoreCaptureResult(
    intrinsic_proto::perception::v1::CaptureResult&& capture_result,
    const PubSub& pubsub,
    const intrinsic_proto::perception::v1::CaptureRequest& request,
    intrinsic_proto::perception::v1::CaptureResponse& response);

// Encodes the capture result and stores it in the capture response or,
// if requested, in a kvstore.
absl::Status EncodeAndStoreCaptureResult(
    CaptureResult&& capture_result, const PubSub& pubsub,
    const intrinsic_proto::perception::v1::CaptureRequest& request,
    intrinsic_proto::perception::v1::CaptureResponse& response);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CAMERAS_SERVICES_V1_CAMERA_SERVICE_UTILS_H_
