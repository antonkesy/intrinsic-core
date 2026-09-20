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

#include "intrinsic/perception/cameras/services/v1/camera_service_utils.h"

#include <utility>

#include "absl/status/status.h"
#include "intrinsic/perception/cameras/capture_result.h"
#include "intrinsic/perception/proto/v1/camera_service.pb.h"
#include "intrinsic/perception/proto/v1/capture_result.pb.h"
#include "intrinsic/perception/proto_conversion/v1/capture_result.h"
#include "intrinsic/perception/proto_conversion/v1/post_processing.h"
#include "intrinsic/platform/pubsub/kvstore.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/stats/scoped_span.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {

absl::Status StoreCaptureResult(
    intrinsic_proto::perception::v1::CaptureResult&& capture_result,
    const PubSub& pubsub,
    const intrinsic_proto::perception::v1::CaptureRequest& request,
    intrinsic_proto::perception::v1::CaptureResponse& response) {
  if (request.has_capture_result_location()) {
    INTR_ASSIGN_OR_RETURN(
        KeyValueStore kvstore,
        pubsub.KeyValueStore(request.capture_result_location().store()));
    INTR_RETURN_IF_ERROR(kvstore.Set(request.capture_result_location().key(),
                                     std::move(capture_result)));
    *response.mutable_capture_result_location() =
        request.capture_result_location();
  } else {
    *response.mutable_capture_result() = std::move(capture_result);
  }
  return absl::OkStatus();
}

absl::Status EncodeAndStoreCaptureResult(
    CaptureResult&& capture_result, const PubSub& pubsub,
    const intrinsic_proto::perception::v1::CaptureRequest& request,
    intrinsic_proto::perception::v1::CaptureResponse& response) {
  auto [_, encoding_by_sensor_id] =
      FromProto(request.post_processing_by_sensor_id());
  intrinsic_proto::perception::v1::CaptureResult capture_result_proto;
  {
    const stats::ScopedSpan encode_span("CameraService::EncodeImages");
    INTR_ASSIGN_OR_RETURN(capture_result_proto,
                          intrinsic_proto::perception::v1::ToProto(
                              capture_result, encoding_by_sensor_id));
  }
  INTR_RETURN_IF_ERROR(StoreCaptureResult(std::move(capture_result_proto),
                                          pubsub, request, response));
  return absl::OkStatus();
}

}  // namespace intrinsic::perception
