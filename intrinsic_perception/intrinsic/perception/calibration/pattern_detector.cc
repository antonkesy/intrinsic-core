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

#include "intrinsic/perception/calibration/pattern_detector.h"

#include <memory>
#include <optional>
#include <utility>

#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "grpcpp/server_context.h"
#include "intrinsic/perception/calibration/pattern_detector_factory.h"
#include "intrinsic/perception/calibration/pattern_detector_interface.h"
#include "intrinsic/perception/cameras/capture_result.h"
#include "intrinsic/perception/cameras/capture_result_helper.h"
#include "intrinsic/perception/cameras/sensor_image.h"
#include "intrinsic/perception/core/config_name_utils.h"
#include "intrinsic/perception/core/drawing.h"
#include "intrinsic/perception/core/encoding.h"
#include "intrinsic/perception/core/image_publisher.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/logging/pattern_detection.h"
#include "intrinsic/perception/proto/v1/image_buffer.pb.h"
#include "intrinsic/perception/proto/v1/pattern_detection_config.pb.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"
#include "intrinsic/perception/proto_conversion/v1/image_buffer.h"
#include "intrinsic/perception/proto_conversion/vector.h"
#include "intrinsic/stats/scoped_span.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {

namespace {
absl::Status MaybeAnnotateAndPublishSensorImage(
    const SensorImage& image,
    const intrinsic_proto::perception::v1::PatternDetectionConfig& config,
    grpc::ServerContext* context,
    intrinsic_proto::perception::v1::PatternDetectionResult& result) {
  if (config.publish_annotated_image()) {
    Image<Rgb8u> annotated_image;
    {
      const stats::ScopedSpan span("PatternDetectorInterface::Run::Annotate",
                                   context);
      INTR_RET_CHECK(image.camera_params().has_value());
      INTR_ASSIGN_OR_RETURN(annotated_image, GetImageAsRgb8u(image));
      std::vector<Vector2f> points;
      for (const auto& detection : result.pattern_detections()) {
        for (const auto& p : detection.image_points()) {
          points.push_back(FromProto(p));
        }
      }
      annotated_image = DrawCrosshairs(std::move(annotated_image), points);
    }
    {
      const stats::ScopedSpan span("PatternDetectorInterface::Run::Encode",
                                   context);
      INTR_ASSIGN_OR_RETURN(
          intrinsic_proto::perception::v1::ImageBuffer encoded_image,
          intrinsic_proto::perception::v1::ToProto(annotated_image,
                                                   Encoding::kJpeg));
      INTR_RETURN_IF_ERROR(
          PublishImage(encoded_image, "/perception/annotated_frame"));
    }
  }
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<PatternDetector> PatternDetector::Create(
    const intrinsic_proto::perception::v1::PatternDetectionConfig&
        pattern_detection_config) {
  INTR_RET_CHECK_NE(
      pattern_detection_config.Type_case(),
      intrinsic_proto::perception::v1::PatternDetectionConfig::TYPE_NOT_SET)
          .SetCode(absl::StatusCode::kInvalidArgument)
      << "Pattern detector configuration not set.";

  INTR_ASSIGN_OR_RETURN(auto pattern_detector_interface,
                        CreatePatternDetector(pattern_detection_config));
  auto executor = std::make_unique<SingleThreadExecutor>();
  return PatternDetector(std::move(pattern_detector_interface),
                         pattern_detection_config, std::move(executor));
}

PatternDetector::PatternDetector(
    std::unique_ptr<PatternDetectorInterface> pattern_detector_interface,
    const intrinsic_proto::perception::v1::PatternDetectionConfig&
        pattern_detection_config,
    std::unique_ptr<SingleThreadExecutor> executor)
    : pattern_detector_interface_(std::move(pattern_detector_interface)),
      pattern_detection_config_(pattern_detection_config),
      executor_(std::move(executor)) {
  CHECK_NE(pattern_detector_interface_, nullptr) << "Programming error.";
}

absl::StatusOr<intrinsic_proto::perception::v1::PatternDetectionResult>
PatternDetector::Run(const CaptureResult& capture_result,
                     grpc::ServerContext* context) {
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::perception::v1::PatternDetectionResult result,
      pattern_detector_interface_->Run(capture_result));

  INTR_ASSIGN_OR_RETURN(
      const SensorImage* absl_nonnull sensor_image,
      (GetFirstSensorImageOfType<Rgb8u, Gray8u, Gray32f>(capture_result)));
  INTR_RETURN_IF_ERROR(MaybeAnnotateAndPublishSensorImage(
      *sensor_image, pattern_detection_config_, context, result));

  if (executor_ != nullptr) {
    const stats::ScopedSpan span(
        "PatternDetectorInterface::Run::LogAnnotatedFrame", context);
    INTR_RETURN_IF_ERROR(LogResult(
        *executor_, {sensor_image, 1}, result,
        ExtractPrefixFromConfigName(pattern_detection_config_.name())));
  }

  return result;
}

}  // namespace intrinsic::perception
