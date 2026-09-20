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

#include "intrinsic/perception/logging/pattern_detection.h"

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/perception/cameras/capture_result_helper.h"
#include "intrinsic/perception/cameras/sensor_image.h"
#include "intrinsic/perception/core/drawing.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/single_thread_executor.h"
#include "intrinsic/perception/logging/image.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"
#include "intrinsic/perception/proto_conversion/vector.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {

absl::Status LogResult(
    const SingleThreadExecutor& executor,
    absl::Span<const SensorImage> sensor_images,
    const intrinsic_proto::perception::v1::PatternDetectionResult& result,
    std::string_view prefix) {
  const std::string full_prefix = absl::StrCat(prefix, "annotated/");

  for (const SensorImage& sensor_image : sensor_images) {
    absl::StatusOr<Image<Rgb8u>> annotated_image =
        GetImageAsRgb8u(sensor_image);
    if (!annotated_image.ok()) {
      LOG(WARNING) << annotated_image.status();
      continue;
    }
    std::vector<Vector2f> points;
    for (const intrinsic_proto::perception::v1::PatternDetection& detection :
         result.pattern_detections()) {
      for (const intrinsic_proto::perception::Vector2f& p :
           detection.image_points()) {
        points.push_back(FromProto(p));
      }
    }
    Image<Rgb8u> image_with_crosshairs =
        DrawCrosshairs(std::move(*annotated_image), points);
    const std::string filename_prefix =
        absl::StrCat(full_prefix, sensor_image.sensor_id(), "_",
                     absl::ToUnixMillis(sensor_image.acquisition_time()));
    INTR_RETURN_IF_ERROR(LogImage(executor, image_with_crosshairs,
                                  filename_prefix,
                                  /*context=*/{}, kAnnotatedFramesEventSource)
                             .status());
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::perception
