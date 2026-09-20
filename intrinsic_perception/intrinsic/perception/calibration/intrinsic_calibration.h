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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_INTRINSIC_CALIBRATION_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_INTRINSIC_CALIBRATION_H_

#include <type_traits>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/range_tools.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"
#include "intrinsic/perception/proto_conversion/vector.h"

namespace intrinsic::perception {

struct CalibrationPoints {
  std::vector<Vector2f> image_points;
  std::vector<Vector3f> object_points;
};

struct IntrinsicCalibrationResult {
  CameraParams camera_params;
  double error;
  std::vector<double> errors_per_image;
};

absl::StatusOr<IntrinsicCalibrationResult> CalibrateIntrinsics(
    Dimensions dimensions,
    absl::Span<const CalibrationPoints> calibration_points);

template <typename PatternDetectionRange,
          typename = std::enable_if_t<std::is_same_v<
              typename PatternDetectionRange::value_type,
              intrinsic_proto::perception::v1::PatternDetection>>>
absl::StatusOr<IntrinsicCalibrationResult> CalibrateIntrinsics(
    Dimensions dimensions, const PatternDetectionRange& pattern_detections) {
  std::vector<CalibrationPoints> calibration_points;
  calibration_points.reserve(pattern_detections.size());
  for (const auto& detection : pattern_detections) {
    if (!detection.image_points().empty()) {
      calibration_points.push_back({
          .image_points =
              Transformed(detection.image_points(),
                          [](const auto& p) { return FromProto(p); }),
          .object_points =
              Transformed(detection.marker_points(),
                          [](const auto& p) { return FromProto(p); }),
      });
    }
  }
  return CalibrateIntrinsics(dimensions, calibration_points);
}

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CALIBRATION_INTRINSIC_CALIBRATION_H_
