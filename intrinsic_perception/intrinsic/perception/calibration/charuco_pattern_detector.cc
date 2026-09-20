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

#include "intrinsic/perception/calibration/charuco_pattern_detector.h"

#include <memory>
#include <optional>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/calibration/charuco_pattern_utils.h"
#include "intrinsic/perception/calibration/pattern_detector_interface.h"
#include "intrinsic/perception/cameras/capture_result.h"
#include "intrinsic/perception/cameras/capture_result_helper.h"
#include "intrinsic/perception/cameras/sensor_image.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/proto/v1/charuco_pattern.pb.h"
#include "intrinsic/perception/proto/v1/pattern_detection_config.pb.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"
#include "intrinsic/perception/proto_conversion/vector.h"
#include "intrinsic/util/status/status_macros.h"
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/objdetect/aruco_board.hpp"

namespace intrinsic::perception {

absl::StatusOr<std::unique_ptr<PatternDetectorInterface>>
CharucoPatternDetector::Create(
    const intrinsic_proto::perception::v1::PatternDetectionConfig&
        pattern_detector_config) {
  auto detector = std::make_unique<CharucoPatternDetector>();
  if (!pattern_detector_config.has_charuco_pattern_detection_config()) {
    return absl::InternalError("no charuco_pattern_detection_config specified");
  }
  detector->charuco_pattern_ =
      pattern_detector_config.charuco_pattern_detection_config()
          .charuco_pattern();

  INTR_ASSIGN_OR_RETURN(cv::Ptr<cv::aruco::CharucoBoard> charucoboard,
                        CreateCharucoBoard(detector->charuco_pattern_));
  detector->chessboard_corners_.reserve(
      charucoboard->getChessboardCorners().size());
  for (const auto& corner : charucoboard->getChessboardCorners()) {
    detector->chessboard_corners_.emplace_back(
        Vector3f(corner.x, corner.y, corner.z));
  }
  return detector;
}

absl::StatusOr<intrinsic_proto::perception::v1::PatternDetectionResult>
CharucoPatternDetector::GetPatternDetectionsImpl(
    const CaptureResult& capture_result) const {
  INTR_ASSIGN_OR_RETURN(
      const SensorImage* absl_nonnull sensor_image,
      (GetFirstSensorImageOfType<Rgb8u, Gray8u, Gray32f>(capture_result)));
  INTR_ASSIGN_OR_RETURN(const Image<Rgb8u>& image,
                        sensor_image->rgb8u().has_value()
                            ? sensor_image->rgb8u().value()
                            : GetImageAsRgb8u(*sensor_image));
  const auto charuco_detection_or_status =
      FindCharucoMarker(image, charuco_pattern_, std::nullopt);
  if (absl::IsNotFound(charuco_detection_or_status.status())) {
    return intrinsic_proto::perception::v1::PatternDetectionResult();
  } else if (!charuco_detection_or_status.ok()) {
    return charuco_detection_or_status.status();
  }
  const CharucoDetection& charuco_detection =
      charuco_detection_or_status.value();
  if (charuco_detection.chessboard_ids.empty()) {
    return intrinsic_proto::perception::v1::PatternDetectionResult();
  }
  // Convert results.
  intrinsic_proto::perception::v1::PatternDetectionResult result;
  auto* pattern_detection = result.add_pattern_detections();
  for (int i = 0; i < charuco_detection.chessboard_ids.size(); ++i) {
    auto* image_point = pattern_detection->add_image_points();
    *image_point = ToProto(Vector2f(charuco_detection.chessboard_corners[i].x,
                                    charuco_detection.chessboard_corners[i].y));
    auto* model_point = pattern_detection->add_marker_points();
    *model_point =
        ToProto(chessboard_corners_[charuco_detection.chessboard_ids[i]]);
    pattern_detection->add_marker_ids(charuco_detection.chessboard_ids[i]);
  }
  return result;
}

}  // namespace intrinsic::perception
