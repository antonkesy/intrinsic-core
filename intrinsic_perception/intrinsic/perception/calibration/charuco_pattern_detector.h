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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_CHARUCO_PATTERN_DETECTOR_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_CHARUCO_PATTERN_DETECTOR_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/perception/calibration/pattern_detector_interface.h"
#include "intrinsic/perception/cameras/capture_result.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/proto/v1/charuco_pattern.pb.h"
#include "intrinsic/perception/proto/v1/pattern_detection_config.pb.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"

namespace intrinsic::perception {

// Internal implementation of ChArUco pattern detection.
class CharucoPatternDetector : public PatternDetectorInterface {
 public:
  // Create pattern detector instance.
  static absl::StatusOr<std::unique_ptr<PatternDetectorInterface>> Create(
      const intrinsic_proto::perception::v1::PatternDetectionConfig&
          pattern_detector_config);

 private:
  absl::StatusOr<intrinsic_proto::perception::v1::PatternDetectionResult>
  GetPatternDetectionsImpl(const CaptureResult& capture_result) const override;

  intrinsic_proto::perception::v1::CharucoPattern charuco_pattern_;
  std::vector<Vector3f> chessboard_corners_;
};

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CALIBRATION_CHARUCO_PATTERN_DETECTOR_H_
