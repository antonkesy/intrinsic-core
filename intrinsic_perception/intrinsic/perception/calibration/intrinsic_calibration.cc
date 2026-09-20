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

#include "intrinsic/perception/calibration/intrinsic_calibration.h"

#include <algorithm>
#include <iterator>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/opencv_wrapper.h"
#include "intrinsic/perception/core/range_tools.h"
#include "intrinsic/util/status/status_macros.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"

namespace intrinsic::perception {

absl::StatusOr<IntrinsicCalibrationResult> CalibrateIntrinsics(
    Dimensions dimensions,
    absl::Span<const CalibrationPoints> calibration_points) {
  if (calibration_points.empty()) {
    return absl::InvalidArgumentError("Input points are empty.");
  }
  std::vector<std::vector<cv::Point2f>> cv_image_points;
  cv_image_points.reserve(calibration_points.size());
  std::vector<std::vector<cv::Point3f>> cv_object_points;
  cv_object_points.reserve(calibration_points.size());

  for (const auto& pts : calibration_points) {
    if (pts.image_points.size() != pts.object_points.size()) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Image point size (%d) and object point size (%d) must match.",
          pts.image_points.size(), pts.object_points.size()));
    }
    cv_image_points.push_back(Transformed(
        pts.image_points,
        [](const Vector2f& p) { return cv::Point2f(p.x(), p.y()); }));
    cv_object_points.push_back(Transformed(
        pts.object_points,
        [](const Vector3f& p) { return cv::Point3f(p.x(), p.y(), p.z()); }));
  }

  cv::Mat cv_intrinsic_params;
  cv::Mat cv_distortion_params;
  const cv::Size cv_image_size = cv::Size(dimensions.cols, dimensions.rows);
  LOG(INFO) << "Calibrating with " << cv_object_points.size() << " images.";
  double error = 0.0;
  std::vector<double> errors_per_image;

  INTR_RETURN_IF_ERROR(TryCatchCvFunction([&]() {
    error = cv::calibrateCamera(
        cv_object_points, cv_image_points, cv_image_size, cv_intrinsic_params,
        cv_distortion_params, cv::noArray(), cv::noArray(), cv::noArray(),
        cv::noArray(), errors_per_image);
  }));

  return IntrinsicCalibrationResult{
      .camera_params = {FromCvIntrinsicParams(dimensions, cv_intrinsic_params),
                        FromCvDistortionParams(cv_distortion_params)},
      .error = error,
      .errors_per_image = errors_per_image,
  };
}

}  // namespace intrinsic::perception
