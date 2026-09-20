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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_INTRINSIC_CALIBRATION_UTILS_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_INTRINSIC_CALIBRATION_UTILS_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/intrinsic_params.h"
#include "intrinsic/perception/core/rectangle.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"

namespace intrinsic {
namespace perception {

// Returns the convex hull of a collection of points.
absl::StatusOr<std::vector<Vector2f>> ComputeConvexHull(
    const std::vector<Vector2f>& image_points);

// Returns the region of interest (enclosing axis-aligned rectangle) of a
// collection of image points.
absl::StatusOr<Rectangle> ComputeRoi(const std::vector<Vector2f>& image_points);

// Adds the area given in polygon to the coverage distribution.
//
// Arbitrary convex polygons can be drawn.
absl::StatusOr<Image<Gray8u>> AddPolygonToCoverageMap(
    const std::vector<Vector2f>& polygon, const Image<Gray8u>& coverage_map);

// Removes the area given in polygon from the coverage distribution.
absl::StatusOr<Image<Gray8u>> RemovePolygonFromCoverageMap(
    const std::vector<Vector2f>& polygon, const Image<Gray8u>& coverage_map);

// Generates a heatmap from a given coverage map.
//
// Pixels that aren't covered by the coverage map (value = 0) will get red
// color, while pixels that were covered often enough (> max_count times) will
// receive a green color. Values in between will receive different shades of
// orange.
Image<Rgb8u> GenerateHeatmap(const Image<Gray8u>& coverage_map,
                             const Dimensions& resize, int max_count = 5);

// Compute Pose of a pattern detection.
absl::StatusOr<Pose3d> ComputePoseFromPatternDetection(
    const intrinsic_proto::perception::v1::PatternDetection& pattern_detection,
    const CameraParams& camera_params);

struct Intrinsics3DErrorStats {
  double mean_3d_error;
  double max_3d_error;
};

// Computes a 3D metric for the given intrinsic parameters using
// a set of undistorted pattern detections.
//
// The 3D error calculation consists in computing the length of a segment
// intersecting two rays with a 3D plane. The two rays are defined by the
// reprojection of a pair of corner detections whereas the plane
// is defined by the board pose. Then, we compute the difference between the
// estimated length of the segment and the actual length (GT distance between
// two corner on the board). The error is averaged over the set of pattern
// detections and normalized w.r.t the length and distance of the line segment,
// returning an error corresponding to a line segment which is 1m in length and
// 1m away from the camera.
absl::StatusOr<Intrinsics3DErrorStats>
GetIntrinsics3DErrorFromPatternDetections(
    const std::vector<intrinsic_proto::perception::v1::PatternDetection>&
        pattern_detections,
    const IntrinsicParams& intrinsic_params);

absl::StatusOr<Intrinsics3DErrorStats>
GetIntrinsics3DErrorFromPatternDetections(
    const std::vector<intrinsic_proto::perception::v1::PatternDetection>&
        pattern_detections,
    const CameraParams& camera_params);

// Determines whether two pattern detections are too close to each other.
bool IsRedundantPatternDetection(
    const intrinsic_proto::perception::v1::PatternDetection& pattern_detection,
    const intrinsic_proto::perception::v1::PatternDetection&
        previous_pattern_detection,
    const int pixel_wise_difference_threshold);

// Determines whether the pattern detection is of rank 1, i.e. the detected
// points seem to lie on a line.
bool IsLowRankPatternDetection(
    const intrinsic_proto::perception::v1::PatternDetection& pattern_detection);

struct ReprojectionErrorResult {
  // Sum of squared reprojection errors in pixels squared.
  double squared_error = 0.0;
  // Number of points used in the error calculation.
  int num_points = 0;
  // Maximum reprojection error of a single point in pixels.
  double max_error = 0.0;
};

// Computes the 2D reprojection error for a single pattern detection,
// given the known camera parameters and the known board pose relative to the
// camera.
absl::StatusOr<ReprojectionErrorResult> ComputeReprojectionErrorForDetection(
    const intrinsic_proto::perception::v1::PatternDetection& detection,
    const CameraParams& camera_params, const Pose3d& camera_t_board);

}  // namespace perception
}  // namespace intrinsic
#endif  // INTRINSIC_PERCEPTION_CALIBRATION_INTRINSIC_CALIBRATION_UTILS_H_
