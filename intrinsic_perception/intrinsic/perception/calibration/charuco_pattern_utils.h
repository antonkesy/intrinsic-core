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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_CHARUCO_PATTERN_UTILS_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_CHARUCO_PATTERN_UTILS_H_

#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/perception/core/camera_params.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/mesh.h"
#include "intrinsic/perception/proto/v1/charuco_pattern.pb.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/objdetect/aruco_board.hpp"

namespace intrinsic::perception {

struct CharucoDetection {
  std::vector<int> aruco_ids;
  // Vector containing the 4 ArUco marker corners (saved as a vector of points).
  std::vector<std::vector<cv::Point2f>> aruco_corners;
  std::vector<int> chessboard_ids;
  // Vector of chessboard corners (each corner saved as a point).
  std::vector<cv::Point2f> chessboard_corners;
};

// Creates the OpenCV representation of a CharucoBoard from a
// proto::CharucoPattern.
absl::StatusOr<cv::Ptr<cv::aruco::CharucoBoard>> CreateCharucoBoard(
    const intrinsic_proto::perception::v1::CharucoPattern& charuco_pattern);

// Detects a ChArUco marker in an image.
absl::StatusOr<CharucoDetection> FindCharucoMarker(
    const Image<Rgb8u>& image,
    const intrinsic_proto::perception::v1::CharucoPattern& charuco_pattern,
    const std::optional<CameraParams>& camera_params);

// Draws a ChArUco detection onto an image.
absl::StatusOr<Image<Rgb8u>> DrawCharucoBoard(
    const Image<Rgb8u>& image, const CharucoDetection& detection,
    const Rgb8u::PixelType& color = Rgb8u::PixelType(0, 0, 255));

// Detects and draws a ChArUco detection onto an image.
absl::StatusOr<Image<Rgb8u>> FindAndDrawCharucoBoard(
    const Image<Rgb8u>& image,
    const intrinsic_proto::perception::v1::CharucoPattern& charuco_pattern,
    const std::optional<CameraParams>& camera_params,
    const Rgb8u::PixelType& color = Rgb8u::PixelType(0, 0, 255));

struct CreateCharucoMeshOptions {
  // Add white margin around the board. If 0 or less, no margin is added.
  float margin = 0.01f;
  // Texture size in pixel (width/height). Must be a power of two.
  int texture_size = 8192;
};

// Creates charuco board mesh from a pattern descriptor with an optional white
// margin. The returned mesh is a planar quad centered at 0 with width/height
// determined by the pattern descriptor (+ the optional margin in meters).
absl::StatusOr<Mesh> CreateCharucoMesh(
    const intrinsic_proto::perception::v1::CharucoPattern& pattern,
    const CreateCharucoMeshOptions& options = CreateCharucoMeshOptions{});

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CALIBRATION_CHARUCO_PATTERN_UTILS_H_
