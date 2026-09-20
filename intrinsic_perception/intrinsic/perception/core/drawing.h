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

#ifndef INTRINSIC_PERCEPTION_CORE_DRAWING_H_
#define INTRINSIC_PERCEPTION_CORE_DRAWING_H_

#include "absl/types/span.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/opencv_wrapper.h"
#include "intrinsic/perception/core/rectangle.h"
#include "opencv2/imgproc.hpp"

namespace intrinsic {
namespace perception {

// Draws 2D crosshair markers on an image centered at the specified points.
// Accepts image by value so rvalues/moved images are modified in-place with
// zero copies.
template <class ImageTrait>
Image<ImageTrait> DrawCrosshairs(Image<ImageTrait> image,
                                 absl::Span<const Vector2f> image_points,
                                 const typename ImageTrait::PixelType& color) {
  constexpr int kOffset = 5;
  const cv::Scalar cv_color = PixelToCvScalar(color);
  for (const auto& point : image_points) {
    cv::line(ToCvMat<ImageTrait>(image),
             cv::Point2f(point.x() - kOffset, point.y() - kOffset),
             cv::Point2f(point.x() + kOffset, point.y() + kOffset), cv_color,
             /* thickness=*/2);
    cv::line(ToCvMat<ImageTrait>(image),
             cv::Point2f(point.x() - kOffset, point.y() + kOffset),
             cv::Point2f(point.x() + kOffset, point.y() - kOffset), cv_color,
             /* thickness=*/2);
  }
  return image;
}

// Convenience overloads with default colors for standard image types.
inline Image<Rgb8u> DrawCrosshairs(
    Image<Rgb8u> image, absl::Span<const Vector2f> image_points,
    const Rgb8u::PixelType& color = Rgb8u::PixelType(0, 255, 255)) {
  return DrawCrosshairs<Rgb8u>(std::move(image), image_points, color);
}

inline Image<Gray8u> DrawCrosshairs(
    Image<Gray8u> image, absl::Span<const Vector2f> image_points,
    const Gray8u::PixelType& color = Gray8u::PixelType(255)) {
  return DrawCrosshairs<Gray8u>(std::move(image), image_points, color);
}

inline Image<Gray32f> DrawCrosshairs(
    Image<Gray32f> image, absl::Span<const Vector2f> image_points,
    const Gray32f::PixelType& color = Gray32f::PixelType(1.0f)) {
  return DrawCrosshairs<Gray32f>(std::move(image), image_points, color);
}

// Draws a 2D bounding box on an image.
// Accepts image by value so rvalues/moved images are modified in-place with
// zero copies.
template <class ImageTrait>
Image<ImageTrait> DrawBoundingBox(Image<ImageTrait> image, const Rectangle& roi,
                                  const typename ImageTrait::PixelType& color) {
  cv::Point p1(roi.origin.col, roi.origin.row);
  cv::Point p2(roi.origin.col + roi.dimensions.cols, roi.origin.row);
  cv::Point p3(roi.origin.col + roi.dimensions.cols,
               roi.origin.row + roi.dimensions.rows);
  cv::Point p4(roi.origin.col, roi.origin.row + roi.dimensions.rows);
  const cv::Scalar cv_color = PixelToCvScalar(color);
  cv::Mat mat = ToCvMat<ImageTrait>(image);
  cv::line(mat, p1, p2, cv_color);
  cv::line(mat, p2, p3, cv_color);
  cv::line(mat, p3, p4, cv_color);
  cv::line(mat, p4, p1, cv_color);
  return image;
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_DRAWING_H_
