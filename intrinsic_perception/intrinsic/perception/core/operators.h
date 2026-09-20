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

#ifndef INTRINSIC_PERCEPTION_CORE_OPERATORS_H_
#define INTRINSIC_PERCEPTION_CORE_OPERATORS_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "intrinsic/perception/core/coordinate.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/opencv_wrapper.h"
#include "intrinsic/perception/core/post_processing.h"
#include "intrinsic/perception/core/rectangle.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "opencv2/core.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace intrinsic {
namespace perception {
namespace details {

template <class ImageTrait>
struct OperatorTraits {
  // Scalar multiplication and element-wise addition/subtraction is defined.
  // Operators don't check boundary conditions for the pixel types.
  static constexpr bool kIsVector = true;
};

template <class T>
struct OperatorTraits<NormalTrait<T>> {
  // Scalar multiplication and element-wise addition/subtraction is not defined
  // since this is a normal.
  static constexpr bool kIsVector = false;
};

}  // namespace details

// Multiplies an image by a factor.
template <class ImageTrait>
Image<ImageTrait> operator*(const Image<ImageTrait>& image, double factor) {
  static_assert(details::OperatorTraits<ImageTrait>::kIsVector,
                "scalar multiplication must be defined");
  return MoveToImage<ImageTrait>(factor * UnsafeConstCastCvMat(image));
}

// Multiplies an image by a factor.
template <class ImageTrait>
Image<ImageTrait> operator*(double factor, const Image<ImageTrait>& image) {
  static_assert(details::OperatorTraits<ImageTrait>::kIsVector,
                "scalar multiplication must be defined");
  return MoveToImage<ImageTrait>(factor * UnsafeConstCastCvMat(image));
}

// Multiplies an image by a factor.
template <class ImageTrait>
Image<ImageTrait>& operator*=(Image<ImageTrait>& image, double factor) {
  static_assert(details::OperatorTraits<ImageTrait>::kIsVector,
                "scalar multiplication must be defined");
  ToCvMat(&image) *= factor;
  return image;
}

// TODO(hinterst): Add other ways for interpolating.
// Resizes an image to a new dimension with area interpolation.
template <class ImageTrait>
Image<ImageTrait> Resize(const Image<ImageTrait>& image,
                         const Dimensions& dimension,
                         int flag = cv::INTER_AREA) {
  CHECK(dimension.cols > 0 && dimension.rows > 0)
      << "New dimension must be valid.";
  Image<ImageTrait> resized(dimension);
  cv::resize(UnsafeConstCastCvMat(image), ToCvMat(resized),
             DimensionsToCvSize(dimension), 0, 0, flag);
  return resized;
}

// Resizes an image by a factor with area interpolation.
template <class ImageTrait>
Image<ImageTrait> Resize(const Image<ImageTrait>& image, double factor) {
  Dimensions dimension(static_cast<int>(std::round(factor * image.cols())),
                       static_cast<int>(std::round(factor * image.rows())));
  return Resize(image, dimension);
}

template <class ImageTrait>
Image<ImageTrait> Resize(const Image<ImageTrait>& image, double factorx,
                         double factory) {
  Dimensions dimension(static_cast<int>(std::round(factorx * image.cols())),
                       static_cast<int>(std::round(factory * image.rows())));
  Image<ImageTrait> resized(dimension);
  cv::resize(UnsafeConstCastCvMat(image), ToCvMat(resized),
             DimensionsToCvSize(dimension), factorx, factory, cv::INTER_AREA);
  return resized;
}

// Image operator macros for +, -, &, | and +=, -=, &=, |=.
// These are operations between two images.
// Note: not all operations are commutative (e.g. subtract)
#define IMAGE_OPERATOR(op, name)                                      \
  template <class ImageTrait>                                         \
  Image<ImageTrait> name(const Image<ImageTrait>& lhs,                \
                         const Image<ImageTrait>& rhs) {              \
    static_assert(details::OperatorTraits<ImageTrait>::kIsVector,     \
                  "name##s must be defined");                         \
    CHECK(lhs.dimensions() == rhs.dimensions())                       \
        << "Input image shapes are not equal!";                       \
    return MoveToImage<ImageTrait>(UnsafeConstCastCvMat(lhs)          \
                                       op UnsafeConstCastCvMat(rhs)); \
  }                                                                   \
                                                                      \
  template <class ImageTrait>                                         \
  Image<ImageTrait> operator op(const Image<ImageTrait>& lhs,         \
                                const Image<ImageTrait>& rhs) {       \
    return name(lhs, rhs);                                            \
  }                                                                   \
                                                                      \
  template <class ImageTrait>                                         \
  Image<ImageTrait>& operator op## =                                  \
      (Image<ImageTrait> & lhs, const Image<ImageTrait>& rhs) {       \
    static_assert(details::OperatorTraits<ImageTrait>::kIsVector,     \
                  "name## must be defined");                          \
    CHECK(lhs.dimensions() == rhs.dimensions())                       \
        << "Input image shapes are not equal!";                       \
    ToCvMat(&lhs) op## = UnsafeConstCastCvMat(rhs);                   \
    return lhs;                                                       \
  }

IMAGE_OPERATOR(+, PixelwiseAdd);
IMAGE_OPERATOR(-, PixelwiseSubtract);
IMAGE_OPERATOR(&, PixelwiseAnd);
IMAGE_OPERATOR(|, PixelwiseOr);
#undef IMAGE_OPERATOR

// Macro for image comparison operations (==, <, >, ...).
// Comparisons are done between two images or an image an a value.
// Note: The comparisons are down per pixel and a boolean mask image is
//       returned, not a scalar.
#define IMAGE_COMPARISON(op, name)                                           \
  template <class ImageTrait>                                                \
  Image<Bool8u> name(const Image<ImageTrait>& lhs,                           \
                     const Image<ImageTrait>& rhs) {                         \
    static_assert(ImageTrait::kNumChannels == 1, "name##s must be defined"); \
    CHECK(lhs.dimensions() == rhs.dimensions())                              \
        << "Input image shapes are not equal!";                              \
    return MoveToImage<Bool8u>(UnsafeConstCastCvMat(lhs)                     \
                                   op UnsafeConstCastCvMat(rhs));            \
  }                                                                          \
                                                                             \
  template <class ImageTrait>                                                \
  Image<Bool8u> name(const Image<ImageTrait>& lhs,                           \
                     typename ImageTrait::PixelType pixel) {                 \
    static_assert(ImageTrait::kNumChannels == 1, "name##s must be defined"); \
    return MoveToImage<Bool8u>(UnsafeConstCastCvMat(lhs) op pixel);          \
  }                                                                          \
                                                                             \
  template <class ImageTrait>                                                \
  Image<Bool8u> name(typename ImageTrait::PixelType pixel,                   \
                     const Image<ImageTrait>& rhs) {                         \
    static_assert(ImageTrait::kNumChannels == 1, "name##s must be defined"); \
    return MoveToImage<Bool8u>(pixel op UnsafeConstCastCvMat(rhs));          \
  }                                                                          \
                                                                             \
  template <class ImageTrait>                                                \
  Image<Bool8u> operator op(const Image<ImageTrait>& lhs,                    \
                            const Image<ImageTrait>& rhs) {                  \
    return name(lhs, rhs);                                                   \
  }                                                                          \
                                                                             \
  template <class ImageTrait>                                                \
  Image<Bool8u> operator op(const Image<ImageTrait>& lhs,                    \
                            typename ImageTrait::PixelType pixel) {          \
    return name(lhs, pixel);                                                 \
  }                                                                          \
                                                                             \
  template <class ImageTrait>                                                \
  Image<Bool8u> operator op(typename ImageTrait::PixelType pixel,            \
                            const Image<ImageTrait>& rhs) {                  \
    return name(pixel, rhs);                                                 \
  }

IMAGE_COMPARISON(==, PixelwiseEqual);
IMAGE_COMPARISON(<, PixelwiseSmaller);
IMAGE_COMPARISON(>, PixelwiseGreater);
IMAGE_COMPARISON(<=, PixelwiseSmallerEqual);
IMAGE_COMPARISON(>=, PixelwiseGreaterEqual);
IMAGE_COMPARISON(!=, PixelwiseUnequal);

#undef IMAGE_COMPARISON

// Assign all pixels in image to value.
template <typename ImageTrait>
void Fill(Image<ImageTrait>* image, typename ImageTrait::PixelType value) {
  CHECK(image != nullptr) << "image is nullptr.";
  std::fill(image->begin(), image->end(), value);
}

// Assign pixels in image to value using a mask.
template <typename ImageTrait>
void FillIf(Image<ImageTrait>* image, const Image<Bool8u>& mask,
            typename ImageTrait::PixelType value) {
  CHECK(image != nullptr) << "image is nullptr.";
  CHECK(image->dimensions() == mask.dimensions())
      << "Image dimensions are unequal.";
  ToCvMat(image).setTo(PixelToCvScalar(value), UnsafeConstCastCvMat(mask));
}

// Set the pixel value for a region of interest.
template <typename ImageTrait>
void FillRoi(Image<ImageTrait>* image, const Rectangle& roi,
             typename ImageTrait::PixelType value) {
  CHECK(image != nullptr) << "image is nullptr.";
  CHECK(roi.TopLeft().col >= 0 && roi.TopLeft().row >= 0 &&
        roi.BottomRight().col < image->dimensions().cols &&
        roi.BottomRight().row < image->dimensions().rows)
      << "Roi is out of bounds.";
  ToCvMat(image)(RectangleToCvRect(roi)).setTo(PixelToCvScalar(value));
}

// Copy pixel in image to another image using a mask.
template <typename ImageTrait>
void CopyIf(const Image<ImageTrait>& lhs, Image<ImageTrait>* rhs,
            const Image<Bool8u>& mask) {
  CHECK(rhs != nullptr) << "rhs is nullptr.";
  CHECK(lhs.dimensions() == mask.dimensions())
      << "Image dimensions are unequal.";
  CHECK(rhs->dimensions() == mask.dimensions())
      << "Image dimensions are unequal.";
  UnsafeConstCastCvMat(lhs).copyTo(ToCvMat(rhs), UnsafeConstCastCvMat(mask));
}

// Copy the pixel value for a region of interest.
template <typename ImageTrait>
void CopyRoi(const Image<ImageTrait>& lhs, Image<ImageTrait>* rhs,
             const Rectangle& roi) {
  CHECK(rhs != nullptr) << "rhs is nullptr.";
  CHECK(roi.TopLeft().col >= 0 && roi.TopLeft().row >= 0 &&
        roi.BottomRight().col < lhs.dimensions().cols &&
        roi.BottomRight().row < lhs.dimensions().rows &&
        lhs.dimensions() == rhs->dimensions())
      << "Roi is out of bounds.";
  auto rect = RectangleToCvRect(roi);
  UnsafeConstCastCvMat(lhs)(rect).copyTo(ToCvMat(rhs)(rect));
}

// Copy the pixel values within a region of interest from lhs to rhs.
template <typename ImageTrait>
void CopyFromRoi(const Image<ImageTrait>& lhs, Image<ImageTrait>& rhs,
                 const Rectangle& roi) {
  CHECK(roi.TopLeft().col >= 0 && roi.TopLeft().row >= 0 &&
        roi.BottomRight().col < lhs.dimensions().cols &&
        roi.BottomRight().row < lhs.dimensions().rows &&
        roi.dimensions == rhs.dimensions())
      << "Roi is out of bounds.";
  auto rect = RectangleToCvRect(roi);
  UnsafeConstCastCvMat(lhs)(rect).copyTo(ToCvMat(rhs));
}

// Copy the pixel values from lhs to a region of interest within rhs.
template <typename ImageTrait>
void CopyToRoi(const Image<ImageTrait>& lhs, Image<ImageTrait>& rhs,
               const Rectangle& roi) {
  CHECK(roi.TopLeft().col >= 0 && roi.TopLeft().row >= 0 &&
        roi.BottomRight().col < rhs.dimensions().cols &&
        roi.BottomRight().row < rhs.dimensions().rows &&
        roi.dimensions == lhs.dimensions())
      << "Roi is out of bounds.";
  auto rect = RectangleToCvRect(roi);
  UnsafeConstCastCvMat(lhs).copyTo(ToCvMat(rhs)(rect));
}

// Assign all pixels in image to value.
template <typename ImageTrait>
void Fill(Image<ImageTrait>& image, typename ImageTrait::PixelType value) {
  std::fill(image.begin(), image.end(), value);
}

// Assign pixels in image to value using a mask.
template <typename ImageTrait>
void FillIf(Image<ImageTrait>& image, const Image<Bool8u>& mask,
            typename ImageTrait::PixelType value) {
  CHECK(image.dimensions() == mask.dimensions())
      << "Image dimensions are unequal.";
  ToCvMat(image).setTo(PixelToCvScalar(value), UnsafeConstCastCvMat(mask));
}

// Set the pixel value for a region of interest.
template <typename ImageTrait>
void FillRoi(Image<ImageTrait>& image, const Rectangle& roi,
             typename ImageTrait::PixelType value) {
  CHECK(roi.TopLeft().col >= 0 && roi.TopLeft().row >= 0 &&
        roi.BottomRight().col < image.dimensions().cols &&
        roi.BottomRight().row < image.dimensions().rows)
      << "Roi is out of bounds.";
  ToCvMat(image)(RectangleToCvRect(roi)).setTo(PixelToCvScalar(value));
}

// Gets a binary mask for a polygon.
inline Image<Bool8u> PolygonMask(const Dimensions& dimensions,
                                 const std::vector<Vector2f>& polygon) {
  CHECK(polygon.size() > 2) << "Polygon needs to contain at least 3 vertices.";
  std::vector<cv::Point> points(polygon.size());
  std::transform(polygon.begin(), polygon.end(), points.begin(),
                 [](Vector2f p) { return cv::Point(p.x(), p.y()); });
  cv::Mat image_mat(dimensions.rows, dimensions.cols, CV_8U, cv::Scalar(0));
  cv::fillConvexPoly(image_mat, points, cv::Scalar(1));
  return MoveToImage<Bool8u>(std::move(image_mat));
}

// Extracts and returns the specified sub-region of the input image.
template <typename ImageTrait>
absl::StatusOr<Image<ImageTrait>> ExtractRoi(const Image<ImageTrait>& lhs,
                                             const Rectangle& roi) {
  if (!Rectangle::IsValid(roi)) {
    return InvalidArgumentErrorBuilder() << "Roi is invalid: " << roi;
  }
  if (!(roi.TopLeft().col >= 0 && roi.TopLeft().row >= 0 &&
        roi.BottomRight().col < lhs.dimensions().cols &&
        roi.BottomRight().row < lhs.dimensions().rows)) {
    return absl::OutOfRangeError("Roi is out of bounds.");
  }
  Image<ImageTrait> roi_image(roi.dimensions);
  auto rect = RectangleToCvRect(roi);
  UnsafeConstCastCvMat(lhs)(rect).copyTo(ToCvMat(&roi_image));
  return roi_image;
}

// Extracts and returns the specified sub-region of the input image. Allows
// extracting rois that are partially (or fully) outside of the image, in which
// case, the pixels that correspond to the outside of the image are filled by
// the value of `fill_value`.
template <typename ImageTrait>
absl::StatusOr<Image<ImageTrait>> ExtractRoi(
    const Image<ImageTrait>& lhs, const Rectangle& roi,
    const typename ImageTrait::PixelType& fill_value) {
  if (!Rectangle::IsValid(roi)) {
    return InvalidArgumentErrorBuilder() << "Roi is invalid: " << roi;
  }
  Image<ImageTrait> roi_image(roi.dimensions, fill_value);
  const cv::Rect rect = RectangleToCvRect(roi);
  // `in_frame_rect` is the rectangle in `lhs` that contains all the data that
  // is needed for the resulting image.
  const cv::Rect in_frame_rect = rect & cv::Rect(0, 0, lhs.cols(), lhs.rows());
  if (in_frame_rect.empty()) {
    // Return the image filled with `fill_value`.
    return roi_image;
  }
  // `in_crop_frame_rect` is the rectangle in the resulting image that contains
  // all the data extracted from original image `lhs`. The values outside of
  // this rectangle is filled by `fill_value`.
  cv::Rect in_crop_frame_rect = in_frame_rect;
  in_crop_frame_rect.x = std::max(0, -rect.x);
  in_crop_frame_rect.y = std::max(0, -rect.y);
  UnsafeConstCastCvMat(lhs)(in_frame_rect)
      .copyTo(ToCvMat(&roi_image)(in_crop_frame_rect));
  return roi_image;
}

// Extracts and returns the specified sub-region of the input image. If optional
// argument `fill_value` is set, allows extracting rois that are partially (or
// fully) outside of the image, in which case, the pixels that correspond to the
// outside of the image are filled by the value of `fill_value`.
template <typename ImageTrait>
absl::StatusOr<Image<ImageTrait>> ExtractRoi(
    const Image<ImageTrait>& lhs, const Rectangle& roi,
    const std::optional<typename ImageTrait::PixelType>& fill_value) {
  if (fill_value.has_value()) {
    return ExtractRoi(lhs, roi, fill_value.value());
  }
  return ExtractRoi(lhs, roi);
}

// Copies the first image in its entirety to the specified position in the
// second image. The signature is based on the one of CopyRoi().
template <typename ImageTrait>
void CopyTo(const Image<ImageTrait>& copy_me, Image<ImageTrait>& image,
            const Coordinate& copy_to) {
  const Rectangle target_roi{copy_to, copy_me.dimensions()};

  CHECK(Rectangle::IsValid(target_roi)) << "Roi is invalid.";
  CHECK(Contains(Rectangle{image.dimensions()}, target_roi))
      << "Roi is out of bounds.";

  auto cv_target_roi = RectangleToCvRect(target_roi);
  UnsafeConstCastCvMat(copy_me).copyTo(ToCvMat(image)(cv_target_roi));
}

// Returns true if both images are identical (i.e. have the same dimensions and
// each pixel has the same value).
template <class ImageTrait>
bool ImageEqual(const Image<ImageTrait>& lhs, const Image<ImageTrait>& rhs) {
  if (lhs.dimensions() == rhs.dimensions()) {
    return std::equal(lhs.begin(), lhs.end(), rhs.begin());
  }
  return false;
}

// Returns true if the images not equal. Equivalent to !ImageEqual(lhs, rhs).
template <class ImageTrait>
bool ImageNotEqual(const Image<ImageTrait>& lhs, const Image<ImageTrait>& rhs) {
  return !(ImageEqual(lhs, rhs));
}

// Counts all the pixels that are unequal zero in an image mask.
inline size_t CountNonZero(const Image<Bool8u>& mask) {
  return cv::countNonZero(UnsafeConstCastCvMat(mask));
}

// Applies a function to an image, converts the image type and returns the
// transformed image.
template <typename OutputTraits, typename InputTraits, typename Function>
Image<OutputTraits> Transform(
    const Image<InputTraits>& source, Function function,
    typename std::enable_if<
        !std::is_same<OutputTraits, InputTraits>::value>::type* = nullptr) {
  Image<OutputTraits> target(source.dimensions());
  std::transform(source.begin(), source.end(), target.begin(),
                 std::move(function));
  return target;
}

// Applies a function to an image and returns the transformed image.
template <typename ImageTrait, typename Function>
Image<ImageTrait> Transform(const Image<ImageTrait>& source,
                            Function function) {
  Image<ImageTrait> target(source.dimensions());
  std::transform(source.begin(), source.end(), target.begin(),
                 std::move(function));
  return target;
}

template <typename ImageTrait>
absl::StatusOr<Image<ImageTrait>> CropAndResize(
    const Image<ImageTrait>& image, const PostProcessing& post_processing,
    int interpolation_method,
    std::optional<typename ImageTrait::PixelType> fill_value) {
  const std::optional<Rectangle>& crop_region = post_processing.crop_region;
  const std::optional<int32_t>& cols = post_processing.cols;
  const std::optional<int32_t>& rows = post_processing.rows;
  if (!crop_region.has_value() && !cols.has_value() && !rows.has_value()) {
    return image;
  }
  const Rectangle image_rect(image.dimensions());
  if (crop_region.has_value() && !fill_value.has_value() &&
      !Contains(image_rect, crop_region.value())) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "crop region outside of image: " << crop_region.value()
           << " not in " << image.dimensions();
  }
  if (!cols.has_value() && !rows.has_value()) {
    INTR_RET_CHECK(crop_region.has_value());
    return ExtractRoi(image, crop_region.value(), fill_value);
  }
  const Rectangle crop_rect = crop_region.value_or(image_rect);
  INTR_ASSIGN_OR_RETURN(const Dimensions resized_dimensions,
                        Resize(image.dimensions(), cols, rows));
  if (resized_dimensions.cols <= 0 || resized_dimensions.rows <= 0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Both rows and cols need to be positive after "
                        "resizing, but got (%d, %d)",
                        resized_dimensions.rows, resized_dimensions.cols));
  }
  Image<ImageTrait> resized(resized_dimensions);
  if (fill_value.has_value() && !Contains(image_rect, crop_rect)) {
    // We have to copy the crop to a temporary image, because of padding.
    INTR_ASSIGN_OR_RETURN(const Image<ImageTrait> crop,
                          ExtractRoi(image, crop_rect, fill_value));
    cv::resize(UnsafeConstCastCvMat(crop), ToCvMat(resized),
               DimensionsToCvSize(resized.dimensions()), 0, 0,
               interpolation_method);
    return resized;
  }
  const auto rect = RectangleToCvRect(crop_rect);
  cv::resize(UnsafeConstCastCvMat(image)(rect), ToCvMat(resized),
             DimensionsToCvSize(resized.dimensions()), 0, 0,
             interpolation_method);
  return resized;
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_OPERATORS_H_
