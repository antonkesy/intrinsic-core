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

#ifndef INTRINSIC_PERCEPTION_CORE_OPENCV_WRAPPER_H_
#define INTRINSIC_PERCEPTION_CORE_OPENCV_WRAPPER_H_

#include <type_traits>
#include <vector>

#include "absl/functional/function_ref.h"
#include "absl/log/absl_check.h"
#include "absl/status/status.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/distortion_params.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/intrinsic_params.h"
#include "intrinsic/perception/core/rectangle.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/cvstd.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/traits.hpp"
#include "opencv2/core/types.hpp"

namespace intrinsic {
namespace perception {

namespace details {

template <class ImageTrait>
struct OpenCvTraits {
  using OpenCvDataType = cv::DataType<typename ImageTrait::ScalarType>;
  static_assert(OpenCvDataType::depth != -1,
                "The provided datatype is not supported by OpenCV.");
  static constexpr int kCvMatType =
      CV_MAKETYPE(OpenCvDataType::depth, ImageTrait::kNumChannels);
};

}  // namespace details

// Creates a mutable cv::Mat view of an image.
template <class ImageTrait>
cv::Mat ToCvMat(Image<ImageTrait>* image);

template <class ImageTrait>
cv::Mat ToCvMat(Image<ImageTrait>& image) {
  return ToCvMat(&image);
}

// Creates an unsafe "const" cv::Mat view of an image
//
// This function is unsafe, since it performs a const_cast and it is the user's
// responsibility to not modify the image data. Don't do the following:
//
//    void EvilFunction(const Image<T>& image) {
//      cv::Mat alias = UnsafeConstCastCvMat(image);
//      alias.at<float>(0,0) = 1; // DON'T DO THAT! This may lead to undefined
//                                // behavior, since it tries to modifies
//                                // potential const or volatile memory.
//    }
//
// See:
// http://stackoverflow.com/questions/13713625/is-cvmat-class-flawed-by-design
//
// Instead, be careful that the cv::Mat alias is only used in a const context.
// It's best practice to pass it directly to an OpenCV_ function:
//
//    void MyFunction(const Image<T>& image_view) {
//        double sum_of_pixels = cv::sum(UnsafeConstCastCvMat(image_view));
//        ...
//    }
template <class ImageTrait>
const cv::Mat UnsafeConstCastCvMat(const Image<ImageTrait>& image);

// We must never create views to temporary objects.
template <class ImageTrait>
// The const return type is intended here to signal the user that the returned
// matrix must not be modified.
// NOLINTNEXTLINE(readability-const-return-type)
const cv::Mat UnsafeConstCastCvMat(Image<ImageTrait>&& image) = delete;

// Move cv::Mat into an image.
//
// Recommended use:
//
//      Image<T> MyFunction(const InputParameter& input) {
//        return MoveToImage<T>(cv::SomeFunctionReturningCvMat(input));
//      }
//
//  If you moving out of an l-value cv::Mat, you have to call std::move.
//
//      Image<T> MyFunction(const InputParameter& input) {
//        cv::Mat mat = cv::SomeFunctionReturningCvMat(input);
//        return MoveToImage<T>(std::move(mat));
//      }
//
// Note: OpenCV_ does not track channel ordering, so the choice of
// MoveToImage<Bgr8u> or MoveToImage<Rgb8u> must be made carefully.
template <class ImageTrait>
Image<ImageTrait> MoveToImage(cv::Mat&& cv_image) {
  ABSL_CHECK(details::OpenCvTraits<ImageTrait>::kCvMatType == cv_image.type())
      << "Pixel types must match!";
  if (cv_image.empty()) {
    return Image<ImageTrait>();
  }
  ABSL_CHECK(cv_image.dims == 2) << "Must be an image!";
  ABSL_CHECK(cv_image.isContinuous())
      << "One can only move out of a continuous cv::Mat!";
  ABSL_CHECK(cv_image.u->refcount == 1)
      << "Refcount must be one; no aliases should be around!";
  ABSL_CHECK(cv_image.allocator == nullptr ||
             cv_image.allocator == cv::Mat::getDefaultAllocator())
      << "The cv::Mat must not have a custom allocator.";

  // moves ownership to image: cv::Mat was created with cv::fastMalloc.
  // Thus, we use cv::fastFree as custom deleter.
  Image<ImageTrait> image(
      {cv_image.cols, cv_image.rows},
      reinterpret_cast<typename ImageTrait::PixelType*>(cv_image.data),
      cv::fastFree);  // passed as function pointer

  // reset cv_image
  cv_image.dims = 0;
  cv_image.rows = 0;
  cv_image.cols = 0;
  cv_image.data = nullptr;
  cv_image.datastart = nullptr;
  cv_image.dataend = nullptr;
  cv_image.datalimit = nullptr;
  cv_image.u->refcount = 0;
  cv_image.u->data = nullptr;
  cv_image.u->origdata = nullptr;
  cv_image.deallocate();
  return image;
}

// Returns the cv::Scalar value for a corresponding image trait.
template <class PixelType>
inline cv::Scalar PixelToCvScalar(const PixelType& value) {
  return cv::Scalar::all(value);
}

template <>
inline cv::Scalar PixelToCvScalar<Vector2b>(const Vector2b& value) {
  return cv::Scalar(value.x(), value.y());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector2u>(const Vector2u& value) {
  return cv::Scalar(value.x(), value.y());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector2i>(const Vector2i& value) {
  return cv::Scalar(value.x(), value.y());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector2f>(const Vector2f& value) {
  return cv::Scalar(value.x(), value.y());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector2d>(const Vector2d& value) {
  return cv::Scalar(value.x(), value.y());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector3b>(const Vector3b& value) {
  return cv::Scalar(value.x(), value.y(), value.z());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector3u>(const Vector3u& value) {
  return cv::Scalar(value.x(), value.y(), value.z());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector3i>(const Vector3i& value) {
  return cv::Scalar(value.x(), value.y(), value.z());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector3f>(const Vector3f& value) {
  return cv::Scalar(value.x(), value.y(), value.z());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector3d>(const Vector3d& value) {
  return cv::Scalar(value.x(), value.y(), value.z());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector4b>(const Vector4b& value) {
  return cv::Scalar(value.x(), value.y(), value.z(), value.w());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector4u>(const Vector4u& value) {
  return cv::Scalar(value.x(), value.y(), value.z(), value.w());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector4i>(const Vector4i& value) {
  return cv::Scalar(value.x(), value.y(), value.z(), value.w());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector4f>(const Vector4f& value) {
  return cv::Scalar(value.x(), value.y(), value.z(), value.w());
}

template <>
inline cv::Scalar PixelToCvScalar<Vector4d>(const Vector4d& value) {
  return cv::Scalar(value.x(), value.y(), value.z(), value.w());
}

// Converts a vector to an OpenCV Point.
template <typename T>
inline cv::Point2f ToCvPoint(const T& point) {
  return cv::Point2f(point.x(), point.y());
}

template <>
inline cv::Point2f ToCvPoint<Vector2f>(const Vector2f& point) {
  return cv::Point2f(point.x(), point.y());
}

// Converts dimensions type to cv::Size.
inline cv::Size DimensionsToCvSize(const Dimensions& dimensions) {
  return cv::Size(dimensions.cols, dimensions.rows);
}

// Converts the rectangle type to a cv::Rect.
inline cv::Rect RectangleToCvRect(const Rectangle& rect) {
  return cv::Rect(rect.origin.col, rect.origin.row, rect.dimensions.cols,
                  rect.dimensions.rows);
}

// Converts the input array of 2d points to a vector of OpenCv 2d points.
template <typename Container2d>
std::vector<cv::Point2f> ToCvPoints2f(const Container2d& pts) {
  using Point = typename std::remove_const<
      typename std::remove_reference<decltype(*std::begin(pts))>::type>::type;
  static_assert(Point::RowsAtCompileTime == 2,
                "The input container must contain 2D points.");

  std::vector<cv::Point2f> cv_pts;
  cv_pts.reserve(pts.size());
  for (const auto& pt : pts) {
    cv_pts.emplace_back(pt.x(), pt.y());
  }
  return cv_pts;
}

// Converts the input array of 3d points to a vector of OpenCv 3d points.
template <typename Container3d>
std::vector<cv::Point3f> ToCvPoints3f(const Container3d& pts) {
  using Point = typename std::remove_const<
      typename std::remove_reference<decltype(*std::begin(pts))>::type>::type;
  static_assert(Point::RowsAtCompileTime == 3,
                "The input container must contain 3D points.");

  std::vector<cv::Point3f> cv_pts;
  cv_pts.reserve(pts.size());
  for (const auto& pt : pts) {
    cv_pts.emplace_back(pt.x(), pt.y(), pt.z());
  }
  return cv_pts;
}

// Converts our internal intrinsic parameters to a cv::Mat.
cv::Mat ToCvIntrinsicParams(const IntrinsicParams& intrinsic_params);

// Converts our internal distortion parameters to a cv::Mat.
cv::Mat ToCvDistortionParams(const DistortionParams& distortion_params);

// Converts intrinsic params from an cv::Mat to our internal intrinsic
// parameters.
IntrinsicParams FromCvIntrinsicParams(const Dimensions& dimensions,
                                      const cv::Mat& mat);

// Converts distortion params from an cv::Mat to our internal distortion
// parameters.
DistortionParams FromCvDistortionParams(const cv::Mat& mat);

// Converts a rotation and translation cv::Mat to our internal pose format.
absl::StatusOr<Pose3d> FromCVPose(const cv::Mat& rotation,
                                  const cv::Mat& translation);

// Surrounds the given function with a try-catch block that converts all
// exceptions to absl::Status. Also internally sets the OpenCV break-on-error
// flag to false, which is required to suppress hardware exceptions.
absl::Status TryCatchCvFunction(
    absl::FunctionRef<void()> f,
    absl::StatusCode code = absl::StatusCode::kInternal);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_OPENCV_WRAPPER_H_
