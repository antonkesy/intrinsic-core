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

#include "intrinsic/perception/core/opencv_wrapper.h"

#include <cstddef>
#include <cstdint>
#include <exception>

#include "absl/cleanup/cleanup.h"
#include "absl/functional/function_ref.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/distortion_params.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/intrinsic_params.h"
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/core/utility.hpp"

namespace intrinsic {
namespace perception {
namespace {

// This is a null allocator which asserts that only no-op allocations occurs.
// Thus, only those calls to cv::Mat::create are allowed which do not change
// image size or image type.
struct NullAllocator final : cv::MatAllocator {
  cv::UMatData* allocate(int dims, const int* sizes, int type, void* data,
                         size_t* step, cv::AccessFlag flags,
                         cv::UMatUsageFlags usageFlags) const override {
    // TODO(b/454602387): Switch to INTR_CHECK.
    CHECK(dims == 2) << "User-defined cv::Mat's are always 2-dimensional";
    CHECK(false)
        << "It is not permitted to call 'allocate' on a user-defined "
           "cv::Mat, since cv::Mat's aliases, and the aliased "
           "cv::Mat would go stale if the user-defined memory is "
           "reallocated. It is okay to call 'create' only if the same size "
           "and type is used as the underlying data; no reallocation "
           "will happen in this case.";
    return nullptr;
  }

  bool allocate(cv::UMatData* data, cv::AccessFlag accessflags,
                cv::UMatUsageFlags usageFlags) const override {
    CHECK(false)
        << "This is user-allocated data; nobody should call deallocate here!";
    return false;
  }

  void deallocate(cv::UMatData* data) const override {
    CHECK(false)
        << "This is user-allocated data; nobody should call deallocate here!";
  }

  static NullAllocator& instance() {
    static auto* s_instance = new NullAllocator();
    return *s_instance;
  }
};

}  // namespace

template <class ImageTrait>
cv::Mat ToCvMat(Image<ImageTrait>* image) {
  CHECK(image != nullptr) << "image is nullptr.";
  cv::Mat m(image->rows(), image->cols(),
            details::OpenCvTraits<ImageTrait>::kCvMatType,
            reinterpret_cast<uint8_t*>(image->data()));
  // This is data owned by image, assert when OpenCV_ attempts to reallocate.
  m.allocator = &NullAllocator::instance();
  return m;
}

template <class ImageTrait>
// NOLINTNEXTLINE(readability-const-return-type)
const cv::Mat UnsafeConstCastCvMat(const Image<ImageTrait>& image) {
  cv::Mat m(image.rows(), image.cols(),
            details::OpenCvTraits<ImageTrait>::kCvMatType,
            // This const_cast is unsafe because cv::Mat aliases. Be careful!
            reinterpret_cast<uint8_t*>(
                const_cast<typename ImageTrait::PixelType*>(image.data())));
  // This is data owned by image, assert when OpenCV_ attempts to reallocate.
  m.allocator = &NullAllocator::instance();
  return m;
}

// Currently-supported Image -> OpenCV_ conversions.
template cv::Mat ToCvMat<Rgb8u>(Image<Rgb8u>* image);
template const cv::Mat UnsafeConstCastCvMat<Rgb8u>(const Image<Rgb8u>& image);

template cv::Mat ToCvMat<Rgba8u>(Image<Rgba8u>* image);
template const cv::Mat UnsafeConstCastCvMat<Rgba8u>(const Image<Rgba8u>& image);

template cv::Mat ToCvMat<Bgr8u>(Image<Bgr8u>* image);
template const cv::Mat UnsafeConstCastCvMat<Bgr8u>(const Image<Bgr8u>& image);

template cv::Mat ToCvMat<Bgra8u>(Image<Bgra8u>* image);
template const cv::Mat UnsafeConstCastCvMat<Bgra8u>(const Image<Bgra8u>& image);

template cv::Mat ToCvMat<Rgb32f>(Image<Rgb32f>* image);
template const cv::Mat UnsafeConstCastCvMat<Rgb32f>(const Image<Rgb32f>& image);

template cv::Mat ToCvMat<Bgr32f>(Image<Bgr32f>* image);
template const cv::Mat UnsafeConstCastCvMat<Bgr32f>(const Image<Bgr32f>& image);

template cv::Mat ToCvMat<Gray8u>(Image<Gray8u>* image);
template const cv::Mat UnsafeConstCastCvMat<Gray8u>(const Image<Gray8u>& image);

template cv::Mat ToCvMat<Gray32f>(Image<Gray32f>* image);
template const cv::Mat UnsafeConstCastCvMat<Gray32f>(
    const Image<Gray32f>& image);

template cv::Mat ToCvMat<Bool8u>(Image<Bool8u>* image);
template const cv::Mat UnsafeConstCastCvMat<Bool8u>(const Image<Bool8u>& image);

template cv::Mat ToCvMat<Normal32f>(Image<Normal32f>* image);
template const cv::Mat UnsafeConstCastCvMat<Normal32f>(
    const Image<Normal32f>& image);

template cv::Mat ToCvMat<Gradient32f>(Image<Gradient32f>* image);
template const cv::Mat UnsafeConstCastCvMat<Gradient32f>(
    const Image<Gradient32f>& image);

template cv::Mat ToCvMat<Depth16u>(Image<Depth16u>* image);
template const cv::Mat UnsafeConstCastCvMat<Depth16u>(
    const Image<Depth16u>& image);

template cv::Mat ToCvMat<Depth32f>(Image<Depth32f>* image);
template const cv::Mat UnsafeConstCastCvMat<Depth32f>(
    const Image<Depth32f>& image);

template cv::Mat ToCvMat<Point32f>(Image<Point32f>* image);
template const cv::Mat UnsafeConstCastCvMat<Point32f>(
    const Image<Point32f>& image);

template cv::Mat ToCvMat<Generic8u>(Image<Generic8u>* image);
template const cv::Mat UnsafeConstCastCvMat<Generic8u>(
    const Image<Generic8u>& image);

template cv::Mat ToCvMat<Generic16u>(Image<Generic16u>* image);
template const cv::Mat UnsafeConstCastCvMat<Generic16u>(
    const Image<Generic16u>& image);

template cv::Mat ToCvMat<Generic8i>(Image<Generic8i>* image);
template const cv::Mat UnsafeConstCastCvMat<Generic8i>(
    const Image<Generic8i>& image);

template cv::Mat ToCvMat<Generic16i>(Image<Generic16i>* image);
template const cv::Mat UnsafeConstCastCvMat<Generic16i>(
    const Image<Generic16i>& image);

template cv::Mat ToCvMat<Generic16i2>(Image<Generic16i2>* image);
template const cv::Mat UnsafeConstCastCvMat<Generic16i2>(
    const Image<Generic16i2>& image);

template cv::Mat ToCvMat<Generic32i>(Image<Generic32i>* image);
template const cv::Mat UnsafeConstCastCvMat<Generic32i>(
    const Image<Generic32i>& image);

template cv::Mat ToCvMat<Generic32f>(Image<Generic32f>* image);
template const cv::Mat UnsafeConstCastCvMat<Generic32f>(
    const Image<Generic32f>& image);

template cv::Mat ToCvMat<Generic32f2>(Image<Generic32f2>* image);
template const cv::Mat UnsafeConstCastCvMat<Generic32f2>(
    const Image<Generic32f2>& image);

template cv::Mat ToCvMat<Generic32f3>(Image<Generic32f3>* image);
template const cv::Mat UnsafeConstCastCvMat<Generic32f3>(
    const Image<Generic32f3>& image);

template cv::Mat ToCvMat<Orientation32f>(Image<Orientation32f>* image);
template const cv::Mat UnsafeConstCastCvMat<Orientation32f>(
    const Image<Orientation32f>& image);

template cv::Mat ToCvMat<Label32i>(Image<Label32i>* image);
template const cv::Mat UnsafeConstCastCvMat<Label32i>(
    const Image<Label32i>& image);

cv::Mat ToCvIntrinsicParams(const IntrinsicParams& intrinsic_params) {
  cv::Mat intrinsics(cv::Size(3, 3), CV_64F, cv::Scalar(0));
  intrinsics.at<double>(0, 0) = intrinsic_params.focal_length_x();
  intrinsics.at<double>(1, 1) = intrinsic_params.focal_length_y();
  intrinsics.at<double>(0, 2) = intrinsic_params.principal_point_x();
  intrinsics.at<double>(1, 2) = intrinsic_params.principal_point_y();
  intrinsics.at<double>(2, 2) = 1;
  return intrinsics;
}

cv::Mat ToCvDistortionParams(const DistortionParams& distortion_params) {
  int size = 4;
  if (distortion_params.tx != 0.0 || distortion_params.ty != 0.0) {
    size = 14;
  } else if (distortion_params.s1 != 0.0 || distortion_params.s2 != 0.0 ||
             distortion_params.s3 != 0.0 || distortion_params.s4 != 0.0) {
    size = 12;
  } else if (distortion_params.k4 != 0.0 || distortion_params.k5 != 0.0 ||
             distortion_params.k6 != 0.0) {
    size = 8;
  } else if (distortion_params.k3 != 0.0) {
    size = 5;
  }

  cv::Mat distortion(size, 1, CV_64F, cv::Scalar(0));
  distortion.at<double>(0) = distortion_params.k1;
  distortion.at<double>(1) = distortion_params.k2;
  distortion.at<double>(2) = distortion_params.p1;
  distortion.at<double>(3) = distortion_params.p2;

  if (size >= 5) {
    distortion.at<double>(4) = distortion_params.k3;
  }
  if (size >= 8) {
    distortion.at<double>(5) = distortion_params.k4;
    distortion.at<double>(6) = distortion_params.k5;
    distortion.at<double>(7) = distortion_params.k6;
  }
  if (size >= 12) {
    distortion.at<double>(8) = distortion_params.s1;
    distortion.at<double>(9) = distortion_params.s2;
    distortion.at<double>(10) = distortion_params.s3;
    distortion.at<double>(11) = distortion_params.s4;
  }
  if (size >= 14) {
    distortion.at<double>(12) = distortion_params.tx;
    distortion.at<double>(13) = distortion_params.ty;
  }
  return distortion;
}

IntrinsicParams FromCvIntrinsicParams(const Dimensions& dimensions,
                                      const cv::Mat& mat) {
  IntrinsicParams intrinsic_params(dimensions, mat.at<double>(0, 0),
                                   mat.at<double>(1, 1), mat.at<double>(0, 2),
                                   mat.at<double>(1, 2));
  return intrinsic_params;
}

DistortionParams FromCvDistortionParams(const cv::Mat& mat) {
  // Assume that the distortion matrix has the following form:
  // [k1, k2, p1, p2, [k3, [k4, k5, k6, [s1, s2, s3, s4, [tx, ty]]]]]
  CHECK_GE(mat.total(), 4)
      << "Distortion matrix must contain at least 4 elements.";
  DistortionParams distortion_params;
  distortion_params.k1 = mat.at<double>(0);
  distortion_params.k2 = mat.at<double>(1);
  distortion_params.p1 = mat.at<double>(2);
  distortion_params.p2 = mat.at<double>(3);
  if (mat.total() >= 5) {
    distortion_params.k3 = mat.at<double>(4);
  }
  if (mat.total() >= 8) {
    distortion_params.k4 = mat.at<double>(5);
    distortion_params.k5 = mat.at<double>(6);
    distortion_params.k6 = mat.at<double>(7);
  }
  if (mat.total() >= 12) {
    distortion_params.s1 = mat.at<double>(8);
    distortion_params.s2 = mat.at<double>(9);
    distortion_params.s3 = mat.at<double>(10);
    distortion_params.s4 = mat.at<double>(11);
  }
  if (mat.total() >= 14) {
    distortion_params.tx = mat.at<double>(12);
    distortion_params.ty = mat.at<double>(13);
  }
  return distortion_params;
}

absl::StatusOr<Pose3d> FromCVPose(const cv::Mat& rotation,
                                  const cv::Mat& translation) {
  if (!((translation.cols == 1 || translation.rows == 1) &&
        (translation.cols == 3 || translation.rows == 3))) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Invalid translation vector size. Size should be [1,"
                        "3] or [3, 1], but is [%d, %d]",
                        translation.cols, translation.rows));
  }
  if (rotation.cols != 3 || rotation.rows != 3) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Invalid rotation matrix size. Size should be [3, "
                        "3], but is [%d, %d]",
                        rotation.cols, rotation.rows));
  }
  const Matrix3d rotation_matrix =
      (Matrix3d() << rotation.at<double>(0, 0), rotation.at<double>(0, 1),
       rotation.at<double>(0, 2), rotation.at<double>(1, 0),
       rotation.at<double>(1, 1), rotation.at<double>(1, 2),
       rotation.at<double>(2, 0), rotation.at<double>(2, 1),
       rotation.at<double>(2, 2))
          .finished();

  const Vector3d translation_vector(translation.at<double>(0),
                                    translation.at<double>(1),
                                    translation.at<double>(2));
  return Pose3d(rotation_matrix, translation_vector);
}

absl::Status TryCatchCvFunction(absl::FunctionRef<void()> f,
                                absl::StatusCode code) {
  // When the break-on-error mode is set, the default error handler issues a
  // hardware exception. Those can't be handled in our stack so we turn them
  // off.
  const bool prev_break_on_error = cv::setBreakOnError(false);
  const absl::Cleanup on_return = [prev_break_on_error] {
    cv::setBreakOnError(prev_break_on_error);
  };
  try {
    f();
  } catch (const cv::Exception& e) {
    return absl::Status(code, absl::StrCat("Caught cv::Exception: ", e.what()));
  } catch (const std::exception& e) {
    return absl::Status(code,
                        absl::StrCat("Caught std::exception: ", e.what()));
  } catch (...) {
    return absl::Status(
        code, "Caught non-cv::Exception during execution of cv method.");
  }
  return absl::OkStatus();
}

}  // namespace perception
}  // namespace intrinsic
