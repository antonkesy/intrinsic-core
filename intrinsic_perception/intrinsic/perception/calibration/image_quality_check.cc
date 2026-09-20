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

#include "intrinsic/perception/calibration/image_quality_check.h"

#include <algorithm>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/perception/core/argument_checks.h"
#include "intrinsic/perception/core/conversion.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/opencv_wrapper.h"
#include "intrinsic/util/status/status_macros.h"
#include "opencv2/core.hpp"
#include "opencv2/core/base.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/types.hpp"

namespace intrinsic {
namespace perception {
namespace {

cv::Mat ComputeDiscreteFourierTransform(const cv::Mat& image) {
  // The output of a DFT is complex, and will thus result in a two channel image
  // (real part in the first channel, imaginary in the second). Initialize this
  // complex image, with the real image in the first channel, and 0s in the
  // second channel.
  cv::Mat planes[] = {cv::Mat_<float>(image),
                      cv::Mat::zeros(image.size(), CV_32F)};
  cv::Mat dft;
  cv::merge(planes, 2, dft);
  cv::dft(dft, dft);
  return dft;
}

// Returns the magnitude of a complex OpenCv Mat.
//
// A Discrete Fourier Transform (DFT) is expressed as a complex image in OpenCV,
// with a real and imaginary part. An alternative is representing the image as a
// magnitude and phase part. This function computes the magnitude from a given
// complex image.
cv::Mat GetMagnitudeFromComplexMat(const cv::Mat& dft) {
  std::vector<cv::Mat> planes(2);
  // The dft contains real values in the first channel, and imaginary values in
  // the second channel. Split the matrix by its channels so we can access both
  // the real and imaginary part.
  cv::split(dft, planes);
  cv::Mat magnitude;
  // The magnitude is computed as sqrt(real**2 + imaginary**2).
  cv::magnitude(planes[0], planes[1], magnitude);
  return magnitude;
}

cv::Mat ComputeInverseFourierTransform(const cv::Mat& dft) {
  cv::Mat idft;
  cv::dft(dft, idft, cv::DFT_INVERSE + cv::DFT_SCALE);
  return idft;
}

}  // namespace

template <typename ImageTraits>
absl::StatusOr<double> ComputeSharpnessScore(
    const Image<ImageTraits>& image, int highpass_filter_size_in_pixel) {
  if (highpass_filter_size_in_pixel < 0) {
    return absl::OutOfRangeError(
        absl::StrFormat("highpass_filter_size_in_pixel must be greater than or "
                        "equal to 0, but is %d",
                        highpass_filter_size_in_pixel));
  }
  INTR_ASSIGN_OR_RETURN(cv::Mat dft, [&image]() -> absl::StatusOr<cv::Mat> {
    if constexpr (std::is_same_v<Rgb8u, ImageTraits>) {
      const Image<Gray8u> gray_image = ConvertRgb8uToGray8u(image);
      return ComputeDiscreteFourierTransform(UnsafeConstCastCvMat(gray_image));
    } else if constexpr (std::is_same_v<Gray8u, ImageTraits>) {
      return ComputeDiscreteFourierTransform(UnsafeConstCastCvMat(image));
    } else if constexpr (std::is_same_v<Gray32f, ImageTraits>) {
      const Image<Gray8u> gray_image = ConvertGray32fToGray8uImage(image);
      return ComputeDiscreteFourierTransform(UnsafeConstCastCvMat(gray_image));
    } else {
      return absl::InvalidArgumentError("Unsupported image type.");
    }
  }());

  // Remove low frequencies.
  // TODO(marekmartina): Consider using a circular highpass filter.
  cv::Scalar value(0, 0);
  int size = highpass_filter_size_in_pixel;
  dft(cv::Rect(0, 0, size, size)) = value;
  dft(cv::Rect(dft.cols - size, 0, size, size)) = value;
  dft(cv::Rect(0, dft.rows - size, size, size)) = value;
  dft(cv::Rect(dft.cols - size, dft.rows - size, size, size)) = value;

  cv::Mat idft = ComputeInverseFourierTransform(dft);
  cv::Mat idft_magnitude = GetMagnitudeFromComplexMat(idft);
  idft_magnitude += cv::Scalar::all(1);
  cv::log(idft_magnitude, idft_magnitude);
  cv::Scalar mean = cv::mean(idft_magnitude);
  LOG(INFO) << "sharpness: " << mean[0];
  return mean[0];
}

// Returns whether an image's bluriness exceeds a given threshold.
template <typename ImageTraits>
absl::StatusOr<bool> IsBlurry(const Image<ImageTraits>& image, double threshold,
                              int highpass_filter_size_in_pixel) {
  if (threshold < 0) {
    return absl::OutOfRangeError(
        absl::StrFormat("Blur detection threshold must be greater than or "
                        "equal to 0, but is %.2f",
                        threshold));
  }
  INTR_ASSIGN_OR_RETURN(
      const double sharpness,
      ComputeSharpnessScore(image, highpass_filter_size_in_pixel));
  return IsBlurry(sharpness, threshold);
}
template absl::StatusOr<double> ComputeSharpnessScore(
    const Image<Rgb8u>& image, int highpass_filter_size_in_pixel);
template absl::StatusOr<double> ComputeSharpnessScore(
    const Image<Gray8u>& image, int highpass_filter_size_in_pixel);
template absl::StatusOr<double> ComputeSharpnessScore(
    const Image<Gray32f>& image, int highpass_filter_size_in_pixel);

template absl::StatusOr<bool> IsBlurry(const Image<Rgb8u>& image,
                                       double threshold,
                                       int highpass_filter_size_in_pixel);
template absl::StatusOr<bool> IsBlurry(const Image<Gray8u>& image,
                                       double threshold,
                                       int highpass_filter_size_in_pixel);
template absl::StatusOr<bool> IsBlurry(const Image<Gray32f>& image,
                                       double threshold,
                                       int highpass_filter_size_in_pixel);

absl::StatusOr<bool> IsOverexposed(const Image<Rgb8u>& image,
                                   double threshold) {
  INTR_RETURN_IF_ERROR(InClosedInterval(threshold, 0.0, 1.0));
  int overexposed_pixels =
      std::count_if(image.begin(), image.end(), [](const Rgb8u::PixelType& p) {
        return p == Rgb8u::PixelType(255, 255, 255);
      });
  // OpenCV make a 1 pixel area if the polygon consists of only the same points,
  // so we don't have to check for 0 area before dividing.
  return (static_cast<double>(overexposed_pixels) / image.area()) > threshold;
}

}  // namespace perception
}  // namespace intrinsic
