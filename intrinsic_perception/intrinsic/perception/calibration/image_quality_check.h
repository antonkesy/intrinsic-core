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

#ifndef PERCEPTION_CALIBRATION_IMAGE_QUALITY_CHECK_H_
#define PERCEPTION_CALIBRATION_IMAGE_QUALITY_CHECK_H_

#include "absl/status/statusor.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"

namespace intrinsic {
namespace perception {

// Returns whether the blurriness of an image exceeds a given threshold.
//
// Decreasing the threshold will allow blurrier images pass the condition.
// Decreasing the filter size has the same effect.
//
// This implementation is based on this code:
// https://github.com/whdcumt/BlurDetection. For a more detailed explanation
// see:
// https://www.pyimagesearch.com/2020/06/15/opencv-fast-fourier-transform-fft-for-blur-detection-in-images-and-video-streams/
//
// Internally, this function applies a FFT to the image, then removes low
// frequencies (=textureless or blurry parts of the image).  When transformed
// back to the image domain, basically an edge image remains (the high
// frequencies of the sharp parts of the image). These remaining edges are
// compared against a threshold, if there is too few edges, the image is deemed
// blurry.
template <typename ImageTraits>
absl::StatusOr<bool> IsBlurry(const Image<ImageTraits>& image,
                              double threshold = 1.0,
                              int highpass_filter_size_in_pixel = 60);

// Returns whether the sharpness score indicates blurriness.
inline bool IsBlurry(double sharpness_score, double threshold = 1.0) {
  return sharpness_score < threshold;
}
extern template absl::StatusOr<bool> IsBlurry(
    const Image<Rgb8u>& image, double threshold,
    int highpass_filter_size_in_pixel);
extern template absl::StatusOr<bool> IsBlurry(
    const Image<Gray8u>& image, double threshold,
    int highpass_filter_size_in_pixel);
extern template absl::StatusOr<bool> IsBlurry(
    const Image<Gray32f>& image, double threshold,
    int highpass_filter_size_in_pixel);

// Returns the sharpness score of an image.
// Higher value means sharper image.
template <typename ImageTraits>
absl::StatusOr<double> ComputeSharpnessScore(
    const Image<ImageTraits>& image, int highpass_filter_size_in_pixel = 60);
extern template absl::StatusOr<double> ComputeSharpnessScore(
    const Image<Rgb8u>& image, int highpass_filter_size_in_pixel);
extern template absl::StatusOr<double> ComputeSharpnessScore(
    const Image<Gray8u>& image, int highpass_filter_size_in_pixel);
extern template absl::StatusOr<double> ComputeSharpnessScore(
    const Image<Gray32f>& image, int highpass_filter_size_in_pixel);

// Returns whether the image is overexposed.
//
// An image is deemed overexposed when the ratio of pixels with the maximum
// value of 255 exceeds the threshold.
absl::StatusOr<bool> IsOverexposed(const Image<Rgb8u>& image,
                                   double threshold = 0.15);

}  // namespace perception
}  // namespace intrinsic

#endif  // PERCEPTION_CALIBRATION_IMAGE_QUALITY_CHECK_H_
