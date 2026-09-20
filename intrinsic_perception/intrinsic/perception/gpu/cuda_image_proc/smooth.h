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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_SMOOTH_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_SMOOTH_H_

#include "absl/status/statusor.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"

namespace intrinsic {
namespace perception {

// Each pixel value in the output image is computed as the weighted sum of
// pixels in the neighborhood of a pixel in the input image, using the separable
// gaussian filter with coefficients [1.0, 4.0, 6.0, 4.0, 1.0]. Out-of-bound
// pixels will be extrapolated based on the `border_type` setting. The weights
// in the 2D filter normally sum to 1.0f but after the summation they are
// further multiplied by sum_weight. This can be useful for example to compute
// properly scaled pixel values after smoothing zero-pixels following a 2x2
// upsampling operation and in which case the sum_weight should be set to 4.0.
template <typename ImageTraits>
absl::StatusOr<CudaImage<ImageTraits>> SmoothImageWithGaussian5x5(
    const CudaImage<ImageTraits>& image,
    BorderType border_type = BorderType::kBorderConstantZero,
    float sum_weight = 1.0f);

extern template absl::StatusOr<CudaImage<Gray8u>>
SmoothImageWithGaussian5x5<Gray8u>(const CudaImage<Gray8u>& image,
                                   BorderType border_type, float sum_weight);

extern template absl::StatusOr<CudaImage<Rgb8u>>
SmoothImageWithGaussian5x5<Rgb8u>(const CudaImage<Rgb8u>& image,
                                  BorderType border_type, float sum_weight);

extern template absl::StatusOr<CudaImage<Gray32f>>
SmoothImageWithGaussian5x5<Gray32f>(const CudaImage<Gray32f>& image,
                                    BorderType border_type, float sum_weight);

extern template absl::StatusOr<CudaImage<Generic32f>>
SmoothImageWithGaussian5x5<Generic32f>(const CudaImage<Generic32f>& image,
                                       BorderType border_type,
                                       float sum_weight);

extern template absl::StatusOr<CudaImage<Generic32f3>>
SmoothImageWithGaussian5x5<Generic32f3>(const CudaImage<Generic32f3>& image,
                                        BorderType border_type,
                                        float sum_weight);

extern template absl::StatusOr<CudaImage<Generic16i>>
SmoothImageWithGaussian5x5<Generic16i>(const CudaImage<Generic16i>& image,
                                       BorderType border_type,
                                       float sum_weight);

extern template absl::StatusOr<CudaImage<Generic16i3>>
SmoothImageWithGaussian5x5<Generic16i3>(const CudaImage<Generic16i3>& image,
                                        BorderType border_type,
                                        float sum_weight);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_SMOOTH_H_
