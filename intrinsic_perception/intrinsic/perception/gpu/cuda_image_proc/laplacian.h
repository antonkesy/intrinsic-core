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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_LAPLACIAN_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_LAPLACIAN_H_

#include "absl/status/statusor.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"

namespace intrinsic {
namespace perception {

// Each pixel in the output image is computed as the weighted sum of pixels in
// the neighborhood of a pixel in the input image using the 3x3 filter
// [0, 1, 0; 1, -4, 1; 0, 1, 0]. Out-of-bound pixels will be extrapolated based
// on the `border_type` setting.
template <typename ImageTraits>
absl::StatusOr<CudaImage<GenericTrait<float, ImageTraits::kNumChannels>>>
ApplyLaplacian3x3(const CudaImage<ImageTraits>& image,
                  BorderType border_type = BorderType::kBorderConstantZero);

extern template absl::StatusOr<CudaImage<Generic32f>> ApplyLaplacian3x3<Gray8u>(
    const CudaImage<Gray8u>& image, BorderType border_type);

extern template absl::StatusOr<CudaImage<Generic32f3>> ApplyLaplacian3x3<Rgb8u>(
    const CudaImage<Rgb8u>& image, BorderType border_type);

extern template absl::StatusOr<CudaImage<Generic32f>>
ApplyLaplacian3x3<Gray32f>(const CudaImage<Gray32f>& image,
                           BorderType border_type);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_LAPLACIAN_H_
