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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_COLOR_CONVERSION_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_COLOR_CONVERSION_H_

#include "absl/status/statusor.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"

namespace intrinsic {
namespace perception {

template <typename OutputImageTraits, typename InputImageTraits>
absl::StatusOr<CudaImage<OutputImageTraits>> RgbToGray(
    const CudaImage<InputImageTraits>& image_rgb);

extern template absl::StatusOr<CudaImage<Gray8u>> RgbToGray(
    const CudaImage<Rgb8u>& image_rgb);

extern template absl::StatusOr<CudaImage<Gray32f>> RgbToGray(
    const CudaImage<Rgb32f>& image_rgb);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_COLOR_CONVERSION_H_
