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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_RESIZE_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_RESIZE_H_

#include "absl/status/statusor.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"

namespace intrinsic {
namespace perception {

// Down-samples an image with dimensions cols x rows to
// Ceil((cols / 2.0)) x Ceil((rows / 2.0)). Also because the downsampling factor
// is a whole number, no bilinear interpolation is performed.
template <typename ImageTraits>
absl::StatusOr<CudaImage<ImageTraits>> Downsample2x2(
    const CudaImage<ImageTraits>& image);

extern template absl::StatusOr<CudaImage<Gray8u>> Downsample2x2<Gray8u>(
    const CudaImage<Gray8u>& image);

extern template absl::StatusOr<CudaImage<Rgb8u>> Downsample2x2<Rgb8u>(
    const CudaImage<Rgb8u>& image);

extern template absl::StatusOr<CudaImage<Gray32f>> Downsample2x2<Gray32f>(
    const CudaImage<Gray32f>& image);

extern template absl::StatusOr<CudaImage<Generic32f>> Downsample2x2<Generic32f>(
    const CudaImage<Generic32f>& image);

extern template absl::StatusOr<CudaImage<Generic32f3>>
Downsample2x2<Generic32f3>(const CudaImage<Generic32f3>& image);

Dimensions DimensionsDownsample2x2(const Dimensions& dimensions);

// Up-samples an image with dimensions cols x rows to (cols * 2) x (rows * 2)
// where odd rows and columns are filled with zeros. Optionally, we can directly
// specify 'dimensions_up' to select whether a row or a column should be dropped
// which can result in odd number of columns or rows in the returned image. This
// can be useful in a pyramid downsampling and upsampling scheme to ensure that
// all corresponding images have equal dimensions. 'dimensions_up' can be
// specified using any combination of (cols * 2 - i) x (rows * 2 - j) where the
// integers i and j can be either 0 or 1.
template <typename ImageTraits>
absl::StatusOr<CudaImage<ImageTraits>> Upsample2x2(
    const CudaImage<ImageTraits>& image, Dimensions dimensions_up = {});

extern template absl::StatusOr<CudaImage<Gray8u>> Upsample2x2<Gray8u>(
    const CudaImage<Gray8u>& image, Dimensions dimensions_up);

extern template absl::StatusOr<CudaImage<Rgb8u>> Upsample2x2<Rgb8u>(
    const CudaImage<Rgb8u>& image, Dimensions dimensions_up);

extern template absl::StatusOr<CudaImage<Gray32f>> Upsample2x2<Gray32f>(
    const CudaImage<Gray32f>& image, Dimensions dimensions_up);

extern template absl::StatusOr<CudaImage<Generic16i>> Upsample2x2<Generic16i>(
    const CudaImage<Generic16i>& image, Dimensions dimensions_up);

extern template absl::StatusOr<CudaImage<Generic16i3>> Upsample2x2<Generic16i3>(
    const CudaImage<Generic16i3>& image, Dimensions dimensions_up);

extern template absl::StatusOr<CudaImage<Generic32f>> Upsample2x2<Generic32f>(
    const CudaImage<Generic32f>& image, Dimensions dimensions_up);

extern template absl::StatusOr<CudaImage<Generic32f3>> Upsample2x2<Generic32f3>(
    const CudaImage<Generic32f3>& image, Dimensions dimensions_up);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_RESIZE_H_
