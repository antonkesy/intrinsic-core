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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_HDR_HDR_MERTENS_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_HDR_HDR_MERTENS_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"

namespace intrinsic {
namespace perception {

struct HdrMertensParams {
  // The exponents below should be >= 0.0. If an exponent is set to 0.0 the
  // corresponding measure is not taken into account. The weight of a given
  // pixel in a particular LDR image is computed as a product of all the
  // measures as below:
  //
  //    weight = (contrast^contrast_exponent) * (saturation^saturation_exponent)
  //      * (well_exposedness^well_exposedness_exponent)
  //
  // Contrast at a particular pixel is computed as the absolute value of a
  // laplacian filter response at that pixel. For RGB images we first
  // convert to grayscale.
  float contrast_exponent = 1.0f;
  // Saturation is measured as the standard deviation of the RGB measurements at
  // each pixel. This measure is skipped for single-channel grayscale images.
  float saturation_exponent = 1.0f;
  // Well-exposedness measures how far a pixel value is from the mid-point,
  // which is 0.5 for images with range [0..1]. For multi-channel images this
  // measure is multiplied for all channels.
  float well_exposedness_exponent = 1.0f;
  // The blending of multiple exposures is performed using an image pyramid,
  // starting at the coarsest level. Here we specify the minimum required image
  // dimensions at the coarsest level.
  Dimensions pyramid_min_dimensions = {1, 1};
  // Border extrapolation type for handling out-of-bounds pixels during
  // smoothing and image pyramid operations.
  BorderType border_type = BorderType::kBorderReflect101;
};

// Setting the `use_float_precision` flag will store images as float32 during
// color conversion and the laplacian pyramid computation. This can result in
// slower execution, but because the intermediate image representations retain
// more precision, the result can be more similar to the one produced by OpenCV.
template <typename ImageTraits>
absl::StatusOr<Image<ImageTraits>> ComputeHdrMertens(
    const std::vector<const Image<ImageTraits>*>& ldr_images,
    const HdrMertensParams& params, bool use_float_precision = false);

extern template absl::StatusOr<Image<Gray8u>> ComputeHdrMertens(
    const std::vector<const Image<Gray8u>*>& ldr_images,
    const HdrMertensParams& params, bool use_float_precision);

extern template absl::StatusOr<Image<Rgb8u>> ComputeHdrMertens(
    const std::vector<const Image<Rgb8u>*>& ldr_images,
    const HdrMertensParams& params, bool use_float_precision);

template <typename ImageTraits>
absl::StatusOr<CudaImage<ImageTraits>> ComputeHdrMertens(
    const std::vector<CudaImage<ImageTraits>>& ldr_images,
    const HdrMertensParams& params);

extern template absl::StatusOr<CudaImage<Gray8u>> ComputeHdrMertens(
    const std::vector<CudaImage<Gray8u>>& ldr_images,
    const HdrMertensParams& params);

extern template absl::StatusOr<CudaImage<Rgb8u>> ComputeHdrMertens(
    const std::vector<CudaImage<Rgb8u>>& ldr_images,
    const HdrMertensParams& params);

extern template absl::StatusOr<CudaImage<Gray32f>> ComputeHdrMertens(
    const std::vector<CudaImage<Gray32f>>& ldr_images,
    const HdrMertensParams& params);

extern template absl::StatusOr<CudaImage<Rgb32f>> ComputeHdrMertens(
    const std::vector<CudaImage<Rgb32f>>& ldr_images,
    const HdrMertensParams& params);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_HDR_HDR_MERTENS_H_
