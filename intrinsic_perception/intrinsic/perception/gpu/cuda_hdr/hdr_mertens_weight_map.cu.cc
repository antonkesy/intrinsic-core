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

#include "intrinsic/perception/gpu/cuda_hdr/hdr_mertens_weight_map.h"

#include <cuda_runtime_api.h>
#include <device_types.h>
#include <vector_types.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation.h"
#include "intrinsic/perception/gpu/cuda_image_proc/color_conversion.h"
#include "intrinsic/perception/gpu/cuda_image_proc/laplacian.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_array.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_math.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {
namespace {

// The normalization of all pixel intensity values is done automatically using
// the ImageTrait::kIntensityMax constant so that the range used for
// computations inside this kernel is effectively [0, 1].
//
// Inputs:
// `image` can be 1 or 3 channels and with range [0, 255] or [0, 1].
//
// `image_laplacian` is 1 channel assumed to have been computed using a
// monochrome version of the input `image` and hence it is assumed to have the
// same range as `image` which is [0, 255] or [0, 1]. Consequently the range of
// the `image_laplacian` can be [-2040, 2040] or [-8, 8] (in the unlikely
// extreme cases).
//
// Output:
// `weight_map` has 1 channel and range >= 0.0.
//
template <typename ImageTraits>
__global__ void ComputeHdrMertensWeightMapKernel(
    const typename ImageTraits::ScalarType* image, const float* image_laplacian,
    float* weight_map, int cols, int rows, float contrast_exponent = 1.0f,
    float saturation_exponent = 1.0f, float well_exposedness_exponent = 1.0f) {
  const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
  const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= cols || y >= rows) {
    return;
  }
  constexpr int kNumChannels = ImageTraits::kNumChannels;
  constexpr float kIntensityMaxInverse = 1.0f / ImageTraits::kIntensityMax;
  image += y * cols * kNumChannels + x * kNumChannels;
  float weight = 1.0f;
  if (contrast_exponent > 0.0f) {
    const float contrast =
        fabs(image_laplacian[y * cols + x] * kIntensityMaxInverse);
    weight *= powf(contrast, contrast_exponent);
  }
  // The saturation metric does not need to be computed for monochrome images.
  if (saturation_exponent > 0.0f && kNumChannels > 1) {
    float pixels[kNumChannels];
    float mean = 0.0f;
#pragma unroll
    for (int i = 0; i < kNumChannels; ++i) {
      pixels[i] = static_cast<float>(image[i]) * kIntensityMaxInverse;
      mean += pixels[i];
    }
    mean /= static_cast<float>(kNumChannels);
    float variance = 0.0f;
#pragma unroll
    for (int i = 0; i < kNumChannels; ++i) {
      float val = pixels[i] - mean;
      variance += val * val;
    }
    variance /= static_cast<float>(kNumChannels);
    const float stdev = sqrt(variance);
    weight *= powf(stdev, saturation_exponent);
  }
  if (well_exposedness_exponent > 0.0f) {
    constexpr float kWellExposednessSigma = 0.2f;
    constexpr float alpha =
        -0.5f / (kWellExposednessSigma * kWellExposednessSigma);
    float all_channel_product = 1.0f;
#pragma unroll
    for (int i = 0; i < kNumChannels; ++i) {
      const float x =
          (static_cast<float>(image[i]) * kIntensityMaxInverse) - 0.5f;
      all_channel_product *= exp(alpha * (x * x));
    }
    weight *= powf(all_channel_product, well_exposedness_exponent);
  }
  // Add small constant to avoid potential division by zero during
  // normalization.
  weight_map[y * cols + x] = weight + 1e-12f;
}

__global__ void NormalizeWeightMapsKernel(float** weight_maps_dataptrs,
                                          int num_weight_maps, int cols,
                                          int rows) {
  const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
  const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= cols || y >= rows) {
    return;
  }
  const size_t pixel_index = y * cols + x;
  float sum = 0.0f;
  for (int i = 0; i < num_weight_maps; ++i) {
    sum += weight_maps_dataptrs[i][pixel_index];
  }
  for (int i = 0; i < num_weight_maps; ++i) {
    // Sum will never be zero because we add a small constant to the final
    // weight in ComputeHdrMertensWeightMapKernel.
    weight_maps_dataptrs[i][pixel_index] /= sum;
  }
}

template <typename ImageTraits>
__global__ void BlendWeightMapAndImageKernel(
    const float* weight_map, const typename ImageTraits::ScalarType* image,
    float* accumulator, int cols, int rows) {
  const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
  const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= cols || y >= rows) {
    return;
  }
  constexpr int kNumChannels = ImageTraits::kNumChannels;
  const size_t pixel_index = y * cols + x;
  const size_t pixel_index_with_channels = pixel_index * kNumChannels;
  weight_map += pixel_index;
  image += pixel_index_with_channels;
  accumulator += pixel_index_with_channels;
#pragma unroll
  for (int i = 0; i < kNumChannels; ++i) {
    accumulator[i] += weight_map[0] * static_cast<float>(image[i]);
  }
}

}  // namespace

template <typename ImageTraits>
absl::StatusOr<CudaImage<Generic32f>> ComputeHdrMertensWeightMap(
    const CudaImage<ImageTraits>& image, float contrast_exponent,
    float saturation_exponent, float well_exposedness_exponent,
    BorderType border_type) {
  static_assert(
      ImageTraits::kNumChannels == 1 || ImageTraits::kNumChannels == 3,
      "image must have either 1 or 3 channels.");
  INTR_ASSIGN_OR_RETURN(CudaImage<Generic32f> weight_map,
                        CudaImage<Generic32f>::Create(image.dimensions()));
  CudaImage<Generic32f> image_laplacian;
  if (contrast_exponent > 0) {
    if constexpr (ImageTraits::kNumChannels == 3) {
      INTR_ASSIGN_OR_RETURN(
          const auto image_gray,
          (RgbToGray<GrayTrait<typename ImageTraits::ScalarType,
                               ImageTraits::kIntensityMax>>(image)));
      INTR_ASSIGN_OR_RETURN(image_laplacian,
                            ApplyLaplacian3x3(image_gray, border_type));
    } else {
      INTR_ASSIGN_OR_RETURN(image_laplacian,
                            ApplyLaplacian3x3(image, border_type));
    }
  }
  constexpr int kBlockSize = 16;
  const dim3 num_blocks(CeilSizeDiv(image.cols(), kBlockSize),
                        CeilSizeDiv(image.rows(), kBlockSize));
  const dim3 num_threads_per_block(kBlockSize, kBlockSize);
  ComputeHdrMertensWeightMapKernel<ImageTraits>
      <<<num_blocks, num_threads_per_block>>>(
          image.data(), image_laplacian.data(), weight_map.data(), image.cols(),
          image.rows(), contrast_exponent, saturation_exponent,
          well_exposedness_exponent);
  CUDA_CHECK(cudaStreamSynchronize(nullptr));
  return weight_map;
}

template absl::StatusOr<CudaImage<Generic32f>> ComputeHdrMertensWeightMap(
    const CudaImage<Gray8u>& image, float contrast_exponent,
    float saturation_exponent, float well_exposedness_exponent,
    BorderType border_type);

template absl::StatusOr<CudaImage<Generic32f>> ComputeHdrMertensWeightMap(
    const CudaImage<Rgb8u>& image, float contrast_exponent,
    float saturation_exponent, float well_exposedness_exponent,
    BorderType border_type);

template absl::StatusOr<CudaImage<Generic32f>> ComputeHdrMertensWeightMap(
    const CudaImage<Gray32f>& image, float contrast_exponent,
    float saturation_exponent, float well_exposedness_exponent,
    BorderType border_type);

template absl::StatusOr<CudaImage<Generic32f>> ComputeHdrMertensWeightMap(
    const CudaImage<Rgb32f>& image, float contrast_exponent,
    float saturation_exponent, float well_exposedness_exponent,
    BorderType border_type);

absl::Status NormalizeWeightMaps(
    std::vector<CudaImage<Generic32f>>& weight_maps) {
  if (weight_maps.empty()) {
    return absl::InvalidArgumentError("weight_maps must not be empty.");
  }
  Dimensions dimensions = weight_maps[0].dimensions();
  for (int i = 1; i < weight_maps.size(); ++i) {
    if (weight_maps[i].dimensions() != dimensions) {
      return absl::InvalidArgumentError(
          "Ensure that all weight maps have the same dimensions.");
    }
  }
  std::vector<const float*> weight_maps_dataptrs;
  for (const auto& weight_map : weight_maps) {
    weight_maps_dataptrs.push_back(weight_map.data());
  }
  INTR_ASSIGN_OR_RETURN(
      CudaArray<const float*> cuda_weight_maps_dataptrs,
      CudaArray<const float*>::CreateFromHostMemory(weight_maps_dataptrs));
  constexpr int kBlockSize = 16;
  const dim3 num_blocks(CeilSizeDiv(dimensions.cols, kBlockSize),
                        CeilSizeDiv(dimensions.rows, kBlockSize));
  const dim3 num_threads_per_block(kBlockSize, kBlockSize);
  NormalizeWeightMapsKernel<<<num_blocks, num_threads_per_block>>>(
      const_cast<float**>(cuda_weight_maps_dataptrs.data()),
      cuda_weight_maps_dataptrs.size(), dimensions.cols, dimensions.rows);
  CUDA_CHECK(cudaStreamSynchronize(nullptr));
  return absl::OkStatus();
}

template <typename ImageTraits>
absl::Status BlendWeightMapAndImage(
    const CudaImage<Generic32f>& weight_map,
    const CudaImage<ImageTraits>& image,
    CudaImage<GenericTrait<float, ImageTraits::kNumChannels>>& accumulator) {
  if (weight_map.dimensions() != image.dimensions()) {
    return absl::InvalidArgumentError(
        "weight_map and image must have the same dimensions.");
  }
  if (weight_map.dimensions() != accumulator.dimensions()) {
    return absl::InvalidArgumentError(
        "weight_map and accumulator must have the same dimensions.");
  }
  constexpr int kBlockSize = 16;
  const dim3 num_blocks(CeilSizeDiv(image.cols(), kBlockSize),
                        CeilSizeDiv(image.rows(), kBlockSize));
  const dim3 num_threads_per_block(kBlockSize, kBlockSize);
  BlendWeightMapAndImageKernel<ImageTraits>
      <<<num_blocks, num_threads_per_block>>>(weight_map.data(), image.data(),
                                              accumulator.data(), image.cols(),
                                              image.rows());
  CUDA_CHECK(cudaStreamSynchronize(nullptr));
  return absl::OkStatus();
}

template absl::Status BlendWeightMapAndImage(
    const CudaImage<Generic32f>& weight_map, const CudaImage<Generic16i>& image,
    CudaImage<Generic32f>& accumulator);

template absl::Status BlendWeightMapAndImage(
    const CudaImage<Generic32f>& weight_map,
    const CudaImage<Generic16i3>& image, CudaImage<Generic32f3>& accumulator);

template absl::Status BlendWeightMapAndImage(
    const CudaImage<Generic32f>& weight_map, const CudaImage<Generic32f>& image,
    CudaImage<Generic32f>& accumulator);

template absl::Status BlendWeightMapAndImage(
    const CudaImage<Generic32f>& weight_map,
    const CudaImage<Generic32f3>& image, CudaImage<Generic32f3>& accumulator);

}  // namespace perception
}  // namespace intrinsic
