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

#include "intrinsic/perception/gpu/cuda_image_proc/resize.h"

#include <cuda_runtime_api.h>
#include <device_types.h>
#include <vector_types.h>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_math.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {
namespace {

template <typename ImageTraits>
__global__ void Downsample2x2Kernel(
    const typename ImageTraits::ScalarType* image, int cols, int rows,
    typename ImageTraits::ScalarType* image_down, int cols_down,
    int rows_down) {
  const unsigned int x_down = blockIdx.x * blockDim.x + threadIdx.x;
  const unsigned int y_down = blockIdx.y * blockDim.y + threadIdx.y;
  if (x_down >= cols_down || y_down >= rows_down) {
    return;
  }
  constexpr int kNumChannels = ImageTraits::kNumChannels;
  image_down += y_down * cols_down * kNumChannels + x_down * kNumChannels;
  const unsigned int x_src = 2 * x_down;
  const unsigned int y_src = 2 * y_down;
  if (x_src < cols && y_src < rows) {
    image += y_src * cols * kNumChannels + x_src * kNumChannels;
#pragma unroll
    for (int i = 0; i < kNumChannels; ++i) {
      image_down[i] = image[i];
    }
  } else {
#pragma unroll
    for (int i = 0; i < kNumChannels; ++i) {
      image_down[i] = 0;
    }
  }
}

template <typename ImageTraits>
__global__ void Upsample2x2Kernel(const typename ImageTraits::ScalarType* image,
                                  int cols, int rows,
                                  typename ImageTraits::ScalarType* image_up,
                                  int cols_up, int rows_up) {
  const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
  const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= cols || y >= rows) {
    return;
  }
  const unsigned int x_up = 2 * x;
  const unsigned int y_up = 2 * y;
  if (x_up >= cols_up || y_up >= rows_up) {
    return;
  }
  constexpr int kNumChannels = ImageTraits::kNumChannels;
  image += y * cols * kNumChannels + x * kNumChannels;
  image_up += y_up * cols_up * kNumChannels + x_up * kNumChannels;
#pragma unroll
  for (int i = 0; i < kNumChannels; ++i) {
    image_up[i] = image[i];
  }
}

}  // namespace

Dimensions DimensionsDownsample2x2(const Dimensions& dimensions) {
  return Dimensions(CeilSizeDiv(dimensions.cols, 2),
                    CeilSizeDiv(dimensions.rows, 2));
}

template <typename ImageTraits>
absl::StatusOr<CudaImage<ImageTraits>> Downsample2x2(
    const CudaImage<ImageTraits>& image) {
  if (image.cols() < 2 || image.rows() < 2) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Input image must have size at least 2x2 (cols x "
                        "rows), but it had size %d x %d.",
                        image.cols(), image.rows()));
  }
  Dimensions dimensions_down = DimensionsDownsample2x2(image.dimensions());
  INTR_ASSIGN_OR_RETURN(CudaImage<ImageTraits> image_down,
                        CudaImage<ImageTraits>::Create(dimensions_down));
  constexpr int kBlockSize = 32;
  const dim3 num_blocks(CeilSizeDiv(image_down.cols(), kBlockSize),
                        CeilSizeDiv(image_down.rows(), kBlockSize));
  const dim3 num_threads_per_block(kBlockSize, kBlockSize);
  Downsample2x2Kernel<ImageTraits><<<num_blocks, num_threads_per_block>>>(
      image.data(), image.cols(), image.rows(), image_down.data(),
      image_down.cols(), image_down.rows());
  CUDA_CHECK(cudaStreamSynchronize(nullptr));
  return image_down;
}

template absl::StatusOr<CudaImage<Gray8u>> Downsample2x2<Gray8u>(
    const CudaImage<Gray8u>& image);

template absl::StatusOr<CudaImage<Rgb8u>> Downsample2x2<Rgb8u>(
    const CudaImage<Rgb8u>& image);

template absl::StatusOr<CudaImage<Gray32f>> Downsample2x2<Gray32f>(
    const CudaImage<Gray32f>& image);

template absl::StatusOr<CudaImage<Generic32f>> Downsample2x2<Generic32f>(
    const CudaImage<Generic32f>& image);

template absl::StatusOr<CudaImage<Generic32f3>> Downsample2x2<Generic32f3>(
    const CudaImage<Generic32f3>& image);

template <typename ImageTraits>
absl::StatusOr<CudaImage<ImageTraits>> Upsample2x2(
    const CudaImage<ImageTraits>& image, Dimensions dimensions_up) {
  if (image.cols() < 1 || image.rows() < 1) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Input image must have size at least 1x1 (cols x "
                        "rows), but it had size %d x %d.",
                        image.cols(), image.rows()));
  }
  if (dimensions_up.empty()) {
    dimensions_up = Dimensions(image.cols() * 2, image.rows() * 2);
  } else {
    if (dimensions_up.cols != image.cols() * 2 &&
        dimensions_up.cols != image.cols() * 2 - 1) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "dimensions_up cols must be %d or %d (was %d)", image.cols() * 2,
          image.cols() * 2 - 1, dimensions_up.cols));
    }
    if (dimensions_up.rows != image.rows() * 2 &&
        dimensions_up.rows != image.rows() * 2 - 1) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "dimensions_up rows must be %d or %d (was %d)", image.rows() * 2,
          image.rows() * 2 - 1, dimensions_up.rows));
    }
  }
  INTR_ASSIGN_OR_RETURN(CudaImage<ImageTraits> image_up,
                        CudaImage<ImageTraits>::Create(dimensions_up));
  image_up.SetZero();
  constexpr int kBlockSize = 32;
  const dim3 num_blocks(CeilSizeDiv(image.cols(), kBlockSize),
                        CeilSizeDiv(image.rows(), kBlockSize));
  const dim3 num_threads_per_block(kBlockSize, kBlockSize);
  Upsample2x2Kernel<ImageTraits><<<num_blocks, num_threads_per_block>>>(
      image.data(), image.cols(), image.rows(), image_up.data(),
      image_up.cols(), image_up.rows());
  CUDA_CHECK(cudaStreamSynchronize(nullptr));
  return image_up;
}

template absl::StatusOr<CudaImage<Gray8u>> Upsample2x2<Gray8u>(
    const CudaImage<Gray8u>& image, Dimensions dimensions_up);

template absl::StatusOr<CudaImage<Rgb8u>> Upsample2x2<Rgb8u>(
    const CudaImage<Rgb8u>& image, Dimensions dimensions_up);

template absl::StatusOr<CudaImage<Gray32f>> Upsample2x2<Gray32f>(
    const CudaImage<Gray32f>& image, Dimensions dimensions_up);

template absl::StatusOr<CudaImage<Generic16i>> Upsample2x2<Generic16i>(
    const CudaImage<Generic16i>& image, Dimensions dimensions_up);

template absl::StatusOr<CudaImage<Generic16i3>> Upsample2x2<Generic16i3>(
    const CudaImage<Generic16i3>& image, Dimensions dimensions_up);

template absl::StatusOr<CudaImage<Generic32f>> Upsample2x2<Generic32f>(
    const CudaImage<Generic32f>& image, Dimensions dimensions_up);

template absl::StatusOr<CudaImage<Generic32f3>> Upsample2x2<Generic32f3>(
    const CudaImage<Generic32f3>& image, Dimensions dimensions_up);

}  // namespace perception
}  // namespace intrinsic
