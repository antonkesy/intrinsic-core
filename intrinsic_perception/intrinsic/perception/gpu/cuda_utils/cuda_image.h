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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_IMAGE_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_IMAGE_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_array.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {

template <typename ScalarT, uint8_t num_channels>
struct CudaGenericTrait {
  static constexpr uint8_t kNumChannels = num_channels;
  using PixelType = std::array<ScalarT, kNumChannels>;
  using ScalarType = ScalarT;
};

using CudaGeneric8u = CudaGenericTrait<uint8_t, 1>;
using CudaGeneric8u3 = CudaGenericTrait<uint8_t, 3>;
using CudaGeneric16u = CudaGenericTrait<uint16_t, 1>;
using CudaGeneric32i = CudaGenericTrait<int32_t, 1>;
using CudaGeneric32f = CudaGenericTrait<float, 1>;
using CudaGeneric32f3 = CudaGenericTrait<float, 3>;
using CudaGeneric64u = CudaGenericTrait<uint64_t, 1>;

template <typename MyCudaImageTrait>
class CudaImage {
 public:
  using CudaImageTrait = MyCudaImageTrait;
  using ScalarType = typename CudaImageTrait::ScalarType;
  static constexpr uint8_t kNumChannels = CudaImageTrait::kNumChannels;

  static absl::StatusOr<CudaImage> Create(const Dimensions& dimensions) {
    if (dimensions.cols <= 0 || dimensions.rows <= 0) {
      return absl::InvalidArgumentError(
          "Both width and height should be greater than zero.");
    }
    CudaImage cuda_image;
    cuda_image.cuda_array_ =
        CudaArray<ScalarType>(dimensions.area() * kNumChannels);
    cuda_image.dimensions_ = dimensions;
    return cuda_image;
  }

  static absl::StatusOr<CudaImage> Create(int cols, int rows) {
    return Create(Dimensions(cols, rows));
  }

  static absl::StatusOr<CudaImage> CreateFromCudaMemory(
      ScalarType* cuda_data, const Dimensions& dimensions) {
    if (dimensions.cols <= 0 || dimensions.rows <= 0) {
      return absl::InvalidArgumentError(
          "Both width and height should be greater than zero.");
    }
    CudaImage cuda_image;
    INTR_ASSIGN_OR_RETURN(cuda_image.cuda_array_,
                          CudaArray<ScalarType>::CreateFromCudaMemory(
                              cuda_data, dimensions.area() * kNumChannels));
    cuda_image.dimensions_ = dimensions;
    return cuda_image;
  }

  static absl::StatusOr<CudaImage> CreateFromCudaMemory(ScalarType* cuda_data,
                                                        int cols, int rows) {
    return CreateFromCudaMemory(cuda_data, Dimensions(cols, rows));
  }

  CudaImage() = default;

  // Copy constructor performs a deep copy of rhs.
  CudaImage(const CudaImage& rhs) = default;

  // Move constructor performs shallow copy and sets rhs member variables to
  // null.
  CudaImage(CudaImage&& rhs) : dimensions_(rhs.dimensions_) {
    cuda_array_ = std::move(rhs.cuda_array_);
    rhs.dimensions_ = {};
  }

  // Copy assignment performs deep copy of rhs.
  CudaImage& operator=(const CudaImage& rhs) = default;

  // Move assignment performs shallow copy and sets rhs member variables to
  // null.
  CudaImage& operator=(CudaImage&& rhs) {
    cuda_array_ = std::move(rhs.cuda_array_);
    dimensions_ = rhs.dimensions_;
    rhs.dimensions_ = {};
    return *this;
  }

  // Releases ownership of (but does not deallocate) the underlying cuda memory
  // and returns a pointer to that memory.
  const ScalarType* Release() {
    const ScalarType* cuda_data = cuda_array_.Release();
    dimensions_ = {};
    return cuda_data;
  }

  void CopyDataFromHost(const ScalarType* host_data) {
    cuda_array_.CopyDataFromHost(host_data);
  }

  void CopyDataFromDevice(const ScalarType* cuda_data) {
    cuda_array_.CopyDataFromDevice(cuda_data);
  }

  void CopyDataToHost(ScalarType* host_data) const {
    cuda_array_.CopyDataToHost(host_data);
  }

  // Sets all elements to zero using cuda memset.
  void SetZero() { cuda_array_.SetZero(); }

  int cols() const { return dimensions_.cols; }
  int rows() const { return dimensions_.rows; }
  const Dimensions& dimensions() const { return dimensions_; }
  int num_channels() const { return kNumChannels; }
  size_t size() const { return dimensions_.area() * kNumChannels; }
  ScalarType* data() { return cuda_array_.data(); }
  const ScalarType* data() const { return cuda_array_.data(); }

 private:
  CudaArray<ScalarType> cuda_array_;
  Dimensions dimensions_;
};

using CudaImage8u3 = CudaImage<CudaGeneric8u3>;
using CudaImage8u = CudaImage<CudaGeneric8u>;
using CudaImage16u = CudaImage<CudaGeneric16u>;
using CudaImage32i = CudaImage<CudaGeneric32i>;
using CudaImage32f3 = CudaImage<CudaGeneric32f3>;

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_IMAGE_H_
