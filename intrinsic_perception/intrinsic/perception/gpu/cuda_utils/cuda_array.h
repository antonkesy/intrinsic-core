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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_ARRAY_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_ARRAY_H_

#include <cuda_runtime.h>
#include <driver_types.h>

#include <cstddef>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_utils.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {

template <typename ScalarType>
class CudaArray {
 public:
  static absl::StatusOr<CudaArray> CreateFromCudaMemory(ScalarType* cuda_data,
                                                        size_t size) {
    if (cuda_data == nullptr) {
      return absl::InvalidArgumentError("cuda_data cannot be set to nullptr.");
    }
    if (size == 0) {
      return absl::InvalidArgumentError("size cannot be zero.");
    }
    CudaArray cuda_array;
    cuda_array.data_ = cuda_data;
    cuda_array.size_ = size;
    return cuda_array;
  }

  static absl::StatusOr<CudaArray> Create(size_t size) {
    CudaArray cuda_array;
    // We use Unified Memory to allocate the cuda array. This, among other
    // things, allows us to oversubscribe GPU memory.
    RETURN_IF_CUDA_ERROR(
        cudaMallocManaged(&cuda_array.data_, sizeof(ScalarType) * size));
    cuda_array.size_ = size;
    return cuda_array;
  }

  static absl::StatusOr<CudaArray> CreateFromHostMemory(
      const std::vector<ScalarType>& host_array) {
    INTR_ASSIGN_OR_RETURN(CudaArray cuda_array, Create(host_array.size()));
    RETURN_IF_CUDA_ERROR(cudaMemcpy(cuda_array.data_, host_array.data(),
                                    sizeof(ScalarType) * host_array.size(),
                                    cudaMemcpyHostToDevice));
    return cuda_array;
  }

  CudaArray() : size_(0), data_(nullptr) {}

  explicit CudaArray(size_t size) : size_(size) {
    if (size_ > 0) {
      CUDA_CHECK(cudaMallocManaged(&data_, sizeof(ScalarType) * size_))
          << "Cannot allocate cuda array.";
    }
  }

  // Copy constructor allocates buffer and performs a deep copy of rhs.
  CudaArray(const CudaArray& rhs) : CudaArray<ScalarType>(rhs.size()) {
    CUDA_CHECK(cudaMemcpy(data_, rhs.data(), sizeof(ScalarType) * rhs.size(),
                          cudaMemcpyDeviceToDevice))
        << "Cannot copy cuda array from device to device.";
  }

  // Move constructor performs shallow copy and sets rhs data to nullptr.
  CudaArray(CudaArray&& rhs) : size_(rhs.size_), data_(rhs.data_) {
    rhs.size_ = 0;
    rhs.data_ = nullptr;
  }

  ~CudaArray() {
    if (data_) {
      CUDA_CHECK(cudaFree(data_)) << "Cannot free cuda array.";
    }
  }

  // Copy assignment performs deep copy, with resize of lhs if needed.
  CudaArray& operator=(const CudaArray& rhs) {
    if (this == &rhs) {
      return *this;
    }
    Resize(rhs.size());
    CopyDataFromDevice(rhs.data());
    return *this;
  }

  // Move assignment performs shallow copy and sets rhs data to nullptr.
  CudaArray& operator=(CudaArray&& rhs) {
    if (data_) {
      CUDA_CHECK(cudaFree(data_)) << "Cannot free cuda array.";
    }
    data_ = rhs.data_;
    size_ = rhs.size_;
    rhs.data_ = nullptr;
    rhs.size_ = 0;
    return *this;
  }

  void Resize(size_t new_size) {
    if (size_ == new_size) {
      return;
    }
    CUDA_CHECK(cudaFree(data_)) << "Cannot free cuda array.";
    data_ = nullptr;
    if (new_size > 0) {
      CUDA_CHECK(cudaMallocManaged(&data_, sizeof(ScalarType) * new_size))
          << "Cannot allocate cuda array.";
    }
    size_ = new_size;
  }

  ScalarType operator[](size_t index) const {
    CHECK(data_) << "Cannot access cuda array because it has not been "
                    "allocated.";
    CHECK(index < size_) << "Index " << index
                         << " is out of bounds. Array size is " << size_ << ".";
    ScalarType result;
    CUDA_CHECK(cudaMemcpy(&result, data_ + index, sizeof(ScalarType),
                          cudaMemcpyDeviceToHost))
        << "Cannot copy cuda array element from device to host.";
    return result;
  }

  // Releases ownership of (but does not deallocate) the underlying cuda memory
  // and returns a pointer to that memory.
  const ScalarType* Release() {
    const ScalarType* cuda_data = data_;
    data_ = nullptr;
    size_ = 0;
    return cuda_data;
  }

  void CopyDataFromHost(const ScalarType* host_data) {
    CHECK(data_) << "Cannot copy data from host to device because the cuda "
                    "array is not allocated.";
    CUDA_CHECK(cudaMemcpy(data_, host_data, sizeof(ScalarType) * size(),
                          cudaMemcpyHostToDevice))
        << "Cannot copy data from host to device.";
  }

  void CopyDataFromDevice(const ScalarType* cuda_data) {
    CHECK(data_) << "Cannot copy data from device because the cuda array is "
                    "not allocated.";
    CUDA_CHECK(cudaMemcpy(data_, cuda_data, sizeof(ScalarType) * size(),
                          cudaMemcpyDeviceToDevice))
        << "Cannot copy data from device.";
  }

  void CopyDataToHost(ScalarType* host_data) const {
    CHECK(data_) << "Cannot copy data from device to host because the cuda "
                    "array is not allocated.";
    CUDA_CHECK(cudaMemcpy(host_data, data_, sizeof(ScalarType) * size(),
                          cudaMemcpyDeviceToHost))
        << "Cannot copy data from device to host.";
  }

  // Sets all elements to zero using cuda memset.
  void SetZero() {
    if (size_ > 0) {
      CHECK(data_)
          << "Cannot set cuda array to zero because it has not been allocated.";
      CUDA_CHECK(cudaMemset(data_, 0, sizeof(ScalarType) * size()));
    }
  }

  absl::Status Merge(CudaArray&& other) {
    if (other.size() == 0) {
      return absl::OkStatus();
    }
    if (size_ == 0) {
      *this = std::move(other);
      return absl::OkStatus();
    }

    INTR_ASSIGN_OR_RETURN(CudaArray<ScalarType> merged,
                          CudaArray<ScalarType>::Create(size_ + other.size()));
    CUDA_CHECK(cudaMemcpy(merged.data_, data_, sizeof(ScalarType) * size(),
                          cudaMemcpyDeviceToDevice));
    CUDA_CHECK(cudaMemcpy(merged.data_ + size_, other.data_,
                          sizeof(ScalarType) * other.size(),
                          cudaMemcpyDeviceToDevice));
    *this = std::move(merged);
    other.data_ = nullptr;
    other.size_ = 0;
    return absl::OkStatus();
  }

  size_t size() const { return size_; }
  ScalarType* data() { return data_; }
  const ScalarType* data() const { return data_; }

 private:
  size_t size_ = 0;
  ScalarType* data_ = nullptr;
};

template <typename ScalarType>
absl::StatusOr<std::vector<ScalarType>> FromCuda(
    const CudaArray<ScalarType>& cuda_array) {
  std::vector<ScalarType> result(cuda_array.size());
  RETURN_IF_CUDA_ERROR(cudaMemcpy(result.data(), cuda_array.data(),
                                  sizeof(ScalarType) * result.size(),
                                  cudaMemcpyDeviceToHost));
  return result;
}

template <typename ScalarType>
absl::StatusOr<std::vector<ScalarType>> FromCuda(
    const CudaArray<ScalarType>& cuda_array, size_t start_index,
    size_t num_elements) {
  INTR_RET_CHECK_LT(start_index, cuda_array.size())
      .SetCode(absl::StatusCode::kInvalidArgument);
  INTR_RET_CHECK_LE(start_index + num_elements, cuda_array.size())
      .SetCode(absl::StatusCode::kInvalidArgument);
  INTR_RET_CHECK_GT(num_elements, 0)
      .SetCode(absl::StatusCode::kInvalidArgument);
  std::vector<ScalarType> result(num_elements);
  RETURN_IF_CUDA_ERROR(
      cudaMemcpy(result.data(), cuda_array.data() + start_index,
                 sizeof(ScalarType) * result.size(), cudaMemcpyDeviceToHost));
  return result;
}

template <typename ScalarType>
absl::Status CopyToCuda(const std::vector<ScalarType>& array,
                        CudaArray<ScalarType>& cuda_array,
                        size_t cuda_start_index, size_t num_elements) {
  INTR_RET_CHECK_LE(num_elements, array.size())
          .SetCode(absl::StatusCode::kInvalidArgument)
      << "num_elements out of bounds: " << num_elements
      << " array.size(): " << array.size();
  INTR_RET_CHECK_LT(cuda_start_index, cuda_array.size())
          .SetCode(absl::StatusCode::kInvalidArgument)
      << "cuda_start_index out of bounds: " << cuda_start_index
      << " cuda_array.size(): " << cuda_array.size();
  INTR_RET_CHECK_LE(cuda_start_index + num_elements, cuda_array.size())
          .SetCode(absl::StatusCode::kInvalidArgument)
      << "cuda_start_index + num_elements out of bounds: "
      << cuda_start_index + num_elements
      << " cuda_array.size(): " << cuda_array.size();
  INTR_RET_CHECK_GT(num_elements, 0)
      .SetCode(absl::StatusCode::kInvalidArgument);
  RETURN_IF_CUDA_ERROR(
      cudaMemcpy(cuda_array.data() + cuda_start_index, array.data(),
                 sizeof(ScalarType) * num_elements, cudaMemcpyHostToDevice));
  return absl::OkStatus();
}

template <typename ScalarType>
absl::Status CopyToCuda(const std::vector<ScalarType>& array,
                        CudaArray<ScalarType>& cuda_array) {
  INTR_RET_CHECK_EQ(array.size(), cuda_array.size())
          .SetCode(absl::StatusCode::kInvalidArgument)
      << "array.size() != cuda_array.size(): " << array.size()
      << " != " << cuda_array.size();
  return CopyToCuda(array, cuda_array, 0, array.size());
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_ARRAY_H_
