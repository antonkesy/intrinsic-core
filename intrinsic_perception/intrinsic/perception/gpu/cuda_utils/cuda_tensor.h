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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_TENSOR_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_TENSOR_H_

#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_array.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {

inline absl::StatusOr<size_t> ComputeNumElements(
    const std::vector<int>& shape) {
  if (shape.empty()) {
    return absl::InvalidArgumentError("shape must not be empty.");
  }
  size_t num_elems = 1;
  for (int i = 0; i < shape.size(); ++i) {
    if (shape[i] <= 0) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "shape value for dimension %d must be greater than zero. (was %d)", i,
          shape[i]));
    }
    num_elems *= shape[i];
  }
  return num_elems;
}

template <typename ScalarType>
class CudaTensor {
 public:
  CudaTensor() = default;

  static absl::StatusOr<CudaTensor> Create(const std::vector<int>& shape) {
    INTR_ASSIGN_OR_RETURN(size_t num_elems, ComputeNumElements(shape));
    CudaTensor cuda_tensor;
    cuda_tensor.cuda_array_ = CudaArray<ScalarType>(num_elems);
    cuda_tensor.shape_ = shape;
    return cuda_tensor;
  }

  static absl::StatusOr<CudaTensor> CreateFromCudaMemory(
      ScalarType* cuda_data, const std::vector<int>& shape) {
    INTR_ASSIGN_OR_RETURN(size_t num_elems, ComputeNumElements(shape));
    CudaTensor cuda_tensor;
    INTR_ASSIGN_OR_RETURN(
        cuda_tensor.cuda_array_,
        CudaArray<ScalarType>::CreateFromCudaMemory(cuda_data, num_elems));
    cuda_tensor.shape_ = shape;
    return cuda_tensor;
  }

  // Copy constructor performs a deep copy of rhs.
  CudaTensor(const CudaTensor& rhs) = default;

  // Move constructor performs shallow copy and sets rhs member variables to
  // null.
  CudaTensor(CudaTensor&& rhs) {
    cuda_array_ = std::move(rhs.cuda_array_);
    shape_ = std::move(rhs.shape_);
    rhs.shape_ = {};
  }

  // Copy assignment performs deep copy of rhs.
  CudaTensor& operator=(const CudaTensor& rhs) = default;

  // Move assignment performs shallow copy and sets rhs member variables to
  // null.
  CudaTensor& operator=(CudaTensor&& rhs) {
    cuda_array_ = std::move(rhs.cuda_array_);
    shape_ = std::move(rhs.shape_);
    rhs.shape_ = {};
    return *this;
  }

  // Reshapes the tensor if the new shape is compatible with the old shape, i.e.
  // when both have equal number of elements. Otherwise the function returns an
  // InvalidArgumentError. If reshaping was successful, the old data is
  // retained, i.e. there is no data loss.
  absl::Status Reshape(const std::vector<int>& shape) {
    INTR_ASSIGN_OR_RETURN(size_t num_elems_new, ComputeNumElements(shape));
    INTR_ASSIGN_OR_RETURN(size_t num_elems, ComputeNumElements(shape_));
    if (num_elems != num_elems_new) {
      return absl::InvalidArgumentError(
          "The new shape is incompatible with the old shape because they have "
          "different number of elements.");
    }
    shape_ = shape;
    return absl::OkStatus();
  }

  // Releases ownership of (but does not deallocate) the underlying cuda memory
  // and returns a pointer to that memory.
  const ScalarType* Release() {
    const ScalarType* cuda_data = cuda_array_.Release();
    shape_ = {};
    return cuda_data;
  }

  void CopyDataFromHost(const ScalarType* host_data) {
    cuda_array_.CopyDataFromHost(host_data);
  }

  void CopyDataToHost(ScalarType* host_data) const {
    cuda_array_.CopyDataToHost(host_data);
  }

  // Sets all elements to zero using cuda memset.
  void SetZero() { cuda_array_.SetZero(); }

  bool HasSameShape(const CudaTensor& rhs) const {
    return shape_ == rhs.shape_;
  }

  const std::vector<int>& shape() const { return shape_; }
  size_t size() const { return cuda_array_.size(); }
  const ScalarType* data() const { return cuda_array_.data(); }
  ScalarType* data() { return cuda_array_.data(); }

 private:
  CudaArray<ScalarType> cuda_array_;
  std::vector<int> shape_;
};

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_TENSOR_H_
