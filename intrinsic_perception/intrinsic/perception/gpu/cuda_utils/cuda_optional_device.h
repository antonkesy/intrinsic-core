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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_OPTIONAL_DEVICE_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_OPTIONAL_DEVICE_H_

#include <assert.h>
#include <cuda_runtime.h>

namespace intrinsic {
namespace perception {

template <typename T>
class CudaOptional {
 public:
  __device__ CudaOptional() : value_{}, value_is_valid_(false) {}

  __device__ explicit CudaOptional(const T& value)
      : value_(value), value_is_valid_(true) {}

  // Copy constructor.
  __device__ CudaOptional(const CudaOptional& rhs) = default;

  // Move constructor.
  __device__ CudaOptional(CudaOptional&& rhs)
      : value_(std::move(rhs.value_)), value_is_valid_(rhs.value_is_valid_) {}

  // Copy assignment.
  __device__ CudaOptional& operator=(const CudaOptional& rhs) = default;

  // Destructor.
  __device__ ~CudaOptional() = default;

  // Move assignment.
  __device__ CudaOptional& operator=(CudaOptional&& rhs) {
    value_ = std::move(rhs.value_);
    value_is_valid_ = rhs.value_is_valid_;
    return *this;
  }

  // Assign value using a copy and set value_is_valid_ to true.
  __device__ CudaOptional& operator=(const T& value) {
    value_ = value;
    value_is_valid_ = true;
    return *this;
  }

  // Assign value using move semantics and set value_is_valid_ to true.
  __device__ CudaOptional& operator=(T&& value) {
    value_ = std::forward<T>(value);
    value_is_valid_ = true;
    return *this;
  }

  __device__ constexpr bool has_value() const { return value_is_valid_; }

  __device__ constexpr explicit operator bool() const { return has_value(); }

  __device__ constexpr const T& operator*() const {
    assert(value_is_valid_);
    return value_;
  }

 private:
  T value_;
  bool value_is_valid_;
};

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_OPTIONAL_DEVICE_H_
