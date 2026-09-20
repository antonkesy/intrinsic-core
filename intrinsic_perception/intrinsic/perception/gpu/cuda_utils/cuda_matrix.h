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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_MATRIX_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_MATRIX_H_

#include "intrinsic/perception/gpu/cuda_utils/cuda_array.h"

namespace intrinsic {
namespace perception {

template <typename ScalarType, size_t num_rows, size_t num_cols>
class CudaMatrix : public CudaArray<ScalarType> {
 public:
  CudaMatrix() : CudaArray<ScalarType>(num_rows * num_cols) {
    static_assert(num_rows > 0);
    static_assert(num_cols > 0);
  }

  size_t rows() const { return num_rows; }
  size_t cols() const { return num_cols; }
};

using CudaMatrix3f = CudaMatrix<float, 3, 3>;
using CudaMatrix4f = CudaMatrix<float, 4, 4>;
using CudaMatrix34f = CudaMatrix<float, 3, 4>;

template <typename ScalarType, size_t N>
using CudaVector = CudaMatrix<ScalarType, N, 1>;

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_MATRIX_H_
