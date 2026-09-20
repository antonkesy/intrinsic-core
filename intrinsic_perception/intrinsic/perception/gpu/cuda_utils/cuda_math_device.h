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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_MATH_DEVICE_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_MATH_DEVICE_H_

#include <cuda_runtime.h>

namespace intrinsic {
namespace perception {

// Computes the matrix product between a matrix and a vector.
// Here `result` = `matrix` @ `vector`.
// `matrix` has shape NumRows x NumCols.
// `vector` has size NumCols.
// `result` has size NumRows.
template <size_t NumRows, size_t NumCols>
__device__ inline void MatProdVec(const float* matrix, const float* vector,
                                  float* result) {
#pragma unroll
  for (int row = 0; row < NumRows; ++row) {
    float sum = 0.0f;
#pragma unroll
    for (int col = 0; col < NumCols; ++col) {
      sum += matrix[row * NumCols + col] * vector[col];
    }
    result[row] = sum;
  }
}

// Computes the matrix product between two matrices of compatible shapes.
// Here `result` = `matrix_a` @ `matrix_b`,
// Assuming a shape is specified as rows x cols, the dimensions are as follows:
// `matrix_a` has shape MxN.
// `matrix_b` has shape NxP.
// `result` has shape MxP.
template <size_t M, size_t N, size_t P>
__device__ inline void MatProdMat(const float* matrix_a, const float* matrix_b,
                                  float* result) {
#pragma unroll
  for (int row = 0; row < M; ++row) {
#pragma unroll
    for (int col = 0; col < P; ++col) {
      float sum = 0.0f;
#pragma unroll
      for (int i = 0; i < N; ++i) {
        sum += matrix_a[row * N + i] * matrix_b[i * P + col];
      }
      result[row * P + col] = sum;
    }
  }
}

// Sets the square `matrix` with shape NxN to identity in-place.
template <size_t N>
__device__ inline void SetIdentity(float* matrix) {
#pragma unroll
  for (int row = 0; row < N; ++row) {
#pragma unroll
    for (int col = 0; col < N; ++col) {
      matrix[row * N + col] = static_cast<float>(row == col);
    }
  }
}

// Computes the inverse of a pose matrix.
// Both the input `pose` and the output `inverse_pose` are 4x4 matrices.
//
// The `pose` matrix is formed by the 3x3 rotation matrix 'R' and the 3x1
// translation vector 't' which are stacked as follows:
// `pose` = [ R | t ]
//          [ 0 | 1 ]
//
// Assuming that the notation R^T represents the transpose of the rotation
// matrix, the output `inverse_pose`, which is also a 4x4 matrix, is then
// computed as follows:
//
// `inverse_pose` = [ (R^T) | -(R^T) @ t ]
//                  [   0   |      1     ]
//
// The last row of both `pose` and `inverse_pose` is set to [0, 0, 0, 1].
__device__ inline void InversePose(const float* pose, float* inverse_pose) {
  float r_inv[9];
#pragma unroll
  for (int row = 0; row < 3; ++row) {
#pragma unroll
    for (int col = 0; col < 3; ++col) {
      r_inv[col * 3 + row] = pose[row * 4 + col];
    }
  }
  float t[3] = {pose[0 * 4 + 3], pose[1 * 4 + 3], pose[2 * 4 + 3]};
  float t_inv[3];
  MatProdVec<3, 3>(r_inv, t, t_inv);
  SetIdentity<4>(inverse_pose);
#pragma unroll
  for (int row = 0; row < 3; ++row) {
#pragma unroll
    for (int col = 0; col < 3; ++col) {
      inverse_pose[row * 4 + col] = r_inv[row * 3 + col];
    }
    inverse_pose[row * 4 + 3] = -t_inv[row];
  }
}

// Interpolate between v0 and v1 based on the parameter t,
// where 0.0 <= t <= 1.0
__device__ inline float LinearInterp(float v0, float v1, float t) {
  return (1 - t) * v0 + t * v1;
}

// The input `value` will be cast to the integer `value_int` and clamped so that
// min <= value_int <= max.
__device__ inline int ClampToInt(float value, int min, int max) {
  int value_int = static_cast<int>(value);
  if (value_int < min) {
    value_int = min;
  } else if (value_int > max) {
    value_int = max;
  }
  return value_int;
}

// Generates a binary mask that can be used with CUDA warp shuffle functions,
// e.g. __shfl_down_sync or __shfl_up_sync. The mask specifies the active
// threads in a warp with 32 threads and ensures that all active threads are
// synchronized before the __shfl_sync operation is executed.
//
// The first active thread is specified by `first_lane_id`, and it is assumed
// that all neighboring threads, specified by `num_active_lanes`, also
// participate in the warp shuffle.
//
// For instance, setting `num_active_lanes` = 4 and `first_lane_id` = 12 would
// ensure that threads 12, 13, 14, 15 actively participate in the warp shuffle.
//
// Assumptions:
// 1) num_active_lanes <= 32.
// 2) first_lane_id >= 0 and first_lane_id < 32.
// 3) num_active_lanes + first_lane_id <= 32.
__device__ inline uint32_t GenerateWarpSyncMask(int num_active_lanes,
                                                int first_lane_id) {
  return (~(0u) >> (32 - num_active_lanes)) << first_lane_id;
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_MATH_DEVICE_H_
