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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_MATH_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_MATH_H_

#include <cstddef>
#include <cstdlib>

namespace intrinsic {
namespace perception {

// Returns the integer ceiling of the division x / y where x and y are unsigned
// integers of type size_t.
inline size_t CeilSizeDiv(size_t x, size_t y) {
  auto res = std::lldiv(x, y);
  return res.rem ? (res.quot + 1) : res.quot;
}

// Returns the rounded division x / y without a floating point operation.
inline size_t RoundSizeDiv(size_t x, size_t y) { return (x + (y / 2)) / y; }

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_MATH_H_
