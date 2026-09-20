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

#ifndef INTRINSIC_MATH_IPOW_H_
#define INTRINSIC_MATH_IPOW_H_

#include <cstdint>

#include "absl/numeric/bits.h"

namespace intrinsic {

// Computes v^i, where i is a non-negative integer.
// When T is a floating point type, this has the same semantics as pow(), but
// is much faster.
// T can also be any integral type, in which case computations will be
// performed in the value domain of this integral type, and overflow semantics
// will be those of T.
// You can also use any type for which operator*= is defined.
template <typename T>
T IPow(T base, int exp) {
  uint32_t uexp = static_cast<uint32_t>(exp);

  if (uexp < 16) {
    T result = (uexp & 1) ? base : static_cast<T>(1);
    if (uexp >= 2) {
      base *= base;
      if (uexp & 2) {
        result *= base;
      }
      if (uexp >= 4) {
        base *= base;
        if (uexp & 4) {
          result *= base;
        }
        if (uexp >= 8) {
          base *= base;
          result *= base;
        }
      }
    }
    return result;
  }

  T result = base;
  int count = absl::countl_zero(uexp);

  uexp <<= count;
  count ^= 31;

  while (count--) {
    uexp <<= 1;
    result *= result;
    if (uexp >= 0x80000000) {
      result *= base;
    }
  }

  return result;
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_IPOW_H_
