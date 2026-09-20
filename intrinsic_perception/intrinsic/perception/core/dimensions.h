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

#ifndef INTRINSIC_PERCEPTION_CORE_DIMENSIONS_H_
#define INTRINSIC_PERCEPTION_CORE_DIMENSIONS_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <istream>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>

#include "absl/log/absl_check.h"
#include "absl/status/statusor.h"
#include "intrinsic/util/status/status_builder.h"

namespace intrinsic {
namespace perception {

/**
 * @brief The dimensions of an image: cols and rows.
 *
 * A dimension can be positive or negative. Although
 * a dimension is mostly positive there are cases
 * where a dimension can be negative, e.g. when doing
 * calculations around image boundaries.
 */
struct Dimensions {
  Dimensions() = default;
  Dimensions(int32_t cols, int32_t rows) : cols(cols), rows(rows) {}

  int32_t area() const { return cols * rows; }

  bool empty() const { return cols == 0 || rows == 0; }

  int32_t cols = {0};  // width of the image
  int32_t rows = {0};  // height of the image

  template <typename H>
  friend H AbslHashValue(H h, const Dimensions& dims) {
    return H::combine(std::move(h), dims.cols, dims.rows);
  }
};

inline bool operator==(const Dimensions& lhs, const Dimensions& rhs) {
  return lhs.cols == rhs.cols && lhs.rows == rhs.rows;
}

inline bool operator!=(const Dimensions& lhs, const Dimensions& rhs) {
  return lhs.cols != rhs.cols || lhs.rows != rhs.rows;
}

inline Dimensions operator+(const Dimensions& lhs, const Dimensions& rhs) {
  return {lhs.cols + rhs.cols, lhs.rows + rhs.rows};
}

inline Dimensions operator-(const Dimensions& lhs, const Dimensions& rhs) {
  return {lhs.cols - rhs.cols, lhs.rows - rhs.rows};
}

inline Dimensions operator*(double factor, const Dimensions& dims) {
  return {static_cast<int32_t>(factor * dims.cols),
          static_cast<int32_t>(factor * dims.rows)};
}

inline Dimensions operator*(const Dimensions& dims, double factor) {
  return factor * dims;
}

inline Dimensions operator/(const Dimensions& dims, double factor) {
  return {static_cast<int32_t>(dims.cols / factor),
          static_cast<int32_t>(dims.rows / factor)};
}

inline std::ostream& operator<<(std::ostream& os, const Dimensions& dims) {
  os << "(" << dims.cols << ", " << dims.rows << ")";
  return os;
}

inline Dimensions Union(const Dimensions& a, const Dimensions& b) {
  return {std::max(a.cols, b.cols), std::max(a.rows, b.rows)};
}

inline std::istream& operator>>(std::istream& is, Dimensions& dims) {
  char ch;
  std::string line;
  std::getline(is, line, ')');
  ABSL_CHECK(!is.fail() && !is.eof() && !is.bad())
      << "Could not properly read a dimension from istream.";
  std::stringstream linestream;
  linestream << line;
  linestream >> ch >> dims.cols >> ch >> dims.rows;
  return is;
}

inline absl::StatusOr<Dimensions> Resize(Dimensions dimensions,
                                         std::optional<int32_t> width,
                                         std::optional<int32_t> height) {
  if (width.has_value() && height.has_value()) {
    return Dimensions(width.value(), height.value());
  }
  if (width.has_value()) {
    if (dimensions.cols == 0) {
      return InvalidArgumentErrorBuilder()
             << "Dimensions cols must not be zero.";
    }
    return Dimensions(
        width.value(),
        std::round(width.value() * static_cast<float>(dimensions.rows) /
                   dimensions.cols));
  }
  if (height.has_value()) {
    if (dimensions.rows == 0) {
      return InvalidArgumentErrorBuilder()
             << "Dimensions rows must not be zero.";
    }
    return Dimensions(
        std::round(height.value() * static_cast<float>(dimensions.cols) /
                   dimensions.rows),
        height.value());
  }
  return dimensions;
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_DIMENSIONS_H_
