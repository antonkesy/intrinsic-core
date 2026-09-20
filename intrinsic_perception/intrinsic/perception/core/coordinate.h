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

#ifndef INTRINSIC_PERCEPTION_CORE_COORDINATE_H_
#define INTRINSIC_PERCEPTION_CORE_COORDINATE_H_

#include <algorithm>
#include <cstdint>
#include <istream>
#include <ostream>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

#include "absl/log/absl_check.h"
#include "intrinsic/perception/core/eigen_types.h"

namespace intrinsic {
namespace perception {

// Discrete 2D location for referencing a pixel in an image.
// A coordinate can be positive or negative. Mostly coordinates are positive
// however, note that there are cases where a coordinate can be negative, too,
// e.g. when doing calculations around image boundaries.
struct Coordinate {
  struct RoundTag {};
  struct TruncTag {};
  struct FloorTag {};
  struct CeilTag {};
  static constexpr RoundTag kRound;
  static constexpr TruncTag kTruncate;
  static constexpr FloorTag kFloor;
  static constexpr CeilTag kCeil;

  Coordinate() = default;
  Coordinate(int32_t col, int32_t row) : col(col), row(row) {}

  // Convert from Eigen vector to Coordinate.
  // By default we std::round. The user can request default integer truncation
  // by calling the ctor as Coordinate(v, Coordinate::kTruncate);
  template <typename Derived, typename ConversionTag = RoundTag>
  explicit Coordinate(const Eigen::MatrixBase<Derived>& vector,
                      ConversionTag conversion_tag = kRound) {
    using T = typename Derived::Scalar;
    static_assert(Derived::SizeAtCompileTime == 2,
                  "Passed matrix expression must be a 2-vector.");
    if constexpr (std::is_same_v<ConversionTag, RoundTag> &&
                  std::is_floating_point_v<T>) {
      col = static_cast<int32_t>(std::round(vector.x()));
      row = static_cast<int32_t>(std::round(vector.y()));
    } else if constexpr (std::is_same_v<ConversionTag, FloorTag> &&
                         std::is_floating_point_v<T>) {
      col = static_cast<int32_t>(std::floor(vector.x()));
      row = static_cast<int32_t>(std::floor(vector.y()));
    } else if constexpr (std::is_same_v<ConversionTag, CeilTag> &&
                         std::is_floating_point_v<T>) {
      col = static_cast<int32_t>(std::ceil(vector.x()));
      row = static_cast<int32_t>(std::ceil(vector.y()));
    } else {
      col = static_cast<int32_t>(vector.x());
      row = static_cast<int32_t>(vector.y());
    }
  }

  template <typename T>
  Vector2<T> cast() const noexcept {
    return {static_cast<T>(col), static_cast<T>(row)};
  }

  int32_t col = {0};
  int32_t row = {0};
};

inline bool operator==(const Coordinate& lhs, const Coordinate& rhs) {
  return lhs.col == rhs.col && lhs.row == rhs.row;
}

inline bool operator!=(const Coordinate& lhs, const Coordinate& rhs) {
  return lhs.col != rhs.col || lhs.row != rhs.row;
}

template <typename H>
H AbslHashValue(H h, const Coordinate& c) {
  return H::combine(std::move(h), c.col, c.row);
}

// Returns a coordinate containing the smallest value in each dimension.
inline Coordinate CwiseMin(const Coordinate& lhs, const Coordinate& rhs) {
  return {std::min(lhs.col, rhs.col), std::min(lhs.row, rhs.row)};
}

// Returns a coordinate containing the largest value in each dimension.
inline Coordinate CwiseMax(const Coordinate& lhs, const Coordinate& rhs) {
  return {std::max(lhs.col, rhs.col), std::max(lhs.row, rhs.row)};
}

inline std::ostream& operator<<(std::ostream& os,
                                const Coordinate& coordinate) {
  os << "(" << coordinate.col << ", " << coordinate.row << ")";
  return os;
}

inline std::istream& operator>>(std::istream& is, Coordinate& coordinate) {
  char ch;
  std::string line;
  std::getline(is, line, ')');
  ABSL_CHECK(!is.fail() && !is.eof() && !is.bad())
      << "Could not properly read a coordinate from istream.";
  std::stringstream linestream;
  linestream << line;
  linestream >> ch >> coordinate.col >> ch >> coordinate.row;
  return is;
}

inline Coordinate operator+(const Coordinate& lhs, const Coordinate& rhs) {
  return Coordinate(lhs.col + rhs.col, lhs.row + rhs.row);
}

inline Coordinate operator-(const Coordinate& lhs, const Coordinate& rhs) {
  return Coordinate(lhs.col - rhs.col, lhs.row - rhs.row);
}

// Returns true, if the lhs is samller than the rhs. This is the case if the lhs
// coordinate appears before the rhs coordinate when traversing coordinates in
// row major order.
inline bool operator<(const Coordinate& lhs, const Coordinate& rhs) {
  return std::tie(lhs.row, lhs.col) < std::tie(rhs.row, rhs.col);
}

// Returns true, if the lhs is smaller or equal than the rhs. This is the case
// if the lhs coordinate appears before or is identical to the rhs coordinate
// when traversing coordinates in row major order.
inline bool operator<=(const Coordinate& lhs, const Coordinate& rhs) {
  return std::tie(lhs.row, lhs.col) <= std::tie(rhs.row, rhs.col);
}

using Coordinates = std::vector<Coordinate>;

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_COORDINATE_H_
