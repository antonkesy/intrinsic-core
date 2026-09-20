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

#ifndef INTRINSIC_PERCEPTION_CORE_RECTANGLE_H_
#define INTRINSIC_PERCEPTION_CORE_RECTANGLE_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <istream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "absl/log/absl_check.h"
#include "intrinsic/perception/core/coordinate.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/eigen_types.h"

namespace intrinsic {
namespace perception {

// Region of interest within an image. Specified by a point and a
// dimension. The point itself is included in the region of interest.
// The point made up by the origin point and the shape is not included
// in the region of interest.
struct Rectangle {
  Rectangle() : origin(0, 0), dimensions(0, 0) {}
  Rectangle(int32_t col, int32_t row, int32_t width, int32_t height)
      : origin(col, row), dimensions(width, height) {}

  // Constructs a rectangle from two extremal points (top-left and bottom-right)
  // which are both included in the resulting rectangle.
  Rectangle(const Coordinate& top_left, const Coordinate& bottom_right)
      : origin(top_left),
        dimensions(bottom_right.col - top_left.col + 1,
                   bottom_right.row - top_left.row + 1) {
    ABSL_CHECK(top_left.col <= bottom_right.col)
        << "Wrong input coordinates. The top left col " << top_left.col << " < "
        << bottom_right.col << ".";
    ABSL_CHECK(top_left.row <= bottom_right.row)
        << "Wrong input coordinates. The top left row " << top_left.row << " < "
        << bottom_right.row << ".";
  }

  // Constructs a rectangle at the given position of the specified size.
  Rectangle(const Coordinate& origin, const Dimensions& dimensions)
      : origin(origin), dimensions(dimensions) {}

  // Constructs a rectangle located at the origin with the user specified size.
  explicit Rectangle(const Dimensions& dimensions)
      : origin(0, 0), dimensions(dimensions) {}

  Coordinate TopLeft() const { return origin; }

  Coordinate BottomRight() const {
    return Coordinate(origin.col + dimensions.cols - 1,
                      origin.row + dimensions.rows - 1);
  }

  // Returns true, if the rectangle is empty.
  bool Empty() const { return dimensions.cols == 0 || dimensions.rows == 0; }

  // Returns the area of an rectangle.
  int32_t area() const { return dimensions.cols * dimensions.rows; }

  static Rectangle Invalid() { return Rectangle(); }

  // Returns true if either of the rectangle's dimensions is non-positive.
  // TODO(hinterst): Why is this func static and not a free function?
  static bool IsInvalid(const Rectangle& r) {
    return r.dimensions.cols <= 0 || r.dimensions.rows <= 0;
  }

  // Returns true, if both of the rectangle's dimensions are positive.
  // TODO(hinterst): Why is this func static and not a free function?
  static bool IsValid(const Rectangle& r) { return !IsInvalid(r); }

  // origin of region of interest
  Coordinate origin;
  // shape of region of interests
  Dimensions dimensions;
};

inline bool operator==(const Rectangle& lhs, const Rectangle& rhs) {
  return lhs.origin == rhs.origin && lhs.dimensions == rhs.dimensions;
}

inline bool operator!=(const Rectangle& lhs, const Rectangle& rhs) {
  return !(lhs == rhs);
}

// Returns the minimal rectangle which contains all passed floating point
// pixels. If an empty range is passed, the returned rectangle is empty.
template <int Options>
Rectangle CreateBoundingRectangle(
    const std::vector<Eigen::Matrix<float, 2, 1, Options>>& float_pixels) {
  if (float_pixels.empty()) return {};
  Vector2f min = float_pixels.front();
  Vector2f max = float_pixels.front();
  for (size_t i = 1; i < float_pixels.size(); ++i) {
    min = min.cwiseMin(float_pixels[i]);
    max = max.cwiseMax(float_pixels[i]);
  }
  const Coordinate mini(std::floor(min[0]), std::floor(min[1]));
  const Coordinate maxi(std::ceil(max[0]), std::ceil(max[1]));
  return Rectangle{mini, maxi};
}

template <int Options>
Rectangle CreateBoundingRectangle(
    const std::initializer_list<Eigen::Matrix<float, 2, 1, Options>>&
        float_pixels) {
  auto float_pixel_iter = float_pixels.begin();
  Vector2f min = *float_pixel_iter;
  Vector2f max = *float_pixel_iter;
  for (++float_pixel_iter; float_pixel_iter != float_pixels.end();
       ++float_pixel_iter) {
    min = min.cwiseMin(*float_pixel_iter);
    max = max.cwiseMax(*float_pixel_iter);
  }
  const Coordinate mini(std::floor(min[0]), std::floor(min[1]));
  const Coordinate maxi(std::ceil(max[0]), std::ceil(max[1]));
  return Rectangle{mini, maxi};
}

// Checks if a point is inside the region of interest rectangle.
inline bool IsInBounds(const Rectangle& r, const Coordinate& point) {
  return point.col >= r.origin.col &&
         point.col < r.origin.col + r.dimensions.cols &&
         point.row >= r.origin.row &&
         point.row < r.origin.row + r.dimensions.rows;
}

// Checks if a point is inside the region of interest rectangle.
inline bool IsInBounds(const Rectangle& r, const Vector2f& point) {
  return point[0] >= r.origin.col &&
         point[0] < r.origin.col + r.dimensions.cols &&
         point[1] >= r.origin.row &&
         point[1] < r.origin.row + r.dimensions.rows;
}

// Returns true, if the first rectangle contains the second.
inline bool Contains(const Rectangle& a, const Rectangle& b) {
  return IsInBounds(a, b.TopLeft()) && IsInBounds(a, b.BottomRight());
}

template <typename T, int Options>
bool Contains(const Rectangle& r,
              const Eigen::Matrix<T, 2, 1, Options>& point) {
  return point[0] >= r.origin.col &&
         point[0] < r.origin.col + r.dimensions.cols &&
         point[1] >= r.origin.row &&
         point[1] < r.origin.row + r.dimensions.rows;
}

// Returns true, if the two rectangles intersect.
inline bool Intersect(const Rectangle& a, const Rectangle& b) {
  if (a.Empty() || b.Empty()) return false;
  return a.origin.col < (b.origin.col + b.dimensions.cols) &&
         b.origin.col < (a.origin.col + a.dimensions.cols) &&  // cols intersect
         a.origin.row < (b.origin.row + b.dimensions.rows) &&
         b.origin.row < (a.origin.row + a.dimensions.rows);  // rows intersect
}

// Returns the union of the two rectangles.
inline Rectangle Union(const Rectangle& a, const Rectangle& b) {
  if (a.Empty()) return b;
  if (b.Empty()) return a;
  const Coordinate top_left = CwiseMin(a.TopLeft(), b.TopLeft());
  const Coordinate bottom_right = CwiseMax(a.BottomRight(), b.BottomRight());
  return {top_left, bottom_right};
}

// Returns the intersection of the two rectangles.
inline Rectangle Intersected(const Rectangle& a, const Rectangle& b) {
  if (!Intersect(a, b)) return Rectangle();
  const Coordinate top_left = CwiseMax(a.TopLeft(), b.TopLeft());
  const Coordinate bottom_right = CwiseMin(a.BottomRight(), b.BottomRight());
  return {top_left, bottom_right};
}

// Returns the intersection over union of the two rectangles.
inline float IntersectionOverUnion(const Rectangle& a, const Rectangle& b) {
  const auto intersection_rectangle = Intersected(a, b);
  const float intersection_area = intersection_rectangle.area();
  if ((a.area() + b.area() - intersection_area) > 0.0f)
    return intersection_area / (a.area() + b.area() - intersection_area);
  else
    return 0.0f;
}

inline Rectangle Padding(const Rectangle& roi, int padding) {
  return Rectangle(roi.origin.col - padding, roi.origin.row - padding,
                   roi.dimensions.cols + padding * 2,
                   roi.dimensions.rows + padding * 2);
}

inline std::ostream& operator<<(std::ostream& os, const Rectangle& rect) {
  os << "(" << rect.origin.col << ", " << rect.origin.row << ", ";
  os << rect.dimensions.cols << ", " << rect.dimensions.rows << ")";
  return os;
}

inline std::istream& operator>>(std::istream& is, Rectangle& rect) {
  char ch;
  std::string line;
  std::getline(is, line, ')');
  ABSL_CHECK(!is.fail() && !is.eof() && !is.bad())
      << "Could not properly read a rectangle from istream.";
  std::stringstream linestream;
  linestream << line;
  linestream >> ch >> rect.origin.col >> ch >> rect.origin.row >> ch >>
      rect.dimensions.cols >> ch >> rect.dimensions.rows;
  return is;
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_RECTANGLE_H_
