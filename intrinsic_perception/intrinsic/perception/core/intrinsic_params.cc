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

#include "intrinsic/perception/core/intrinsic_params.h"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <ostream>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/rectangle.h"
#include "intrinsic/util/status/status_builder.h"

namespace intrinsic {
namespace perception {

absl::StatusOr<IntrinsicParams> Crop(const IntrinsicParams& intrinsic_params,
                                     const Rectangle& roi,
                                     bool allow_out_of_bounds) {
  if (!allow_out_of_bounds &&
      !Contains(Rectangle(intrinsic_params.dimensions()), roi)) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Cropped roi must be contained in the provided image dimensions. "
           << roi << " is partially outside of an image of size "
           << intrinsic_params.dimensions();
  }
  return IntrinsicParams(roi.dimensions, intrinsic_params.focal_length_x(),
                         intrinsic_params.focal_length_y(),
                         intrinsic_params.principal_point_x() - roi.origin.col,
                         intrinsic_params.principal_point_y() - roi.origin.row);
}

IntrinsicParams Resize(const IntrinsicParams& intrinsic_params,
                       Dimensions size) {
  if (size == intrinsic_params.dimensions()) {
    return intrinsic_params;
  }
  const double scale_x =
      static_cast<double>(size.cols) / intrinsic_params.dimensions().cols;
  const double scale_y =
      static_cast<double>(size.rows) / intrinsic_params.dimensions().rows;
  return IntrinsicParams(
      size, intrinsic_params.focal_length_x() * scale_x,
      intrinsic_params.focal_length_y() * scale_y,
      (intrinsic_params.principal_point_x() + 0.5) * scale_x - 0.5,
      (intrinsic_params.principal_point_y() + 0.5) * scale_y - 0.5);
}

IntrinsicParams Resize(const IntrinsicParams& intrinsic_params, float scale_x,
                       float scale_y) {
  const Dimensions dimensions(
      std::round(intrinsic_params.dimensions().cols * scale_x),
      std::round(intrinsic_params.dimensions().rows * scale_y));
  return IntrinsicParams(
      dimensions, intrinsic_params.focal_length_x() * scale_x,
      intrinsic_params.focal_length_y() * scale_y,
      (intrinsic_params.principal_point_x() + 0.5) * scale_x - 0.5,
      (intrinsic_params.principal_point_y() + 0.5) * scale_y - 0.5);
}

IntrinsicParams Padding(const IntrinsicParams& intrinsic_params, int padding) {
  return IntrinsicParams(
      Dimensions(intrinsic_params.dimensions().cols + 2 * padding,
                 intrinsic_params.dimensions().rows + 2 * padding),
      intrinsic_params.focal_length_x(), intrinsic_params.focal_length_y(),
      intrinsic_params.principal_point_x() + padding,
      intrinsic_params.principal_point_y() + padding);
}

IntrinsicParams ReplaceDimensions(const IntrinsicParams& intrinsics,
                                  const Dimensions& new_dimensions) {
  return {new_dimensions, intrinsics.focal_length_x(),
          intrinsics.focal_length_y(), intrinsics.principal_point_x(),
          intrinsics.principal_point_y()};
}

absl::Status IntrinsicParamsNear(const IntrinsicParams& observed,
                                 const IntrinsicParams& expected, double eps,
                                 IntrinsicParamsCheckOptions check_options) {
  if (eps < 0) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Threshold 'eps' must be greater or equal to zero.";
  }
  if (check_options.check_dimensions &&
      observed.dimensions() != expected.dimensions()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Unexpected dimensions."
           << "\n\tObserved: " << observed.dimensions()
           << "\n\tExpected: " << expected.dimensions();
  }
  if (check_options.check_focal_length) {
    if (std::abs(observed.focal_length_x() - expected.focal_length_x()) > eps) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << std::setprecision(std::numeric_limits<double>::max_digits10)
             << "Focal length x is not near."
             << "\n\tObserved: " << observed.focal_length_x()
             << "\n\tExpected: " << expected.focal_length_x() << "\n\tError: "
             << std::abs(observed.focal_length_x() - expected.focal_length_x());
    }
    if (std::abs(observed.focal_length_y() - expected.focal_length_y()) > eps) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << std::setprecision(std::numeric_limits<double>::max_digits10)
             << "Focal length y is not near."
             << "\n\tObserved: " << observed.focal_length_y()
             << "\n\tExpected: " << expected.focal_length_y() << "\n\tError: "
             << std::abs(observed.focal_length_y() - expected.focal_length_y());
    }
  }
  if (check_options.check_principal_point) {
    if (std::abs(observed.principal_point_x() - expected.principal_point_x()) >
        eps) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "Principal point x is not near."
             << std::setprecision(std::numeric_limits<double>::max_digits10)
             << "\n\tObserved: " << observed.principal_point_x()
             << "\n\tExpected: " << expected.principal_point_x()
             << "\n\tError: "
             << std::abs(observed.principal_point_x() -
                         expected.principal_point_x());
    }
    if (std::abs(observed.principal_point_y() - expected.principal_point_y()) >
        eps) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << std::setprecision(std::numeric_limits<double>::max_digits10)
             << "Principal point y is not near."
             << "\n\tObserved: " << observed.principal_point_y()
             << "\n\tExpected: " << expected.principal_point_y()
             << "\n\tError: "
             << std::abs(observed.principal_point_y() -
                         expected.principal_point_y());
    }
  }
  return absl::OkStatus();
}

absl::Status IntrinsicParamsNear(
    const IntrinsicParams& observed, const IntrinsicParams& expected,
    double eps, const bool check_focal_length_and_principal_point) {
  return IntrinsicParamsNear(
      observed, expected, eps,
      IntrinsicParamsCheckOptions{
          .check_dimensions = true,
          .check_focal_length = check_focal_length_and_principal_point,
          .check_principal_point = check_focal_length_and_principal_point,
      });
}

Vector2f ProjectToImage(const IntrinsicParams& intrinsics,
                        const Vector3f& point_optical) {
  return Vector2f(
      intrinsics.focal_length_x() * (point_optical.x() / point_optical.z()) +
          intrinsics.principal_point_x(),
      intrinsics.focal_length_y() * (point_optical.y() / point_optical.z()) +
          intrinsics.principal_point_y());
}

Vector2f ProjectToImage(const IntrinsicParams& intrinsics,
                        const Isometry3f& optical_t_ref,
                        const Vector3f& point_ref) {
  const Vector3f point_optical = optical_t_ref * point_ref;
  return ProjectToImage(intrinsics, point_optical);
}

Vector2f ProjectToImageOrthographic(const IntrinsicParams& intrinsics,
                                    const Vector3f& point_optical) {
  return Vector2f(intrinsics.focal_length_x() * point_optical.x() +
                      intrinsics.principal_point_x(),
                  intrinsics.focal_length_y() * point_optical.y() +
                      intrinsics.principal_point_y());
}

std::vector<Vector2f> ProjectPointsToImage(
    const IntrinsicParams& intrinsics,
    const std::vector<Vector3f>& points_optical) {
  std::vector<Vector2f> projected_points;
  projected_points.reserve(points_optical.size());
  for (const Vector3f& p : points_optical) {
    projected_points.push_back(ProjectToImage(intrinsics, p));
  }
  return projected_points;
}

std::vector<Vector2f> ProjectPointsToImage(
    const IntrinsicParams& intrinsics, const Isometry3f& optical_t_ref,
    const std::vector<Vector3f>& points_ref) {
  std::vector<Vector2f> projected_points;
  projected_points.reserve(points_ref.size());
  for (const Vector3f& point_ref : points_ref) {
    projected_points.push_back(
        ProjectToImage(intrinsics, optical_t_ref, point_ref));
  }
  return projected_points;
}

Vector3f ProjectToOptical(const IntrinsicParams& intrinsics,
                          const Vector2f& uv) {
  return Vector3f(
      (uv.x() - intrinsics.principal_point_x()) / intrinsics.focal_length_x(),
      (uv.y() - intrinsics.principal_point_y()) / intrinsics.focal_length_y(),
      1.0f);
}

Vector3f ProjectToOptical(const IntrinsicParams& intrinsics, const Vector2f& uv,
                          float depth) {
  return Vector3f((uv.x() - intrinsics.principal_point_x()) /
                      intrinsics.focal_length_x(),
                  (uv.y() - intrinsics.principal_point_y()) /
                      intrinsics.focal_length_y(),
                  1.0f) *
         depth;
}

void PrintTo(const IntrinsicParams& intrinsics, std::ostream* os) {
  *os << absl::StrFormat(
      "{\n\tdimensions: [%d, %d],\n\tfocal_length: [%f, "
      "%f],\n\tprincipal_point: [%f, %f]\n}",
      intrinsics.dimensions().cols, intrinsics.dimensions().rows,
      intrinsics.focal_length_x(), intrinsics.focal_length_y(),
      intrinsics.principal_point_x(), intrinsics.principal_point_y());
}

std::ostream& operator<<(std::ostream& os, const IntrinsicParams& intrinsics) {
  PrintTo(intrinsics, &os);
  return os;
}

}  // namespace perception
}  // namespace intrinsic
