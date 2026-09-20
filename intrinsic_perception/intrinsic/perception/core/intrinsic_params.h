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

#ifndef INTRINSIC_PERCEPTION_CORE_INTRINSIC_PARAMS_H_
#define INTRINSIC_PERCEPTION_CORE_INTRINSIC_PARAMS_H_

#include <ostream>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/rectangle.h"

namespace intrinsic {
namespace perception {

// This class represent intrinsic camera parameters. Note that this class only
// considers intrinsic camera parameters from the linear camera model.
// Non-linear distortions are to be handled separately. See here for details:
// https://en.wikipedia.org/wiki/Camera_resectioning#Intrinsic_parameters
class IntrinsicParams {
 public:
  // Creates intrinsic parameters with a perfectly centered principal point.
  IntrinsicParams(const Dimensions& dimensions, double focal_length_x,
                  double focal_length_y)
      : IntrinsicParams(
            dimensions, focal_length_x, focal_length_y,
            dimensions.cols == 0 ? 0.0 : 0.5 * (dimensions.cols - 1),
            dimensions.rows == 0 ? 0.0 : 0.5 * (dimensions.rows - 1)) {}

  // Constructs intrinsic parameters from focal length, sensor cell size and
  // the principal point.
  //
  // The focal length and the sensor cell size must be specified in the same
  // metric units. The sensor cell size is represented by sx_units_per_px, i.e.
  // the sensor cell width and sy_units_per_px, the cell's height.
  IntrinsicParams(const Dimensions& dimensions, double focal_length_units,
                  double sx_units_per_px, double sy_units_per_px, double cx_px,
                  double cy_px)
      : dimensions_(dimensions),
        f_(focal_length_units),
        sx_(sx_units_per_px),
        sy_(sy_units_per_px),
        cx_(cx_px),
        cy_(cy_px) {}

  // Constructs intrinsic parameters from focal length and principal point.
  // All parameters must be specified in pixels.
  IntrinsicParams(const Dimensions& dimensions, double focal_length_x,
                  double focal_length_y, double principal_point_x,
                  double principal_point_y)
      : dimensions_(dimensions),
        f_(focal_length_x),
        sx_(1),
        sy_(focal_length_x / focal_length_y),
        cx_(principal_point_x),
        cy_(principal_point_y) {}

  Matrix3d matrix() const {
    return (Matrix3d() << f_ / sx_, 0, cx_, 0, f_ / sy_, cy_, 0, 0, 1)
        .finished();
  }

  Matrix3d inverse_matrix() const {
    const double f_inv = 1.0 / f_;
    const double fx_inv = sx_ * f_inv;
    const double fy_inv = sy_ * f_inv;
    return (Matrix3d() << fx_inv, 0, -cx_ * fx_inv, 0, fy_inv, -cy_ * fy_inv, 0,
            0, 1)
        .finished();
  }

  const Dimensions& dimensions() const { return dimensions_; }

  // Returns the focal length.
  //
  // In case the class has been constructed from focal length and principal
  // point only, i.e. without the sensor size information, the focal length will
  // be identical to the user specified horizonal focal length (f_ / sx_).
  //
  // The unit of the focal length is either the metric units used on the ctor
  // for metric construction or it is equal to pixels, when the class is
  // constructed without specifying the sensor cell size.
  ABSL_DEPRECATED("Use focal_length_x() and focal_length_y() instead.")
  double focal_length() const { return f_; }

  // Returns the horizontal focal length in pixels.
  double focal_length_x() const { return f_ / sx_; }

  // Returns the vertical focal length in pixels.
  double focal_length_y() const { return f_ / sy_; }

  // Returns the size of a censor cell in meters.
  ABSL_DEPRECATED(
      "The physical sensor size should not be needed. Rely on the values saved "
      "in the camera matrix through matrix()")
  Vector2d sensor_cell_size() const { return {sx_, sy_}; }

  // Returns the horizontal size of a sensor cell (or pixel) in meters.
  ABSL_DEPRECATED(
      "The physical sensor size should not be needed. Rely on the values saved "
      "in the camera matrix through matrix()")
  double sensor_cell_size_x() const { return sx_; }

  // Returns the vertical size of a sensor cell (or pixel) in meters.
  ABSL_DEPRECATED(
      "The physical sensor size should not be needed. Rely on the values saved "
      "in the camera matrix through matrix()")
  double sensor_cell_size_y() const { return sy_; }

  // Returns the location of the principal point in pixels.
  Vector2d principal_point() const { return {cx_, cy_}; }

  // Returns the principal point's x-offset from the top-left of the image in
  // pixels.
  double principal_point_x() const { return cx_; }

  // Returns the principal point's x-offset from the top-left of the image in
  // pixels.
  double principal_point_y() const { return cy_; }

 private:
  // The dimensions of the image plane.
  Dimensions dimensions_;

  // The focal length.
  double f_;
  // The width of the camera's sensor cell.
  double sx_;
  // The height of the camera's sensor cell.
  double sy_;
  // The x-position of the principal point.
  double cx_;
  // The y-position of the principal point.
  double cy_;
};

// Returns intrinsic parameters of a cropped region.
absl::StatusOr<IntrinsicParams> Crop(const IntrinsicParams& intrinsic_params,
                                     const Rectangle& roi,
                                     bool allow_out_of_bounds = false);

// Returns intrinsic parameters for a scaled image.
IntrinsicParams Resize(const IntrinsicParams& intrinsic_params,
                       Dimensions size);

// Returns intrinsic parameters corresponding to an image scaled in x and y
// directions.
IntrinsicParams Resize(const IntrinsicParams& intrinsic_params, float scale_x,
                       float scale_y);

// Pads the intrinsic parameters. This only affects the image dimensions and
// principal point, but not the focal length.
IntrinsicParams Padding(const IntrinsicParams& intrinsic_params, int padding);

// Returns intrinsic parameters with the new dimensions. The focal length and
// principal point are left unchanged.
IntrinsicParams ReplaceDimensions(const IntrinsicParams& intrinsics,
                                  const Dimensions& new_dimensions);

// Options to specify which intrinsic parameters are validated.
struct IntrinsicParamsCheckOptions {
  bool check_dimensions = true;
  bool check_focal_length = true;
  bool check_principal_point = true;
};

// Returns an OkStatus() if the observed and expected camera parameters are
// close to each other according to the provided check options.
absl::Status IntrinsicParamsNear(const IntrinsicParams& observed,
                                 const IntrinsicParams& expected, double eps,
                                 IntrinsicParamsCheckOptions check_options);

// Returns an OkStatus() if the observed and expected camera parameters are
// close to each other.
// In order to be close to each other, camera dimensions of the input parameters
// have to be identical and the focal length and principal point values have to
// be close to each other up to a user specified 'eps'. Since the focal length
// and principal point values are specified in pixels, the 'eps' parameter
// specifies a threshold in pixels, too. The default value is 1/1000 th of a
// pixel.
absl::Status IntrinsicParamsNear(
    const IntrinsicParams& observed, const IntrinsicParams& expected,
    double eps = 1e-3, bool check_focal_length_and_principal_point = true);

// Returns the projected 3D point. Pre-condition: point_optical.z() != 0
Vector2f ProjectToImage(const IntrinsicParams& intrinsics,
                        const Vector3f& point_optical);

// Returns the projected 3D point. Pre-condition:
// (optical_t_ref * point_ref).z() ≠ 0
Vector2f ProjectToImage(const IntrinsicParams& intrinsics,
                        const Isometry3f& optical_t_ref,
                        const Vector3f& point_ref);

// Returns the orthographic projection of the 3D point.
//
// The orthographic projection to image plane can be derived from combining the
// OpenGL orthographic projection matrix with the view transform, see
// https://www.songho.ca/opengl/gl_projectionmatrix.html#ortho. Note,
// IntrinsicParams::focal_length_x is the ratio between the width of the sensor
// in pixels to the width of the sensor in meters.
// IntrinsicParams::focal_length_x is the ratio between the height of the sensor
// in pixels to the height of the sensor in meters.
Vector2f ProjectToImageOrthographic(const IntrinsicParams& intrinsics,
                                    const Vector3f& point_optical);

// Returns projected points. Pre-condition: point_optical.z() ≠ 0.
std::vector<Vector2f> ProjectPointsToImage(
    const IntrinsicParams& intrinsics,
    const std::vector<Vector3f>& points_optical);

// Returns projected points. Pre-condition: (optical_t_ref * point_ref).z() ≠ 0.
std::vector<Vector2f> ProjectPointsToImage(
    const IntrinsicParams& intrinsics, const Isometry3f& optical_t_ref,
    const std::vector<Vector3f>& points_ref);

// Projects pinhole measurement into the optical frame of the camera.
Vector3f ProjectToOptical(const IntrinsicParams& intrinsics,
                          const Vector2f& uv);

// Projects pinhole measurement into the camera frame.
Vector3f ProjectToOptical(const IntrinsicParams& intrinsics, const Vector2f& uv,
                          float depth);

// Prints intrinsic params to the specified output stream.
void PrintTo(const IntrinsicParams& intrinsics, std::ostream* os);

std::ostream& operator<<(std::ostream& os, const IntrinsicParams& intrinsics);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_INTRINSIC_PARAMS_H_
