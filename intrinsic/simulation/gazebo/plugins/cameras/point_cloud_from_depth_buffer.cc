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

#include "intrinsic/simulation/gazebo/plugins/cameras/point_cloud_from_depth_buffer.h"

#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/intrinsic_params.h"

namespace intrinsic {
namespace simulation {

perception::Image<perception::Point32f> ComputePointCloudFromDepthBuffer(
    const perception::IntrinsicParams& intrinsic_params,
    const perception::Dimensions& dim,
    const perception::Depth32f::PixelType* depth_buffer) {
  perception::Image<perception::Point32f> points_image(dim);
  for (int r = 0; r < dim.rows; r++) {
    const int row_offset = r * dim.cols;
    for (int c = 0; c < dim.cols; c++) {
      points_image(c, r) = perception::ProjectToOptical(
          intrinsic_params, {c, r}, depth_buffer[row_offset + c]);
    }
  }
  return points_image;
}

}  // namespace simulation
}  // namespace intrinsic
