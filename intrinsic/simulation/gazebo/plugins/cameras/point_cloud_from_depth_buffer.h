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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_POINT_CLOUD_FROM_DEPTH_BUFFER_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_POINT_CLOUD_FROM_DEPTH_BUFFER_H_

#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/intrinsic_params.h"

namespace intrinsic {
namespace simulation {

// Computes a 3D point cloud image from a float depth buffer using camera
// intrinsics. WARNING: Does not check the underlying `depth_buffer` dimensions
// for performance.
perception::Image<perception::Point32f> ComputePointCloudFromDepthBuffer(
    const perception::IntrinsicParams& intrinsic_params,
    const perception::Dimensions& dim,
    const perception::Depth32f::PixelType* depth_buffer);

}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_CAMERAS_POINT_CLOUD_FROM_DEPTH_BUFFER_H_
