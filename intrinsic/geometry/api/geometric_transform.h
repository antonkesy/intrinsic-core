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

#ifndef GEOMETRY_API_GEOMETRIC_TRANSFORM_H_
#define GEOMETRY_API_GEOMETRIC_TRANSFORM_H_

#include <utility>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/proto/v1/geometric_transform.pb.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::geo {
/**
 * Extracts a pose and a scale from a GeometricTransform proto.
 *
 * This function supports both matrix4d and TRS representations in the proto.
 * For matrix4d, it attempts to decompose the affine matrix into a pose and a
 * scale vector. For TRS, it directly constructs the pose from translation and
 * rotation, and extracts the scale vector.
 *
 * @param gt The GeometricTransform proto to extract from.
 * @return A pair containing the Pose3d and the eigenmath::Vector3d scale, or
 * an error status if decomposition fails (e.g., reflection or skew).
 */
absl::StatusOr<std::pair<Pose3d, eigenmath::Vector3d>> GetPoseAndScale(
    const intrinsic_proto::geometry::v1::GeometricTransform& gt);

}  // namespace intrinsic::geo
#endif  // GEOMETRY_API_GEOMETRIC_TRANSFORM_H_
