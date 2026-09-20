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

#include "intrinsic/perception/proto_conversion/eigen_conversions.h"

#include "absl/log/check.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto/pose.pb.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/perception/core/eigen_types.h"

namespace intrinsic {
namespace perception {

intrinsic_proto::Pose FromEigen(const Eigen::Isometry3d& pose) {
  Pose3d pose_3d(Eigen::Matrix3d(pose.matrix().topLeftCorner(3, 3)));
  pose_3d.translation() = pose.translation().head(3);
  return ToProto(pose_3d);
}

Eigen::Isometry3d ToEigen(const intrinsic_proto::Pose& pose) {
  auto pose_3d_converted = FromProto(pose);
  CHECK_OK(pose_3d_converted.status());
  return Eigen::Isometry3d(pose_3d_converted->matrix());
}

}  // namespace perception
}  // namespace intrinsic
