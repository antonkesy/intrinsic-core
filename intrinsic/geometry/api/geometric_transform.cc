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

#include "intrinsic/geometry/api/geometric_transform.h"

#include <utility>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/rotation_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/proto/v1/geometric_transform.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::geo {
absl::StatusOr<std::pair<Pose3d, eigenmath::Vector3d>> GetPoseAndScale(
    const intrinsic_proto::geometry::v1::GeometricTransform& gt) {
  if (gt.has_matrix4d()) {
    INTR_ASSIGN_OR_RETURN(eigenmath::MatrixXd m_any,
                          intrinsic_proto::FromProto(gt.matrix4d()));
    INTR_ASSIGN_OR_RETURN(eigenmath::Matrix4d m, toAffineMatrix4d(m_any));
    return matrixToPoseAndScale(m);
  } else if (gt.has_trs()) {
    eigenmath::Vector3d t = eigenmath::Vector3d::Zero();
    if (gt.trs().has_translation()) {
      t = intrinsic_proto::FromProto(gt.trs().translation());
    }
    double r = 0, p = 0, y = 0;
    if (gt.trs().has_rotation_rpy()) {
      r = gt.trs().rotation_rpy().r();
      p = gt.trs().rotation_rpy().p();
      y = gt.trs().rotation_rpy().y();
    }
    auto q = eigenmath::RotationFromRPY<eigenmath::Quaterniond>(r, p, y);

    eigenmath::Vector3d scale = eigenmath::Vector3d::Ones();
    if (gt.trs().has_scale()) {
      scale = intrinsic_proto::FromProto(gt.trs().scale());
    }

    return std::make_pair(Pose3d(q, t), scale);
  }
  return std::make_pair(Pose3d::Identity(), eigenmath::Vector3d::Ones());
}

}  // namespace intrinsic::geo
