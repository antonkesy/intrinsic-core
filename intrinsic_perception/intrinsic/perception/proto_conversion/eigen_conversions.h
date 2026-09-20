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

#ifndef INTRINSIC_PERCEPTION_PROTO_CONVERSION_EIGEN_CONVERSIONS_H_
#define INTRINSIC_PERCEPTION_PROTO_CONVERSION_EIGEN_CONVERSIONS_H_

#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto/pose.pb.h"
#include "intrinsic/perception/core/eigen_types.h"

namespace intrinsic {
namespace perception {

intrinsic_proto::Pose FromEigen(const Eigen::Isometry3d& pose);

Eigen::Isometry3d ToEigen(const intrinsic_proto::Pose& pose);

template <typename Scalar, int Options>
Isometry3<Scalar> ToEigen(const Pose3<Scalar, Options>& pose) {
  Isometry3<Scalar> result = Isometry3<Scalar>::Identity();
  result.linear() = pose.rotationMatrix();
  result.translation() = pose.translation();
  return result;
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_PROTO_CONVERSION_EIGEN_CONVERSIONS_H_
