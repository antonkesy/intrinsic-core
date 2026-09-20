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

// This file offers protobuf:native conversion routines, provided by
// blue/shared/messages/proto_utils according to our preferred conventions.
// See proto_conversion.h for details.
//
// We intentionally add those definitions to the respective namespace of their
// argument, so that argument-dependent lookup will work naturally.

#ifndef THIRD_PARTY_INTRINSIC_UTILS_PROTO_CONVERSION_BLUE_WRAPPERS_NO_USING_H_
#define THIRD_PARTY_INTRINSIC_UTILS_PROTO_CONVERSION_BLUE_WRAPPERS_NO_USING_H_

#include "intrinsic/eigenmath/pose2.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/messages/geometry_types.pb.h"
#include "intrinsic/utils/proto_conversion/proto_conversion.h"

namespace blue {

namespace messages_proto {

DECLARE_FROM_PROTO(intrinsic::eigenmath::Pose2d, Pose2D);
DECLARE_FROM_PROTO(intrinsic::Pose3d, Pose3D);
DECLARE_FROM_PROTO(intrinsic::eigenmath::Vector3d, Vec3);
DECLARE_FROM_PROTO(intrinsic::eigenmath::VectorXd, VectorXd);
DECLARE_FROM_PROTO(intrinsic::eigenmath::MatrixXd, Matrixd);

}  // namespace messages_proto

}  // namespace blue

namespace blue::eigenmath {

DECLARE_TO_PROTO(intrinsic::eigenmath::Pose2d, blue::messages_proto::Pose2D);
DECLARE_TO_PROTO(intrinsic::Pose3d, blue::messages_proto::Pose3D);

}  // namespace blue::eigenmath

namespace Eigen {

DECLARE_TO_PROTO(intrinsic::eigenmath::Vector3d, blue::messages_proto::Vec3);
DECLARE_TO_PROTO(intrinsic::eigenmath::VectorXd,
                 blue::messages_proto::VectorXd);
DECLARE_TO_PROTO(intrinsic::eigenmath::Matrix3d, blue::messages_proto::Matrixd);
DECLARE_TO_PROTO(intrinsic::eigenmath::MatrixXd, blue::messages_proto::Matrixd);
DECLARE_TO_PROTO(intrinsic::eigenmath::Matrix6d, blue::messages_proto::Matrixd);

}  // namespace Eigen

#endif  // THIRD_PARTY_INTRINSIC_UTILS_PROTO_CONVERSION_BLUE_WRAPPERS_NO_USING_H_
