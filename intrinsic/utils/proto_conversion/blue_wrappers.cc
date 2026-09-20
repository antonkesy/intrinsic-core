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

#include "intrinsic/utils/proto_conversion/blue_wrappers.h"

#include <cmath>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/math/pose3.h"

namespace blue {

namespace messages_proto {

absl::StatusOr<intrinsic::eigenmath::Pose2d> FromProto(const Pose2D& p) {
  return intrinsic::eigenmath::Pose2d{{p.position().x(), p.position().y()},
                                      p.orientation()};
}

absl::StatusOr<intrinsic::Pose3d> FromProto(const Pose3D& p) {
  intrinsic::eigenmath::Vector3d trans;
  intrinsic::eigenmath::Quaterniond rot;
  trans.x() = p.position().x();
  trans.y() = p.position().y();
  trans.z() = p.position().z();
  if (p.has_orientation()) {
    rot.x() = p.orientation().x();
    rot.y() = p.orientation().y();
    rot.z() = p.orientation().z();
    rot.w() = p.orientation().w();

    // We don't normalize because that is not the original behavior of this
    // function and would break the contract. There isn't a reason why it cannot
    // be done, aside from it's not worth the refactoring effort. Instead please
    // consider using the proto from the math folder. The new proto already has
    // both checks in place.
    double sqNorm = rot.squaredNorm();
    if (!std::isfinite(sqNorm) || !(sqNorm > 0)) {
      return absl::InvalidArgumentError("Quaternion does not have valid norm.");
    }
  } else {
    rot = intrinsic::eigenmath::Quaterniond::Identity();
  }

  return intrinsic::Pose3d{rot, trans};
}

absl::StatusOr<intrinsic::eigenmath::Vector3d> FromProto(const Vec3& v) {
  intrinsic::eigenmath::Vector3d result;
  result[0] = v.x();
  result[1] = v.y();
  result[2] = v.z();
  return result;
}

absl::StatusOr<intrinsic::eigenmath::VectorXd> FromProto(const VectorXd& v) {
  intrinsic::eigenmath::VectorXd result;
  result.resize(v.rows());

  if (v.rows() != result.rows() || v.values().size() != result.rows()) {
    return absl::InvalidArgumentError("Couldn't convert proto to native.");
  }
  for (int i = 0; i < result.rows(); ++i) {
    result(i) = v.values(i);
  }
  return result;
}

absl::StatusOr<intrinsic::eigenmath::MatrixXd> FromProto(const Matrixd& m) {
  constexpr int kMaxMatrixDimension = 2048;
  if (m.rows() <= 0 || m.rows() > kMaxMatrixDimension || m.cols() <= 0 ||
      m.cols() > kMaxMatrixDimension) {
    return absl::InvalidArgumentError("Matrix dimension exceeds max of 2048");
  }

  intrinsic::eigenmath::MatrixXd result;
  result.resize(m.rows(), m.cols());
  if (m.rows() != result.rows() || m.cols() != result.cols() ||
      m.values().size() != result.rows() * result.cols()) {
    return absl::InvalidArgumentError("Couldn't convert proto to native.");
  }
  for (int j = 0; j < result.cols(); ++j) {
    for (int i = 0; i < result.rows(); ++i) {
      result(i, j) = m.values(i + j * result.rows());
    }
  }
  return result;
}

}  // namespace messages_proto
}  // namespace blue

namespace blue {
namespace eigenmath {

absl::StatusOr<blue::messages_proto::Pose2D> ToProto(
    const intrinsic::eigenmath::Pose2d& p) {
  blue::messages_proto::Pose2D result;
  result.set_orientation(p.angle());
  result.mutable_position()->set_x(p.translation().x());
  result.mutable_position()->set_y(p.translation().y());
  return result;
}

absl::StatusOr<blue::messages_proto::Pose3D> ToProto(
    const intrinsic::Pose3d& p) {
  blue::messages_proto::Pose3D result;
  result.mutable_position()->set_x(p.translation().x());
  result.mutable_position()->set_y(p.translation().y());
  result.mutable_position()->set_z(p.translation().z());
  result.mutable_orientation()->set_x(p.quaternion().x());
  result.mutable_orientation()->set_y(p.quaternion().y());
  result.mutable_orientation()->set_z(p.quaternion().z());
  result.mutable_orientation()->set_w(p.quaternion().w());
  return result;
}

}  // namespace eigenmath
}  // namespace blue

namespace Eigen {

absl::StatusOr<blue::messages_proto::Vec3> ToProto(
    const intrinsic::eigenmath::Vector3d& v) {
  blue::messages_proto::Vec3 result;
  result.set_x(v[0]);
  result.set_y(v[1]);
  result.set_z(v[2]);
  return result;
}

absl::StatusOr<blue::messages_proto::VectorXd> ToProto(
    const intrinsic::eigenmath::VectorXd& v) {
  blue::messages_proto::VectorXd result;
  result.set_rows(v.rows());
  result.mutable_values()->Clear();
  result.mutable_values()->Reserve(v.rows());
  for (int i = 0; i < v.rows(); ++i) {
    result.mutable_values()->AddAlreadyReserved(v(i));
  }
  return result;
}

namespace {
template <typename MatrixType>
absl::StatusOr<blue::messages_proto::Matrixd> ToProtoImpl(const MatrixType& m) {
  blue::messages_proto::Matrixd result;
  result.set_rows(m.rows());
  result.set_cols(m.cols());
  result.mutable_values()->Clear();
  result.mutable_values()->Reserve(m.rows() * m.cols());
  for (int j = 0; j < m.cols(); ++j) {
    for (int i = 0; i < m.rows(); ++i) {
      result.mutable_values()->AddAlreadyReserved(m(i, j));
    }
  }
  return result;
}
}  // namespace

absl::StatusOr<blue::messages_proto::Matrixd> ToProto(
    const intrinsic::eigenmath::Matrix3d& m) {
  return ToProtoImpl(m);
}

absl::StatusOr<blue::messages_proto::Matrixd> ToProto(
    const intrinsic::eigenmath::MatrixXd& m) {
  return ToProtoImpl(m);
}

absl::StatusOr<blue::messages_proto::Matrixd> ToProto(
    const intrinsic::eigenmath::Matrix6d& m) {
  return ToProtoImpl(m);
}

}  // namespace Eigen
