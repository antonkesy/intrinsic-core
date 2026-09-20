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

#ifndef INTRINSIC_MATH_TF2_CONVERT_INTRINSIC_H_
#define INTRINSIC_MATH_TF2_CONVERT_INTRINSIC_H_

#include "absl/log/log.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "google/protobuf/timestamp.pb.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto/header.pb.h"
#include "intrinsic/math/proto/quaternion.pb.h"
#include "intrinsic/math/proto/transform.pb.h"
#include "intrinsic/math/proto/transform_stamped.pb.h"
#include "intrinsic/math/proto/vector3.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace tf2 {

// Vector3

// Convert a tf2 Vector3 type to its equivalent geometry_msgs
// representation. This function is a specialization of the toMsg template
// defined in tf2/convert.h. \param in A tf2 Vector3 object. \return The Vector3
// converted to a geometry_msgs message type.
inline geometry_msgs::msg::Vector3 toMsg(
    const intrinsic::eigenmath::Vector3<double>& in) {
  geometry_msgs::msg::Vector3 out;
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  return out;
}

inline geometry_msgs::msg::Vector3 toMsg(const intrinsic_proto::Vector3& in) {
  geometry_msgs::msg::Vector3 out;
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  return out;
}

// Convert a Vector3 message to its equivalent tf2 representation.
// This function is a specialization of the fromMsg template defined in
// tf2/convert.h. \param in A Vector3 message type. \param out The Vector3
// converted to a tf2 type.
inline void fromMsg(const geometry_msgs::msg::Vector3& in,
                    intrinsic::eigenmath::Vector3<double>& out) {
  out = intrinsic::eigenmath::Vector3<double>(in.x, in.y, in.z);
}

// Point

// Convert a tf2 Point type to its equivalent geometry_msgs
// representation. This function is a specialization of the toMsg template
// defined in tf2/convert.h. \param in A intrinsic Vector object. \return The
// Point converted to a geometry_msgs message type.
inline geometry_msgs::msg::Point toMsg(
    const intrinsic::eigenmath::Vector3<double>& in,
    geometry_msgs::msg::Point& out) {
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  return out;
}

//  \brief Convert a Point message to its equivalent tf2 representation.
// This function is a specialization of the fromMsg template defined in
// tf2/convert.h. \param in A Point message type. \param out The Vector3
// converted to a tf2 type.
inline void fromMsg(const geometry_msgs::msg::Point& in,
                    intrinsic::eigenmath::Vector3<double>& out) {
  out = intrinsic::eigenmath::Vector3<double>(in.x, in.y, in.z);
}

// Quaternion

//  \brief Convert a tf2 Quaternion type to its equivalent geometry_msgs
// representation. This function is a specialization of the toMsg template
// defined in tf2/convert.h. \param in A tf2 Quaternion object. \return The
// Quaternion converted to a geometry_msgs message type.
inline geometry_msgs::msg::Quaternion toMsg(
    const intrinsic::eigenmath::Quaternion<double>& in) {
  geometry_msgs::msg::Quaternion out;
  out.w = in.w();
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  return out;
}

inline geometry_msgs::msg::Quaternion toMsg(
    const intrinsic_proto::Quaternion& in) {
  geometry_msgs::msg::Quaternion out;
  out.w = in.w();
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  return out;
}

//  \brief Convert a Quaternion message to its equivalent tf2 representation.
// This function is a specialization of the fromMsg template defined in
// tf2/convert.h. \param in A Quaternion message type. \param out The Quaternion
// converted to a tf2 type.
inline void fromMsg(const geometry_msgs::msg::Quaternion& in,
                    intrinsic::eigenmath::Quaternion<double>& out) {
  // w at the end in the constructor
  out = intrinsic::eigenmath::Quaternion<double>(in.w, in.x, in.y, in.z);
}

// Pose

// Convert a tf2 Transform type to an equivalent geometry_msgs Pose
// message. \param in A tf2 Transform object. \param out The Transform converted
// to a geometry_msgs Pose message type.
inline geometry_msgs::msg::Pose& toMsg(const intrinsic::Pose3d& in,
                                       geometry_msgs::msg::Pose& out) {
  toMsg(in.translation(), out.position);
  out.orientation = toMsg(in.quaternion());
  return out;
}

//  \brief Convert a geometry_msgs Pose message to an equivalent tf2 Transform
// type. \param in A Pose message. \param out The Pose converted to a tf2
// Transform type.
inline void fromMsg(const geometry_msgs::msg::Pose& in,
                    intrinsic::Pose3d& out) {
  out.translation() = intrinsic::eigenmath::Vector3<double>(
      in.position.x, in.position.y, in.position.z);
  // w at the end in the constructor
  out.setQuaternion(intrinsic::eigenmath::Quaternion<double>(
      in.orientation.w, in.orientation.x, in.orientation.y, in.orientation.z));
}

// Transform

//  \brief Convert a tf2 Transform type to its equivalent geometry_msgs
// representation. This function is a specialization of the toMsg template
// defined in tf2/convert.h. \param in A tf2 Transform object. \return The
// Transform converted to a geometry_msgs message type.
inline geometry_msgs::msg::Transform toMsg(const intrinsic::Pose3d& in) {
  geometry_msgs::msg::Transform out;
  out.translation = toMsg(in.translation());
  out.rotation = toMsg(in.quaternion());
  return out;
}

inline geometry_msgs::msg::Transform toMsg(
    const intrinsic_proto::Transform& in) {
  geometry_msgs::msg::Transform out;
  out.translation = toMsg(in.translation());
  if (in.has_rotation()) {
    out.rotation = toMsg(in.rotation());
  } else {
    out.rotation.w = 1.0;
    out.rotation.x = 0.0;
    out.rotation.y = 0.0;
    out.rotation.z = 0.0;
  }
  return out;
}

//  \brief Convert a Transform message to its equivalent tf2 representation.
// This function is a specialization of the toMsg template defined in
// tf2/convert.h. \param in A Transform message type. \param out The Transform
// converted to a tf2 type.
inline void fromMsg(const geometry_msgs::msg::Transform& in,
                    intrinsic::Pose3d& out) {
  intrinsic::eigenmath::Vector3<double> trans;
  tf2::fromMsg(in.translation, trans);
  out.translation() = trans;
  intrinsic::eigenmath::Quaternion<double> quat;
  if (in.rotation.w == 0.0 && in.rotation.x == 0.0 && in.rotation.y == 0.0 &&
      in.rotation.z == 0.0) {
    LOG(ERROR) << "Transform rotation is all zeros, using identity.";
    quat = intrinsic::eigenmath::Quaternion<double>::Identity();
  } else {
    tf2::fromMsg(in.rotation, quat);
  }
  out.setQuaternion(quat);
}

inline absl::StatusOr<intrinsic::Pose3d> fromMsg(
    const geometry_msgs::msg::Transform& in) {
  intrinsic::eigenmath::Vector3<double> trans;
  tf2::fromMsg(in.translation, trans);
  intrinsic::eigenmath::Quaternion<double> quat;
  tf2::fromMsg(in.rotation, quat);
  INTR_ASSIGN_OR_RETURN(
      intrinsic::eigenmath::SO3<double> rot,
      intrinsic::eigenmath::SO3<double>::FromQuaternion(quat));
  return intrinsic::Pose3d(rot, trans);
}

// TransformStamped

// \brief Convert an intrinsic_proto::TransformStamped to its equivalent
// geometry_msgs representation. This function is a specialization of the toMsg
// template defined in tf2/convert.h.
// \param in An intrinsic_proto::TransformStamped object.
// \return The TransformStamped converted to a geometry_msgs message type.
inline geometry_msgs::msg::TransformStamped toMsg(
    const intrinsic_proto::TransformStamped& in) {
  geometry_msgs::msg::TransformStamped out;
  out.header.frame_id = in.header().frame_id();
  out.header.stamp.sec = in.header().stamp().seconds();
  out.header.stamp.nanosec = in.header().stamp().nanos();
  out.child_frame_id = in.child_frame_id();
  out.transform = toMsg(in.transform());
  return out;
}

}  // namespace tf2

#endif  // INTRINSIC_MATH_TF2_CONVERT_INTRINSIC_H_
