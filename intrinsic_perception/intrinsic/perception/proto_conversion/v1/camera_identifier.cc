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

#include "intrinsic/perception/proto_conversion/v1/camera_identifier.h"

#include <utility>
#include <variant>

#include "absl/functional/overload.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/proto/v1/camera_identifier.pb.h"
#include "intrinsic/perception/proto_conversion/v1/camera_drivers.h"

namespace intrinsic_proto::perception::v1 {

intrinsic::perception::CameraIdentifier FromProto(
    const CameraIdentifier& camera_identifier) {
  intrinsic::perception::CameraIdentifier::Driver driver = std::monostate();
  if (camera_identifier.has_genicam()) {
    driver = FromProto(camera_identifier.genicam());
  } else if (camera_identifier.has_ros()) {
    driver = FromProto(camera_identifier.ros());
  } else if (camera_identifier.has_file_camera()) {
    driver = FromProto(camera_identifier.file_camera());
  } else if (camera_identifier.has_fake_genicam()) {
    driver = FromProto(camera_identifier.fake_genicam());
  }
  return intrinsic::perception::CameraIdentifier{
      .driver = std::move(driver),
  };
}

CameraIdentifier ToProto(
    const intrinsic::perception::CameraIdentifier& camera_identifier) {
  CameraIdentifier proto;
  std::visit(
      absl::Overload{
          [](const std::monostate&) {},
          [&proto](
              const intrinsic::perception::CameraIdentifier::GenICam& driver) {
            *proto.mutable_genicam() = ToProto(driver);
          },
          [&proto](const intrinsic::perception::CameraIdentifier::Ros& driver) {
            *proto.mutable_ros() = ToProto(driver);
          },
          [&proto](const intrinsic::perception::CameraIdentifier::FileCamera&
                       driver) {
            *proto.mutable_file_camera() = ToProto(driver);
          },
          [&proto](const intrinsic::perception::CameraIdentifier::FakeGenICam&
                       driver) {
            *proto.mutable_fake_genicam() = ToProto(driver);
          }
      },
      camera_identifier.driver);
  return proto;
}

}  // namespace intrinsic_proto::perception::v1
