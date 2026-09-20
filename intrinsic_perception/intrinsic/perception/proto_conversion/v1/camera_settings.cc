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

#include "intrinsic/perception/proto_conversion/v1/camera_settings.h"

#include <string>
#include <utility>
#include <variant>

#include "absl/functional/overload.h"
#include "google/protobuf/empty.pb.h"
#include "intrinsic/perception/cameras/camera_setting.h"
#include "intrinsic/perception/cameras/camera_setting_access.h"
#include "intrinsic/perception/cameras/camera_setting_properties.h"
#include "intrinsic/perception/proto/v1/camera_settings.pb.h"
#include "intrinsic/perception/proto/v1/settings.pb.h"
#include "intrinsic/perception/proto_conversion/v1/settings.h"

namespace intrinsic_proto::perception::v1 {

intrinsic::perception::CameraSetting FromProto(
    const CameraSetting& camera_setting) {
  intrinsic::perception::CameraSetting::Value value = std::monostate();
  if (camera_setting.has_integer_value()) {
    value = intrinsic::perception::CameraSetting::Integer(
        camera_setting.integer_value());
  } else if (camera_setting.has_float_value()) {
    value = intrinsic::perception::CameraSetting::Float(
        camera_setting.float_value());
  } else if (camera_setting.has_bool_value()) {
    value = intrinsic::perception::CameraSetting::Boolean(
        camera_setting.bool_value());
  } else if (camera_setting.has_string_value()) {
    value = intrinsic::perception::CameraSetting::String(
        camera_setting.string_value());
  } else if (camera_setting.has_enumeration_value()) {
    value = intrinsic::perception::CameraSetting::Enumeration{
        .value = camera_setting.enumeration_value()};
  } else if (camera_setting.has_command_value()) {
    value = intrinsic::perception::CameraSetting::Command();
  }
  return intrinsic::perception::CameraSetting{
      .name = camera_setting.name(),
      .value = std::move(value),
  };
}

CameraSetting ToProto(
    const intrinsic::perception::CameraSetting& camera_setting) {
  CameraSetting proto;
  proto.set_name(camera_setting.name);
  std::visit(
      absl::Overload{
          [](const std::monostate&) {},
          [&proto](const intrinsic::perception::CameraSetting::Integer& value) {
            proto.set_integer_value(value.value);
          },
          [&proto](const intrinsic::perception::CameraSetting::Float& value) {
            proto.set_float_value(value.value);
          },
          [&proto](const intrinsic::perception::CameraSetting::Boolean& value) {
            proto.set_bool_value(value.value);
          },
          [&proto](const intrinsic::perception::CameraSetting::String& value) {
            proto.set_string_value(value.value);
          },
          [&proto](
              const intrinsic::perception::CameraSetting::Enumeration& value) {
            proto.set_enumeration_value(value.value);
          },
          [&proto](const intrinsic::perception::CameraSetting::Command&) {
            *proto.mutable_command_value() = google::protobuf::Empty();
          }},
      camera_setting.value);
  return proto;
}

intrinsic::perception::CameraSettingAccess FromProto(
    const CameraSettingAccess& camera_setting_access) {
  return intrinsic::perception::CameraSettingAccess{
      .name = camera_setting_access.name(),
      .mode = intrinsic::perception::CameraSettingAccess::Mode(
          camera_setting_access.mode()),
  };
}

CameraSettingAccess ToProto(
    const intrinsic::perception::CameraSettingAccess& camera_setting_access) {
  CameraSettingAccess proto;
  proto.set_name(camera_setting_access.name);
  proto.set_mode(CameraSettingAccess::Mode(camera_setting_access.mode));
  return proto;
}

intrinsic::perception::CameraSettingProperties FromProto(
    const CameraSettingProperties& camera_setting_properties) {
  intrinsic::perception::CameraSettingProperties::Properties properties =
      std::monostate();
  if (camera_setting_properties.has_float_properties()) {
    properties = FromProto(camera_setting_properties.float_properties());
  } else if (camera_setting_properties.has_integer_properties()) {
    properties = FromProto(camera_setting_properties.integer_properties());
  } else if (camera_setting_properties.has_enum_properties()) {
    properties = FromProto(camera_setting_properties.enum_properties());
  }
  return intrinsic::perception::CameraSettingProperties{
      .name = camera_setting_properties.name(),
      .properties = std::move(properties),
  };
}

CameraSettingProperties ToProto(
    const intrinsic::perception::CameraSettingProperties&
        camera_setting_properties) {
  CameraSettingProperties proto;
  proto.set_name(camera_setting_properties.name);
  std::visit(
      absl::Overload{
          [](const std::monostate&) {},
          [&proto](const intrinsic::perception::CameraSettingProperties::Float&
                       float_properties) {
            *proto.mutable_float_properties() = ToProto(float_properties);
          },
          [&proto](
              const intrinsic::perception::CameraSettingProperties::Integer&
                  integer_properties) {
            *proto.mutable_integer_properties() = ToProto(integer_properties);
          },
          [&proto](
              const intrinsic::perception::CameraSettingProperties::Enumeration&
                  enum_properties) {
            *proto.mutable_enum_properties() = ToProto(enum_properties);
          },
      },
      camera_setting_properties.properties);
  return proto;
}

}  // namespace intrinsic_proto::perception::v1
