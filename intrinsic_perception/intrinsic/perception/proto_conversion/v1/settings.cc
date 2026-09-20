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

#include "intrinsic/perception/proto_conversion/v1/settings.h"

#include <optional>
#include <string>
#include <vector>

#include "intrinsic/perception/cameras/camera_setting_properties.h"
#include "intrinsic/perception/proto/v1/settings.pb.h"

namespace intrinsic_proto::perception::v1 {

intrinsic::perception::CameraSettingProperties::Float FromProto(
    const FloatSettingProperties& float_properties) {
  return intrinsic::perception::CameraSettingProperties::Float{
      .range = float_properties.has_range()
                   ? std::optional<intrinsic::perception::
                                       CameraSettingProperties::Float::Range>({
                         .minimum = float_properties.range().minimum(),
                         .maximum = float_properties.range().maximum(),
                     })
                   : std::nullopt,
      .increment = float_properties.increment(),
      .unit = float_properties.has_unit()
                  ? std::optional<std::string>(float_properties.unit())
                  : std::nullopt,
  };
}

FloatSettingProperties ToProto(
    const intrinsic::perception::CameraSettingProperties::Float&
        float_properties) {
  FloatSettingProperties proto;
  if (float_properties.range.has_value()) {
    proto.mutable_range()->set_minimum(float_properties.range->minimum);
    proto.mutable_range()->set_maximum(float_properties.range->maximum);
  }
  proto.set_increment(float_properties.increment);
  if (float_properties.unit.has_value()) {
    proto.set_unit(*float_properties.unit);
  }
  return proto;
}

intrinsic::perception::CameraSettingProperties::Integer FromProto(
    const IntegerSettingProperties& integer_properties) {
  return intrinsic::perception::CameraSettingProperties::Integer{
      .range =
          integer_properties.has_range()
              ? std::optional<intrinsic::perception::CameraSettingProperties::
                                  Integer::Range>({
                    .minimum = integer_properties.range().minimum(),
                    .maximum = integer_properties.range().maximum(),
                })
              : std::nullopt,
      .increment = integer_properties.increment(),
      .unit = integer_properties.has_unit()
                  ? std::optional<std::string>(integer_properties.unit())
                  : std::nullopt,
  };
}

IntegerSettingProperties ToProto(
    const intrinsic::perception::CameraSettingProperties::Integer&
        integer_properties) {
  IntegerSettingProperties proto;
  if (integer_properties.range.has_value()) {
    proto.mutable_range()->set_minimum(integer_properties.range->minimum);
    proto.mutable_range()->set_maximum(integer_properties.range->maximum);
  }
  proto.set_increment(integer_properties.increment);
  if (integer_properties.unit.has_value()) {
    proto.set_unit(*integer_properties.unit);
  }
  return proto;
}

intrinsic::perception::CameraSettingProperties::Enumeration FromProto(
    const EnumSettingProperties& enumeration_properties) {
  return intrinsic::perception::CameraSettingProperties::Enumeration{
      .values =
          std::vector<std::string>(enumeration_properties.values().begin(),
                                   enumeration_properties.values().end())};
}

EnumSettingProperties ToProto(
    const intrinsic::perception::CameraSettingProperties::Enumeration&
        enumeration_properties) {
  EnumSettingProperties proto;
  proto.mutable_values()->Add(enumeration_properties.values.begin(),
                              enumeration_properties.values.end());
  return proto;
}

}  // namespace intrinsic_proto::perception::v1
