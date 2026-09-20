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

#include "intrinsic/perception/proto_conversion/v1/image_file_reference.h"

#include <optional>

#include "intrinsic/perception/core/image_file_reference.h"
#include "intrinsic/perception/proto/v1/image_file_reference.pb.h"
#include "intrinsic/perception/proto_conversion/v1/pixel_type.h"

namespace intrinsic_proto::perception::v1 {

intrinsic::perception::ImageFileReference FromProto(
    const ImageFileReference& image_file_reference) {
  return intrinsic::perception::ImageFileReference{
      .path = image_file_reference.path(),
      .pixel_type = FromProto(image_file_reference.pixel_type()),
      .num_channels = image_file_reference.num_channels(),
      .sensor_id = image_file_reference.has_sensor_id()
                       ? std::make_optional(image_file_reference.sensor_id())
                       : std::nullopt,
  };
}

ImageFileReference ToProto(
    const intrinsic::perception::ImageFileReference& image_file_reference) {
  ImageFileReference image_file_reference_proto;
  image_file_reference_proto.set_path(image_file_reference.path);
  image_file_reference_proto.set_pixel_type(
      ToProto(image_file_reference.pixel_type));
  image_file_reference_proto.set_num_channels(
      image_file_reference.num_channels);
  if (image_file_reference.sensor_id.has_value()) {
    image_file_reference_proto.set_sensor_id(*image_file_reference.sensor_id);
  }
  return image_file_reference_proto;
}

}  // namespace intrinsic_proto::perception::v1
