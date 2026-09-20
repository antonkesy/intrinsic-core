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

#ifndef INTRINSIC_PERCEPTION_PROTO_CONVERSION_V1_PIXEL_TYPE_H_
#define INTRINSIC_PERCEPTION_PROTO_CONVERSION_V1_PIXEL_TYPE_H_

#include "intrinsic/perception/core/pixel_type.h"
#include "intrinsic/perception/proto/v1/image_buffer.pb.h"

namespace intrinsic_proto::perception::v1 {

intrinsic::perception::PixelType FromProto(PixelType pixel_type);

PixelType ToProto(intrinsic::perception::PixelType pixel_type);

}  // namespace intrinsic_proto::perception::v1

#endif  // INTRINSIC_PERCEPTION_PROTO_CONVERSION_V1_PIXEL_TYPE_H_
