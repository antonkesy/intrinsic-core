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

#ifndef INTRINSIC_PERCEPTION_CORE_IMAGE_PUBLISHER_H_
#define INTRINSIC_PERCEPTION_CORE_IMAGE_PUBLISHER_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/perception/proto/v1/image_buffer.pb.h"

namespace intrinsic {
namespace perception {

// Publishes an image buffer over PubSub to the specified topic.
// Publishers are internally cached per topic.
absl::Status PublishImage(
    const intrinsic_proto::perception::v1::ImageBuffer& image,
    absl::string_view topic);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_IMAGE_PUBLISHER_H_
