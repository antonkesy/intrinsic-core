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

#ifndef INTRINSIC_PERCEPTION_CORE_PIXEL_TYPE_H_
#define INTRINSIC_PERCEPTION_CORE_PIXEL_TYPE_H_

#include "magic_enum/magic_enum.hpp"

namespace intrinsic::perception {

enum class PixelType {
  kUnspecified,
  kIntensity,
  kDepth,
  kPoint,
  kNormal,
};

template <typename Sink>
void AbslStringify(Sink& sink, PixelType p) {
  sink.Append(magic_enum::enum_name(p));
}

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CORE_PIXEL_TYPE_H_
