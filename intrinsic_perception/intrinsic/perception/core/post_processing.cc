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

#include "intrinsic/perception/core/post_processing.h"

#include <optional>
#include <ostream>

namespace intrinsic::perception {

std::ostream& operator<<(std::ostream& os, const PostProcessing& p) {
  const auto print_optional = [&os]<typename T>(std::optional<T> t) {
    if (t.has_value()) {
      os << t.value();
    } else {
      os << "nullopt";
    }
  };

  os << "(crop_region: ";
  print_optional(p.crop_region);
  os << ", cols: ";
  print_optional(p.cols);
  os << ", rows: ";
  print_optional(p.rows);
  os << ", skip_undistortion: " << p.skip_undistortion << ")";
  return os;
}

}  // namespace intrinsic::perception
