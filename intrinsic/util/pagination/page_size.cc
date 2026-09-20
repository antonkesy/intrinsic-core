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

#include "intrinsic/util/pagination/page_size.h"

#include <algorithm>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace intrinsic {

PageSizeValidator::PageSizeValidator(int default_size, int max_size)
    : default_size_(default_size), max_size_(max_size) {}

absl::StatusOr<int> PageSizeValidator::Run(int page_size) const {
  if (page_size == 0) {
    return std::min(default_size_, max_size_);
  }
  if (page_size < 0) {
    return absl::InvalidArgumentError("Page size must be non-negative.");
  }
  if (page_size > max_size_) {
    return max_size_;
  }
  return page_size;
}

}  // namespace intrinsic
