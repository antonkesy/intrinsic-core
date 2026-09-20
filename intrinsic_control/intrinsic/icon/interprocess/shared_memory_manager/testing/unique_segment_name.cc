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

#include "intrinsic/icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"

#include <cstdint>
#include <string>

#include "absl/random/distributions.h"
#include "absl/random/random.h"
#include "absl/strings/str_cat.h"

namespace intrinsic::icon {

std::string UniqueHardwareModuleName() {
  return absl::StrCat(absl::Hex(absl::Uniform<uint64_t>(absl::BitGen())));
}

std::string UniqueMemoryNamespace() {
  return absl::StrCat(absl::Hex(absl::Uniform<uint64_t>(absl::BitGen())));
}

}  // namespace intrinsic::icon
