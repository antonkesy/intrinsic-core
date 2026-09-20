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

#include "intrinsic/perception/cameras/camera_identifier.h"

#include <ostream>
#include <string>
#include <string_view>

#include "absl/strings/ascii.h"
#include "absl/strings/str_cat.h"

namespace intrinsic::perception {

namespace {

std::string Slugify(std::string_view input) {
  std::string result;
  result.reserve(input.size());
  bool pending_separator = false;
  for (char c : input) {
    if (absl::ascii_isalnum(c)) {
      if (pending_separator && !result.empty()) {
        result.push_back('_');
      }
      result.push_back(absl::ascii_tolower(c));
      pending_separator = false;
    } else {
      pending_separator = true;
    }
  }
  return result;
}

}  // namespace

std::ostream& operator<<(std::ostream& os,
                         const CameraIdentifier& camera_identifier) {
  return os << absl::StrCat(camera_identifier);
}

std::string CanonicalString(const CameraIdentifier& identifier) {
  return Slugify(absl::StrCat(identifier));
}

}  // namespace intrinsic::perception
