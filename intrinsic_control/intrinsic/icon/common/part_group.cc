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

#include "intrinsic/icon/common/part_group.h"

#include <algorithm>
#include <string>
#include <vector>

#include "absl/container/btree_set.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic {
namespace icon {

PartGroup Part(absl::string_view part_name) {
  return PartGroup({std::string(part_name)});
}

bool IsSinglePart(const PartGroup& group) { return (group.size() == 1); }

const std::string* GetIfSinglePart(const PartGroup& group) {
  if (!IsSinglePart(group)) {
    return nullptr;
  }
  return &(*group.cbegin());
}

PartGroup FromProto(const intrinsic_proto::icon::v1::PartGroup& proto) {
  return PartGroup(proto.parts().begin(), proto.parts().end());
}

intrinsic_proto::icon::v1::PartGroup ToProto(const PartGroup& group) {
  // Sort the names.
  std::vector<std::string> parts(group.cbegin(), group.cend());
  std::sort(parts.begin(), parts.end());
  // Copy to proto.
  intrinsic_proto::icon::v1::PartGroup out;
  *out.mutable_parts() = {parts.begin(), parts.end()};
  return out;
}

}  // namespace icon
}  // namespace intrinsic
