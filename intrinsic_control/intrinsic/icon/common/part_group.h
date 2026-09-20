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

#ifndef INTRINSIC_ICON_COMMON_PART_GROUP_H_
#define INTRINSIC_ICON_COMMON_PART_GROUP_H_

#include <string>

#include "absl/container/btree_set.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/proto/v1/types.pb.h"

namespace intrinsic {
namespace icon {

// A PartGroup is a set containing the names of parts that are to be
// controlled together. It may be empty. It may name a single part.
using PartGroup = absl::btree_set<std::string>;

// Creates a PartGroup containing a single part name.
PartGroup Part(absl::string_view part_name);

// Checks if a PartGroup contains a single part name.
bool IsSinglePart(const PartGroup& group);

// Returns a pointer to the single part name, or `nullptr` if `group` does not
// contain a single part.
const std::string* GetIfSinglePart(const PartGroup& group);

// Creates a PartGroup from proto representation.
PartGroup FromProto(const intrinsic_proto::icon::v1::PartGroup& proto);

// Converts a PartGroup to proto representation. Sorts the parts by name in the
// returned proto.
intrinsic_proto::icon::v1::PartGroup ToProto(const PartGroup& group);

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_COMMON_PART_GROUP_H_
