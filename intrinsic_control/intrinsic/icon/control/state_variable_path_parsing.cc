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

#include "intrinsic/icon/control/state_variable_path_parsing.h"

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/common/state_variable_path_constants.h"
#include "intrinsic/icon/common/state_variable_path_util.h"
#include "intrinsic/util/status/status_macros.h"
#include "re2/re2.h"

namespace intrinsic::icon {

absl::StatusOr<StateVariablePathNode> ParseNodeString(
    absl::string_view node_string) {
  std::string name;
  std::string index_str;
  if (node_string.empty()) {
    return absl::InvalidArgumentError("Node string cannot be empty.");
  }
  // Capture the values of capture group 1 (name) and capture group 2 (index) of
  // regex kPartStatusNodeRegex.
  if (!RE2::PartialMatch(node_string, *kPartStatusNodeRegex, &name,
                         &index_str)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Could not parse node string '", node_string, "' with regex: '",
        kPartStatusNodeRegex.pattern_, "'. Valid examples: '",
        absl::StrJoin(node_example_strings, "', '"), "'"));
  }

  if (name.size() > kMaxNodeNameLength) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Node name is too long: ", name.size(), " max: ", kMaxNodeNameLength));
  }

  std::optional<size_t> index_opt;
  if (!index_str.empty()) {
    size_t index;
    if (!absl::SimpleAtoi(index_str, &index)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Cannot convert index string '", index_str, "' to integer."));
    }

    index_opt = index;
  }
  return StateVariablePathNode{name, index_opt};
}

absl::StatusOr<std::vector<StateVariablePathNode>> PathToNodeVec(
    absl::string_view path) {
  if (!absl::StartsWith(path, kStateVariablePathPrefix)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "State variable paths must start with '", kStateVariablePathPrefix,
        "', but '", path, "' does not!"));
  }
  path.remove_prefix(absl::string_view(kStateVariablePathPrefix).size());
  const std::vector<absl::string_view> node_strings =
      absl::StrSplit(path, kStateVariablePathSeparator);

  std::vector<StateVariablePathNode> part_field_nodes;
  for (size_t i = 0; i < node_strings.size(); ++i) {
    INTR_ASSIGN_OR_RETURN(auto node, ParseNodeString(node_strings[i]));
    part_field_nodes.emplace_back(std::move(node));
  }
  return part_field_nodes;
}
}  // namespace intrinsic::icon
