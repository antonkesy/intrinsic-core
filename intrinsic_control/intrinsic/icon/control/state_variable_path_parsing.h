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

#ifndef INTRINSIC_ICON_CONTROL_STATE_VARIABLE_PATH_PARSING_H_
#define INTRINSIC_ICON_CONTROL_STATE_VARIABLE_PATH_PARSING_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/common/state_variable_path_util.h"
#include "re2/re2.h"

namespace intrinsic::icon {

// Regex to match strings like "my_value" or "my_value[123]"
constexpr LazyRE2 kPartStatusNodeRegex = {
    R"(^([[:alpha:]]{1}\w*)(?:\[(\d+)\])?$)"};
// Unit-test verified examples for valid node strings.
constexpr absl::string_view node_example_strings[2] = {"my_value",
                                                       "my_value[0]"};

// Populates a node from a string. The string must contain a valid name
// (alphanumeric and underscore) and optionally an array index, e.g.
// position[0].
absl::StatusOr<StateVariablePathNode> ParseNodeString(
    absl::string_view node_string);

// Populates a node vector from a string. The path is split into nodes using
// `kStateVariablePathSeparator` and nodes are parsed with `ParseNodeString()`.
// The path needs to start with `kStateVariablePathPrefix`, which is stripped
// away before parsing the nodes. Returns an InvalidArgumentError if the `path`
// does not start with `kStateVariablePathPrefix` or if a node does not match
// the regex specified in `kPartStatusNodeRegex`.
absl::StatusOr<std::vector<StateVariablePathNode>> PathToNodeVec(
    absl::string_view path);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_STATE_VARIABLE_PATH_PARSING_H_
