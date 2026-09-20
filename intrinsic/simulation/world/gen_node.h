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

#ifndef INTRINSIC_SIMULATION_WORLD_GEN_NODE_H_
#define INTRINSIC_SIMULATION_WORLD_GEN_NODE_H_

#include <memory>
#include <string>
#include <vector>

namespace intrinsic {
namespace simulation {
namespace details {

// GenNode is a simple N-ary tree structure designed to help conversion of
// recursive data. Once the tree is built `Flatten()` would output in prelude,
// flatten of children, postlude order, resursively.
struct GenNode {
 public:
  std::string start_tag;
  std::string pose_elem;
  std::string prelude;
  std::string postlude;
  std::string end_tag;

  // Adds a GenNode as a child of this GenNode.
  void AddChild(std::unique_ptr<GenNode> child);

  // Returns the flattened string of this GenNode and its children (in creation
  // order), recursively.
  std::string Flatten();

 private:
  std::vector<std::unique_ptr<GenNode>> children;
};

}  // namespace details
}  // namespace simulation
}  // namespace intrinsic

#endif  // INTRINSIC_SIMULATION_WORLD_GEN_NODE_H_
