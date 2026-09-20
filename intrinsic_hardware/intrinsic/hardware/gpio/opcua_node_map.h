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

#ifndef INTRINSIC_HARDWARE_GPIO_OPCUA_NODE_MAP_H_
#define INTRINSIC_HARDWARE_GPIO_OPCUA_NODE_MAP_H_

#include <cstddef>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "open62541/types.h"

namespace intrinsic::opcua {

// Unordered associative container to associate templated node values with node
// display name, node string identifier (if valid) and node identifier in opcua
// xml standard. For convenience, STL iterators are exposed to iterate over
// [display_name, node_value, xml_node_value] key value pairs. Lookups are
// performed against all three keys. If any one matches, the iterator to the
// underlying data is returned.
template <typename T>
class OpcuaMultiKeyNodeMap {
 public:
  OpcuaMultiKeyNodeMap() = default;
  ~OpcuaMultiKeyNodeMap() = default;

  OpcuaMultiKeyNodeMap(OpcuaMultiKeyNodeMap&&) = default;
  OpcuaMultiKeyNodeMap(const OpcuaMultiKeyNodeMap&) = default;
  OpcuaMultiKeyNodeMap& operator=(const OpcuaMultiKeyNodeMap&) = default;
  OpcuaMultiKeyNodeMap& operator=(OpcuaMultiKeyNodeMap&&) = default;

  // Adds support for STL iterators to iterate over
  // [display_name, node_value, xml_node_value].
  using underlying_container = absl::flat_hash_map<std::string, T>;
  using iterator = typename underlying_container::iterator;
  using const_iterator = typename underlying_container::const_iterator;
  iterator begin() { return name_to_data_.begin(); }
  iterator end() { return name_to_data_.end(); }
  const_iterator begin() const { return name_to_data_.begin(); }
  const_iterator end() const { return name_to_data_.end(); }
  const_iterator cbegin() const { return name_to_data_.begin(); }
  const_iterator cend() const { return name_to_data_.end(); }

  // Inserts node_data in the map and associate it with display name, the node
  // string identifier (if valid) and the xml node identifier string.
  void Insert(const UA_NodeId& node_id, const absl::string_view display_name,
              const absl::string_view xml_node_id, T node_data) {
    name_to_data_.insert({std::string(display_name), node_data});

    if (node_id.identifierType == UA_NODEIDTYPE_STRING) {
      const auto node_identifier =
          absl::string_view((const char*)node_id.identifier.string.data,
                            node_id.identifier.string.length);
      node_string_id_to_display_name_.insert(
          {std::string(node_identifier), std::string(display_name)});
    }
    xml_node_id_to_display_name_.insert(
        {std::string(xml_node_id), std::string(display_name)});
  }

  // Returns an iterator to the [display_name, node_value, xml_node_value] given
  // the node name. The name is matched against all of the display name, the
  // node string identifier and the xml node identifier string. However,
  // the returned iterator's key always contains the node display name. Iterator
  // should be checked for validity before being de-referenced.
  const_iterator Find(const absl::string_view name) const {
    if (const auto it = name_to_data_.find(name); it != name_to_data_.cend()) {
      return it;
    }
    if (const auto it = node_string_id_to_display_name_.find(name);
        it != node_string_id_to_display_name_.cend()) {
      const auto& different_name = it->second;
      return name_to_data_.find(different_name);
    }
    if (const auto it = xml_node_id_to_display_name_.find(name);
        it != xml_node_id_to_display_name_.cend()) {
      const auto& different_name = it->second;
      return name_to_data_.find(different_name);
    }
    return name_to_data_.cend();
  }

  // Returns true if the given node name is present in the map.
  bool Contains(const absl::string_view name) const {
    return Find(name) != name_to_data_.cend();
  }

  // Returns the number of node values stored in the container.
  size_t Size() const { return name_to_data_.size(); }

  // Returns true if there are no node values.
  bool Empty() const { return name_to_data_.empty(); }

 private:
  // Map from node display name to the data associated to that node.
  underlying_container name_to_data_;

  // Map from node string identifier to the node display name.
  absl::flat_hash_map<std::string, std::string> node_string_id_to_display_name_;

  // Map from xml node identifier string to the node display name.
  absl::flat_hash_map<std::string, std::string> xml_node_id_to_display_name_;
};

}  // namespace intrinsic::opcua

#endif  // INTRINSIC_HARDWARE_GPIO_OPCUA_NODE_MAP_H_
