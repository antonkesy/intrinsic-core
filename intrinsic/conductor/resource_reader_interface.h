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

#ifndef INTRINSIC_CONDUCTOR_RESOURCE_READER_INTERFACE_H_
#define INTRINSIC_CONDUCTOR_RESOURCE_READER_INTERFACE_H_

#include <cstdint>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/config/proto/resource_set.pb.h"
#include "intrinsic/resources/proto/geometric_resource_data.pb.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"

namespace intrinsic::conductor {

class ResourceReaderInterface {
 public:
  virtual ~ResourceReaderInterface() = default;
  virtual absl::StatusOr<
      std::vector<intrinsic_proto::resources::GeometricResourceInstanceData>>
  GeometricResourceInstanceData(
      const intrinsic_proto::config::ResourceSet& rs) const = 0;
  virtual absl::StatusOr<std::pair<
      std::vector<intrinsic_proto::resources::GeometricResourceInstanceData>,
      intrinsic_proto::world::ObjectWorldUpdates>>
  GetGeometricResourceSetData() const = 0;
  virtual uintptr_t GetHandle() const = 0;
};

}  // namespace intrinsic::conductor

#endif  // INTRINSIC_CONDUCTOR_RESOURCE_READER_INTERFACE_H_
