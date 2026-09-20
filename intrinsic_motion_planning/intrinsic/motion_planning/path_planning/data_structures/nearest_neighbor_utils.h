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

// Note, this header file includes ompl_header files that include an ompl
// exception headers. As a direct consequence, files including this header
// file need to add the following flags into the build rules:
//   copts = ["-fexceptions"],
//   features = ["-use_header_modules"],
#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_NEAREST_NEIGHBOR_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_NEAREST_NEIGHBOR_UTILS_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/motion_planning/path_planning/data_structures/linear_nearest_neighbor.h"
#include "intrinsic/motion_planning/path_planning/data_structures/nearest_neighbor.h"
#include "intrinsic/motion_planning/path_planning/data_structures/nearest_neighbor_configs.pb.h"

namespace intrinsic {
// Generates the specific nearest neighbor data type from a proto defined in
// proto::NearestNeighborSpecification.
template <typename NearestNeighborType, typename ProtoType, typename DataType>
absl::StatusOr<std::unique_ptr<NearestNeighbors<DataType>>>
CreateNearestNeighborFromAnyConfig(
    const proto::NearestNeighborSpecification& spec) {
  ProtoType proto;
  if (!spec.config().UnpackTo(&proto)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to unpack config to specified type. Failure occurred for ",
        spec.name()));
  }
  auto nearest_neighbor = std::make_unique<NearestNeighborType>(proto);
  return std::unique_ptr<NearestNeighbors<DataType>>(
      std::move(nearest_neighbor));
}

// Generates a unique_ptr to the nearest neighbor structure defined in
// proto::NearestNeighborSpecification.
template <typename DataType>
absl::StatusOr<std::unique_ptr<NearestNeighbors<DataType>>>
CreateNearestNeighbor(const proto::NearestNeighborSpecification& config) {
  if (config.name() == "LinearNearestNeighbor") {
    return CreateNearestNeighborFromAnyConfig<LinearNearestNeighbor<DataType>,
                                              proto::NearestNeighborConfig,
                                              DataType>(config);
  }

  return absl::InvalidArgumentError(
      absl::StrCat("Unknown nearest neighbor type: ", config.name()));
}
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_DATA_STRUCTURES_NEAREST_NEIGHBOR_UTILS_H_
