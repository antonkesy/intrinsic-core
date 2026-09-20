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

#ifndef INTRINSIC_WORLD_UTIL_COMPOSE_WORLD_UTIL_H_
#define INTRINSIC_WORLD_UTIL_COMPOSE_WORLD_UTIL_H_

#include <cstddef>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/config/proto/resource_set.pb.h"
#include "intrinsic/geometry/storage/geometry_library.h"
#include "intrinsic/resources/proto/geometric_resource_data.pb.h"
#include "intrinsic/world/proto/object_world_service.pb.h"
#include "intrinsic/world/proto/object_world_updates.pb.h"
#include "intrinsic/world/world.h"

namespace intrinsic {

// Options for composing resource instances into the world.
struct ComposeWorldOptions {
  static const ComposeWorldOptions& Default() {
    static ComposeWorldOptions options;
    return options;
  }

  // Update policy for the world updates.
  enum class UpdatePolicy {
    // Default is the same as kFailOnFirstError
    kDefault = 0,
    // Return an error on the first failed update.
    kFailOnFirstError = 0,
    // Skip failed updates, and return the world with the updates that
    // succeeded, and the errors for the failed updates.
    kSkipFailedUpdates = 1,
  };

  // If set, will suppress logging each individual resource.
  bool quiet = false;
  // The policy to use when updating the world.
  UpdatePolicy update_policy = UpdatePolicy::kFailOnFirstError;
};

// Result of composing resource instances into the world.
struct ComposeWorldResult {
  struct UpdateError {
    // The index of the update in the ObjectWorldUpdates proto.
    size_t index;

    // A copy of the update that failed.
    intrinsic_proto::world::ObjectWorldUpdate problem_update;
  };

  struct CompositionError {
    // Either the problem update or the resource name that failed to compose.
    std::variant<UpdateError, std::string> problem;

    // The status of the failed update.
    absl::Status status;
  };

  // The world with the updates that succeeded.
  World world;

  // The errors status of all the updates that failed.
  std::vector<CompositionError> composition_errors;
};

// Composes resources instances into the world based on the given relationship
// updates.
absl::StatusOr<ComposeWorldResult> ComposeResourceInstancesIntoWorld(
    const std::vector<
        intrinsic_proto::resources::GeometricResourceInstanceData>&
        resource_instance_data,
    const intrinsic_proto::world::ObjectWorldUpdates& updates,
    GeometryLibrary& geolib,
    const ComposeWorldOptions& options = ComposeWorldOptions::Default());

absl::StatusOr<ComposeWorldResult> ComposeResourceSetIntoWorld(
    const intrinsic_proto::resources::GeometricResourceSetData& data,
    GeometryLibrary& geolib,
    const ComposeWorldOptions& options = ComposeWorldOptions::Default());

}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_COMPOSE_WORLD_UTIL_H_
