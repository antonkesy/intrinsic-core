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

#ifndef INTRINSIC_SKILLS_APPS_TESTING_TEST_WORLD_H_
#define INTRINSIC_SKILLS_APPS_TESTING_TEST_WORLD_H_

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace testing {

// Loads a pre-generated test world to be found at `world_gzf_path`.
//
// Example usage:
// INTR_ASSIGN_OR_RETURN(auto world,
// LoadTestWorld("intrinsic/skills/apps/test_data/ur5e_world.gzf"));
absl::StatusOr<::intrinsic::World> LoadTestWorld(
    absl::string_view world_gz_filepath);

// Same as above, but additionally sets `robot_name` and `resource_name`.
// Returns an error status if the world contains more than one robot.
absl::StatusOr<::intrinsic::World> LoadTestWorld(
    absl::string_view world_gz_filepath, const WorldObjectName& robot_name,
    absl::string_view robot_resource_name);

}  // namespace testing
}  // namespace intrinsic

#endif  // INTRINSIC_SKILLS_APPS_TESTING_TEST_WORLD_H_
