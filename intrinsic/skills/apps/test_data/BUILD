# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Test data for skill implementations.
load("//intrinsic/world/build_def:sdf_world.bzl", "sdf_world")

package(default_visibility = [
    "//visibility:public",
])

sdf_world(
    name = "ur5e_world",
    testonly = True,
    src = "ur5e.sdf",
    generate_group_ids = True,  # TODO(b/283374158): Set to False or remove
    model_resource_names = {
        "ur5e": "my_robot",
    },
    sdf_assets = ["//intrinsic/robot_definitions/ur:ur5e"],
    updates_pbtxts = ["ur5e_world_updates.pbtxt"],
)
