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

"""Providers for solution-related starlark rules"""

ResourceSetInfo = provider(
    "provided by intrinsic_resource_set() rule",
    fields = [
        "resources_proto",
        "resource_bundle_files",  # a list of resource bundle files to be deployed
    ],
)

SolutionInfo = provider(
    "provided by the intrinsic_solution() rule",
    fields = {
        "asset_bundles": "asset bundle files used in the solution",
        "solution": "binary proto file containing a LocalSolution message",
    },
)

GeometricDataInfo = provider(
    "provided by the geometric_data aspect that converts a solution",
    fields = {
        "proto": "binary proto file containing a GeometricResourceSetData proto",
    },
)
