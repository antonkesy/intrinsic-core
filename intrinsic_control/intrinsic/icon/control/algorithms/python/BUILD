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

load("//bazel:cc_macros.bzl", "pybind_extension")

# With bazel the pybind_extension does not work if the visibility is restricted.
package(default_visibility = ["//visibility:public"])

# Use the Python wrapper in intrinsic/manipulation/skills/force/utils.py to access this library.
pybind_extension(
    name = "directional_stiffness",
    srcs = ["directional_stiffness.cc"],
    deps = [
        "//intrinsic_control/intrinsic/icon/control/algorithms:directional_stiffness",
        "//intrinsic_sdk/intrinsic/eigenmath",
        "@abseil-cpp//absl/status:statusor",
        "@pybind11_abseil//pybind11_abseil:status_casters",
    ],
)

# Use the Python wrapper in intrinsic/manipulation/skills/force/utils.py to access this library.
pybind_extension(
    name = "overdamping",
    srcs = ["overdamping.cc"],
    deps = [
        "//intrinsic_control/intrinsic/icon/control/algorithms:overdamping",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status_macro",
        "//intrinsic_sdk/intrinsic/eigenmath",
        "@abseil-cpp//absl/status:statusor",
        "@pybind11_abseil//pybind11_abseil:status_casters",
    ],
)
