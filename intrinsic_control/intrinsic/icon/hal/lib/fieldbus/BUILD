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

load("//bazel:cc_macros.bzl", "cc_library")

package(
    default_visibility = [
        "//intrinsic_control/intrinsic/icon:__subpackages__",
        "//intrinsic_control/intrinsic/icon/hal:__subpackages__",
    ],
)

cc_library(
    name = "process_variable",
    srcs = ["process_variable.cc"],
    hdrs = ["process_variable.h"],
    deps = [
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status_or",
    ],
)

cc_library(
    name = "value_pattern_utils",
    srcs = ["value_pattern_utils.cc"],
    hdrs = ["value_pattern_utils.h"],
    deps = [
        ":process_variable",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/strings",
    ],
)
