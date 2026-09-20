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

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")

bzl_library(
    name = "fetch_nvidia_libs_bzl",
    srcs = ["fetch_nvidia_libs.bzl"],
    visibility = ["//intrinsic/production/external/nvidia_libs:__subpackages__"],
)

bzl_library(
    name = "cuda_from_redist_bzl",
    srcs = ["cuda_from_redist.bzl"],
    visibility = ["//intrinsic/production/external/nvidia_libs:__subpackages__"],
    deps = [":fetch_nvidia_libs_bzl"],
)

bzl_library(
    name = "cudnn_from_redist_bzl",
    srcs = ["cudnn_from_redist.bzl"],
    visibility = ["//intrinsic/production/external/nvidia_libs:__subpackages__"],
    deps = [":fetch_nvidia_libs_bzl"],
)

bzl_library(
    name = "nccl_from_redist_bzl",
    srcs = ["nccl_from_redist.bzl"],
    visibility = ["//intrinsic/production/external/nvidia_libs:__subpackages__"],
)
