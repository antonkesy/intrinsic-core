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

load("@bazel_skylib//rules:expand_template.bzl", "expand_template")
load("//bazel:cc_macros.bzl", "cc_library")
load(":sensor_pixel_sizes.bzl", "SENSOR_PIXEL_SIZES")

package(default_visibility = ["//visibility:public"])

expand_template(
    name = "sensor_pixel_sizes_h",
    out = "sensor_pixel_sizes.h",
    substitutions = {
        "%SENSOR_PIXEL_SIZES%": ",\n".join([
            '          {\n              "%s",\n              {\n%s\n              },\n          }' % (
                vendor,
                ",\n".join([
                    '                  {"%s", %s}' % (model, size)
                    for model, size in SENSOR_PIXEL_SIZES[vendor].items()
                ]),
            )
            for vendor in SENSOR_PIXEL_SIZES.keys()
        ]),
    },
    template = "sensor_pixel_sizes.h.tpl",
)

cc_library(
    name = "sensor_pixel_size",
    srcs = ["sensor_pixel_size.cc"],
    hdrs = [
        "sensor_pixel_size.h",
        ":sensor_pixel_sizes.h",
    ],
    deps = [
        "//intrinsic_perception/intrinsic/perception/core:range_tools",
        "@abseil-cpp//absl/container:flat_hash_map",
    ],
)
