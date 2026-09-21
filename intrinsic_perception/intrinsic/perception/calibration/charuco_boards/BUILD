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

load("//bazel:python.bzl", "py_binary")
load(":charuco_boards.bzl", "charuco_board")

package(default_visibility = ["//visibility:private"])

py_binary(
    name = "generate_charuco_board",
    srcs = ["generate_charuco_board.py"],
    deps = [
        "@ai_intrinsic_sdks_pip_deps//numpy",
        "@ai_intrinsic_sdks_pip_deps//opencv_python_headless",
        "@ai_intrinsic_sdks_pip_deps//pillow",
        "@ai_intrinsic_sdks_pip_deps//scipy",
        "@ai_intrinsic_sdks_pip_deps//trimesh",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
    ],
)

# ------------------------------------------------------------------------------
# 200 mm x 150 mm Boards
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# 140 mm x 100 mm Boards
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# 300 mm x 200 mm Boards
# ------------------------------------------------------------------------------

charuco_board(
    name = "charuco_9x14_20mm_15mm_dict_5x5",
    checker_mm = 20,
    grid = (9, 14),
    marker_mm = 15,
    size_mm = (300, 200),
)

# ------------------------------------------------------------------------------
# 400 mm x 300 mm Boards
# ------------------------------------------------------------------------------

charuco_board(
    name = "charuco_9x12_30mm_22mm_dict_5x5",
    checker_mm = 30,
    grid = (9, 12),
    marker_mm = 22,
    size_mm = (400, 300),
)

# ------------------------------------------------------------------------------
# 600 mm x 400 mm Boards
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# 600 mm x 450 mm Boards
# ------------------------------------------------------------------------------

charuco_board(
    name = "charuco_11x15_35mm_26mm_dict_4x4",
    checker_mm = 35,
    dictionary = "DICT_4X4",
    grid = (11, 15),
    marker_mm = 26,
    size_mm = (600, 450),
)

# ------------------------------------------------------------------------------
# 800 mm x 600 mm Boards
# ------------------------------------------------------------------------------

charuco_board(
    name = "charuco_22x30_25mm_18mm_dict_5x5",
    checker_mm = 25,
    grid = (22, 30),
    marker_mm = 18,
    size_mm = (800, 600),
)
