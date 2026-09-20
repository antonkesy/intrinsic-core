#!/bin/bash

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

set -euo pipefail

# This test runs the check_objects binary on a given gzf file.

world_gzf_filename=""
additional_check_objects_args=""

for arg in "$@"; do
  case "${arg}" in
    --world_gzf_filename=*)
      world_gzf_filename="${arg#*=}"
      ;;
    --additional_check_objects_args=*)
      additional_check_objects_args="${arg#*=}"
      # Strip outer quotes if present
      additional_check_objects_args="${additional_check_objects_args#\'}"
      additional_check_objects_args="${additional_check_objects_args%\'}"
      ;;
  esac
done

CHECK_OBJECTS="$(rlocation "intrinsic-core/intrinsic/world/tools/check_objects")"
readonly CHECK_OBJECTS

"${CHECK_OBJECTS}" \
  --world_gzf_filename="${world_gzf_filename}" \
  --print_debug_info \
  --nocolor \
  ${additional_check_objects_args}