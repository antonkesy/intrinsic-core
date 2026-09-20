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

# Generic main script for e2e tests of Intrinsic apps. Notebooks and scripts
# to-be-tested can be passed by flag and this script will create a test
# case for each one of them at runtime.
set -euo pipefail

source "$(rlocation "intrinsic-core/intrinsic/testing/util/integration_test_util.sh")"

readonly GRPC_TIMEOUT=60

NAMESPACE=""
TEST_START_TIME="$(date +%s)"
readonly TEST_START_TIME

# Passes if running the binary succeeded.
# Parameters:
#   $1: Path to the binary in the runfiles. E.g.,
#       "intrinsic/script_under_test".
#   $@: Additional parameters passed to the test script, optional.
function test_binary {
  local test_result

  TEST_CMD=(
    "$@"
  )

  if [[ -n "${TIMEOUT_MINUTES:-}" && "${TIMEOUT_MINUTES}" -gt 0 ]]; then
    ts_echo "Running test with timeout ${TIMEOUT_MINUTES}m"
    TEST_CMD=(timeout "${TIMEOUT_MINUTES}m" "${TEST_CMD[@]}")
  fi

  # Run inside TEST_TMPDIR so that, e.g., the binary can write files.
  (cd "${TEST_TMPDIR}" && \
    GRPC_CONNECTION_TIMEOUT="${GRPC_TIMEOUT}" \
      "${TEST_CMD[@]}")
  test_result=$?

  if [[ ${test_result} -ne 0 ]]
  then
    print_app_logs "executive skills-cpp"
  fi
  return ${test_result}
}

# Passes if running the notebook succeeded.
# Parameters:
#   $1: Path to the notebook in the runfiles. E.g.,
#       "intrinsic/notebook_under_test.ipynb".
#   $@: Additional parameters passed to the test_notebook script, optional.
#       Defaults to "".
function test_notebook {
  TEST_CMD=("$(rlocation "intrinsic-core/intrinsic/apps/testing/e2e/test_notebook")")
  NOTEBOOK_PATH="$1"
  shift 1
  TEST_CMD+=(
    --notebook_path="${NOTEBOOK_PATH}"
    --hostname="${HOSTNAME}"
    "$@"
  )

  test_binary "${TEST_CMD[@]}"
}

# Passes if running the behavior tree succeeded.
# Parameters:
#   $1: Path to the behavior tree in the runfiles. E.g.,
#       "intrinsic/behavior_tree_under_test.ipynb".
#   $@: Additional parameters passed to the test_behavior_tree script, optional.
#       Defaults to "".
function test_behavior_tree {
  TEST_CMD=("$(rlocation "intrinsic-core/intrinsic/apps/testing/e2e/test_behavior_tree")")
  BEHAVIOR_TREE_PATH="$1"
  shift 1
  TEST_CMD+=(
    --behavior_tree_path="${BEHAVIOR_TREE_PATH}"
    "$@"
  )

  test_binary "${TEST_CMD[@]}"
}

function setup_all_tests() {
  if [[ "${E2E_DEBUG-false}" != "true" ]]; then
    ts_echo "Deleting all running workcells"
    delete_all_running_apps
    ts_echo "Deleted all running workcells"
  else
    echo "Not stopping running app because E2E_DEBUG=true was given."
  fi

  ts_echo "Starting app ${INTRINSIC_SOLUTION}"
  # We specify --registry and --skip_direct_upload to avoid adding significant
  # load onto the API relay.  In the future we may want to remove those flags in
  # order to load test it internally.
  args=(
    --cluster="${CLUSTER_NAME}"
    "--operation_mode=${OPERATION_MODE}"
    "--registry=gcr.io/${INTRINSIC_ORG#*@}"
    "--skip_direct_upload"
  )
  "${INTRINSIC_SOLUTION}" "${args[@]}"
  ts_echo "Started app ${INTRINSIC_SOLUTION}"

  NAMESPACE=$(get_app_namespace)

  ts_echo "General test setup complete"
}

function teardown_all_tests() {
  ts_echo " Starting teardown of all tests"

  if [[ "${E2E_DEBUG-false}" != "true" ]]; then
    inctl_app_stop
  else
    echo "Not stopping running app because E2E_DEBUG=true was given."
  fi

  ts_echo " Finished teardown of all tests"
}

function test::setup_all_tests() {
  setup_all_tests "$@"
}

function test::teardown_all_tests() {
  teardown_all_tests "$@"
}
