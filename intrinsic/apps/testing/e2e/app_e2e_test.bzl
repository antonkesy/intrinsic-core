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

"""Macros for e2e testing of Intrinsic apps."""

load("//bazel:sh_macros.bzl", "sh_test")
load("//intrinsic/util/path_resolver:paths.bzl", "to_rlocation_path")
load("//intrinsic/util/wrapped_executable:wrapped_executable.bzl", "wrapped_test")

def _gen_test_script_impl(ctx):
    test_config = {
        "operation_mode": ctx.attr.operation_mode,
        "tests": [],
    }
    if ctx.attr.application:
        test_config["solution"] = to_rlocation_path(ctx, ctx.executable.application)
    if ctx.attr.notebooks:
        test_config["tests"].append({
            "files": [to_rlocation_path(ctx, n) for n in ctx.files.notebooks],
            "params": [],
            "type": "notebook",
        })
    if ctx.attr.behavior_trees:
        test_config["tests"].append({
            "files": [to_rlocation_path(ctx, n) for n in ctx.files.behavior_trees],
            "params": [
                "--sim_mode=%s" % ctx.attr.behavior_tree_simulation_mode,
            ],
            "type": "behavior_tree",
        })
    if ctx.attr.binary:
        test_config["tests"].append({
            "files": [to_rlocation_path(ctx, ctx.executable.binary)],
            "params": [
                ctx.expand_location(arg, targets = ctx.attr.binary_data)
                for arg in ctx.attr.binary_params
            ],
            "type": "binary",
        })
    elif ctx.attr.binary_params:
        fail("binary_params are set but no binary is specified")
    elif ctx.attr.binary_data:
        fail("binary_data are set but no binary is specified")
    test_config_file = ctx.actions.declare_file(ctx.label.name + "_test_config.json")
    ctx.actions.write(
        output = test_config_file,
        content = json.encode(test_config),
    )

    test_script = ctx.actions.declare_file(ctx.label.name + ".sh")

    args = ctx.actions.args()
    args.add("--test_config", test_config_file.path)
    args.add("--output_file", test_script.path)
    ctx.actions.run(
        inputs = [test_config_file],
        outputs = [test_script],
        executable = ctx.executable._test_generator,
        arguments = [args],
        mnemonic = "GenAppE2ETestScript",
    )

    return [
        DefaultInfo(
            files = depset([test_script]),
        ),
    ]

_gen_test_script = rule(
    implementation = _gen_test_script_impl,
    attrs = {
        "application": attr.label(executable = True, cfg = "target"),
        "behavior_tree_simulation_mode": attr.string(values = ["REALITY", "DRAFT"]),
        "behavior_trees": attr.label_list(allow_files = [".bt.pb", ".bundle.tar"]),
        "binary": attr.label(executable = True, cfg = "target"),
        "binary_data": attr.label_list(allow_files = True),
        "binary_params": attr.string_list(),
        "notebooks": attr.label_list(allow_files = True),
        "operation_mode": attr.string(values = ["sim", "real"]),
        "_test_generator": attr.label(
            default = Label("//intrinsic/apps/testing/e2e:testgen"),
            cfg = "exec",
            executable = True,
        ),
    },
)

PY_TEST_DEPS = [
    Label("//intrinsic_control/intrinsic/icon/python:create_action_utils"),
    Label("//intrinsic_control/intrinsic/icon/python:icon"),
    Label("//intrinsic_sdk/intrinsic/solutions:solutions_lib"),
    Label("@ai_intrinsic_sdks_pip_deps//numpy"),
]

def _app_e2e_test_impl(
        name,
        visibility,
        application,
        operation_mode,
        notebooks,
        behavior_trees,
        behavior_tree_simulation_mode,
        binary,
        binary_params,
        binary_data,
        size,
        timeout_minutes):
    if not notebooks and not behavior_trees and not binary:
        fail("Must specify at least one notebook, behavior tree, or binary")

    data = []

    if application:
        data.append(application)

    if notebooks:
        data += notebooks
        data.append(Label("//intrinsic/apps/testing/e2e:test_notebook"))

    if behavior_trees:
        data += behavior_trees
        data.append(Label("//intrinsic/apps/testing/e2e:test_behavior_tree"))

    if binary:
        # Embed "args" and "env" to ensure they are passed to the wrapped test.
        wrapped_test(
            name = name + "_wrapped",
            target = binary,
            tags = [
                "manual",
            ],
            visibility = ["//visibility:private"],
        )
        binary = name + "_wrapped"

        data.append(binary)
        data.extend(binary_data)

    test_script = name + "_sh"
    _gen_test_script(
        name = test_script,
        application = application,
        operation_mode = operation_mode,
        notebooks = notebooks,
        behavior_trees = behavior_trees,
        behavior_tree_simulation_mode = behavior_tree_simulation_mode,
        binary = binary,
        binary_params = binary_params,
        binary_data = binary_data,
        tags = ["manual"],
        testonly = True,
        visibility = ["//visibility:private"],
    )

    sh_test(
        name = name,
        size = size,
        timeout = "eternal",  # 60min, time will be limited by guitar workflow
        srcs = [test_script],
        data = data,
        env = {
            "TIMEOUT_MINUTES": str(timeout_minutes),
        },
        tags = [
            "manual",
        ],
        deps = [
            Label("//intrinsic/apps/testing/e2e:app_e2e_test"),
        ],
        visibility = visibility,
    )

app_e2e_test = macro(
    implementation = _app_e2e_test_impl,
    doc = """Creates an e2e test of a Intrinsic app.

    Creates an sh_test that starts the given 'application' and
    runs the given Jupyter 'notebooks', plans, and test binaries.

    The created test will have one test case for each given notebook,
    plan, and binary. The notebooks and plans are executed in the order in which they
    are specified (notebooks first).

    `deployments.connect_to_selected_solution()` in notebooks and python binaries is
    fully supported. The test environment contains an appropriate solution
    selection configuration such that `connect_to_selected_solution()` will
    connect to the solution represented by 'application'.

    To run the test locally for debugging and developing use:
        bazel test --config=intrinsic --test_output streamed --test_env CLUSTER_NAME=vmkube \
        //<path_to_test>:<test_name>

    Additional command line parameters are forwarded to all tests (in addition to
    'binary_params'), e.g.:
        bazel run --config=intrinsic --test_env CLUSTER_NAME=vmkube \
        //<path_to_test>:<test_name> -- --my_flag=my_value
    """,
    attrs = {
        # Note: Most attributes require configurable=False such that we can access their
        # value in the macro (otherwise we'd get `select` objects instead of values).
        # See https://github.com/bazelbuild/bazel/issues/24674.
        "application": attr.label(
            executable = True,
            cfg = "target",
            mandatory = True,
            configurable = False,
            doc = "The application to be started. If not set, an empty application will be started.",
        ),
        "behavior_tree_simulation_mode": attr.string(
            values = ["REALITY", "DRAFT"],
            default = "REALITY",
            doc = "Set the simulation mode for executed behavior trees.",
        ),
        "behavior_trees": attr.label_list(
            allow_files = [".bt.pb", ".bundle.tar"],
            configurable = False,
            doc = "List of labels pointing to the binary protos with behavior trees to run.",
        ),
        "binary": attr.label(
            executable = True,
            cfg = "target",
            configurable = False,
            doc = "The test binary to run (for example a `py_test` using the SBL).",
        ),
        "binary_data": attr.label_list(
            allow_files = True,
            cfg = "target",
            configurable = False,
            doc = "Additional runfiles to be passed to the binary test.",
        ),
        "binary_params": attr.string_list(
            doc = "List of parameters to send to the test binary.",
        ),
        "notebooks": attr.label_list(
            allow_files = True,
            configurable = False,
            doc = "List of labels pointing to the .ipynb files to be tested.",
        ),
        "operation_mode": attr.string(
            values = ["sim", "real"],
            default = "sim",
            doc = "Operation mode for solution.",
        ),
        "size": attr.string(
            default = "large",
            configurable = False,
            doc = "Test size.",
        ),
        "timeout_minutes": attr.int(
            default = 0,  # 0 means no timeout
            doc = "Timeout for the test executing excluding setup and teardown phase.",
            configurable = False,
        ),
    },
)
