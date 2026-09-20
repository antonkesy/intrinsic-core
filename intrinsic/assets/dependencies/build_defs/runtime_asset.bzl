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

"""Bazel rules for the runtime Asset."""

def _intrinsic_runtime_installed_asset_impl(ctx):
    runtime_installed_asset_out = ctx.outputs.out
    version_file = ctx.file.version_file

    args = ctx.actions.args().add(
        "--output_installed_asset",
        runtime_installed_asset_out,
    ).add(
        "--version_file",
        version_file,
    )
    ctx.actions.run(
        outputs = [runtime_installed_asset_out],
        inputs = [version_file],
        executable = ctx.executable._runtimeassetgen,
        arguments = [args],
        mnemonic = "RuntimeInstalledAssetGen",
        progress_message = "Generating runtime InstalledAsset for %{label}",
    )

    return [
        DefaultInfo(files = depset([runtime_installed_asset_out])),
    ]

intrinsic_runtime_installed_asset = rule(
    implementation = _intrinsic_runtime_installed_asset_impl,
    doc = "Builds the runtime as a serialized InstalledAsset proto file.",
    attrs = {
        "stamp": attr.int(default = -1),
        "version_file": attr.label(
            default = Label("//intrinsic/assets/dependencies/build_defs:version"),
            allow_single_file = True,
        ),
        "_runtimeassetgen": attr.label(
            default = Label("//intrinsic/assets/dependencies/build_defs:runtimeinstalledassetgen_main"),
            cfg = "exec",
            executable = True,
        ),
    },
    outputs = {"out": "%{name}.pbtxt"},
)

def _intrinsic_runtime_asset_instance_impl(ctx):
    runtime_asset_instance_out = ctx.outputs.out
    version_file = ctx.file.version_file

    args = ctx.actions.args().add(
        "--output_asset_instance",
        runtime_asset_instance_out,
    ).add(
        "--version_file",
        version_file,
    )
    ctx.actions.run(
        outputs = [runtime_asset_instance_out],
        inputs = [version_file],
        executable = ctx.executable._runtimeassetinstancegen,
        arguments = [args],
        mnemonic = "RuntimeAssetInstanceGen",
        progress_message = "Generating runtime AssetInstance for %{label}",
    )

    return [
        DefaultInfo(files = depset([runtime_asset_instance_out])),
    ]

intrinsic_runtime_asset_instance = rule(
    implementation = _intrinsic_runtime_asset_instance_impl,
    doc = "Builds the runtime as a serialized AssetInstance proto file.",
    attrs = {
        "stamp": attr.int(default = -1),
        "version_file": attr.label(
            default = Label("//intrinsic/assets/dependencies/build_defs:version"),
            allow_single_file = True,
        ),
        "_runtimeassetinstancegen": attr.label(
            default = Label("//intrinsic/assets/dependencies/build_defs:runtimeassetinstancegen_main"),
            cfg = "exec",
            executable = True,
        ),
    },
    outputs = {"out": "%{name}.pbtxt"},
)

def _intrinsic_runtime_resource_instance_impl(ctx):
    runtime_resource_instance_out = ctx.outputs.out
    version_file = ctx.file.version_file

    args = ctx.actions.args().add(
        "--output_resource_instance",
        runtime_resource_instance_out,
    ).add(
        "--version_file",
        version_file,
    )
    ctx.actions.run(
        outputs = [runtime_resource_instance_out],
        inputs = [version_file],
        executable = ctx.executable._runtimeresourceinstancegen,
        arguments = [args],
        mnemonic = "RuntimeResourceInstanceGen",
        progress_message = "Generating runtime ResourceInstance for %{label}",
    )

    return [
        DefaultInfo(files = depset([runtime_resource_instance_out])),
    ]

intrinsic_runtime_resource_instance = rule(
    implementation = _intrinsic_runtime_resource_instance_impl,
    doc = "Builds the runtime as a serialized ResourceInstance proto file.",
    attrs = {
        "stamp": attr.int(default = -1),
        "version_file": attr.label(
            default = Label("//intrinsic/assets/dependencies/build_defs:version"),
            allow_single_file = True,
        ),
        "_runtimeresourceinstancegen": attr.label(
            default = Label("//intrinsic/assets/dependencies/build_defs:runtimeresourceinstancegen_main"),
            cfg = "exec",
            executable = True,
        ),
    },
    outputs = {"out": "%{name}.pbtxt"},
)
