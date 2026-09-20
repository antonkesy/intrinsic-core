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

"""Provider and rules for generating xacro output at build time."""

# Disable buildifier, rule checked-in here until https://github.com/ros/xacro/issues/381 is resolved upstream
XacroInfo = provider(
    "Provider holding the result of a xacro generation step.",
    fields = ["result"],
)

XACRO_EXTENSION = ".xacro"

def _xacro_impl(ctx):
    # FIX 1: Use .basename instead of .path when 'out' is not provided.
    # This prevents Bazel from accidentally double-nesting the output files
    # deep inside the workspace paths when used inside the xacro_filegroup macro.
    out = ctx.outputs.out or ctx.actions.declare_file(ctx.file.src.basename[:-len(XACRO_EXTENSION)])

    # Compute prefix in the case that this is in an external module
    prefix = "external/" + ctx.label.repo_name + "/"

    # Create a temporary directory in the current location in the tree
    temp_dir = "TMP_XACRO/" + ctx.label.name

    # Gather inputs for the xacro command
    direct_inputs = [ctx.file.src] + ctx.files.data
    dep_inputs = [dep[XacroInfo].result for dep in ctx.attr.deps]

    # For each direct input, symlink the input into the temporary directory
    symlink_paths = []
    for input in direct_inputs:
        input_path = input.path
        if input_path.startswith(prefix):
            input_path = input_path[len(prefix):]

        # strip google3 prefix as part of b/490385134
        # TODO(b/477580650): Remove google3/ logic after moved to incode
        if input_path.startswith("google3/"):
            input_path = input_path[len("google3/"):]
        symlink_path = ctx.actions.declare_file(temp_dir + "/" + input_path)
        ctx.actions.symlink(
            output = symlink_path,
            target_file = input,
        )
        symlink_paths.append(symlink_path)

    # For each dependent input, symlink the input into the temporary directory
    for di in dep_inputs:
        di_path = di.short_path

        # strip google3 prefix as part of b/490385134
        # TODO(b/477580650): Remove google3/ logic after moved to incode
        if di_path.startswith("google3/"):
            di_path = di_path[len("google3/"):]
        symlink_path = ctx.actions.declare_file(temp_dir + "/" + di_path)
        ctx.actions.symlink(
            output = symlink_path,
            target_file = di,
        )
        symlink_paths.append(symlink_path)

    # Recompute the primary input_path just like we did for the symlinks
    input_path = ctx.file.src.path
    if input_path.startswith(prefix):
        input_path = input_path[len(prefix):]

    # strip google3 prefix as part of b/490385134
    # TODO(b/477580650): Remove google3/ logic after moved to incode
    if input_path.startswith("google3/"):
        input_path = input_path[len("google3/"):]

    # FIX 2: Accurately determine the root directory based on where Bazel actually
    # placed the symlinks, completely avoiding the buggy double-slash execution root
    # string manipulation. symlink_paths[0] is guaranteed to be the symlink for ctx.file.src.
    root_dir = symlink_paths[0].path[:-len(input_path) - 1]

    arguments = [
        "-o",
        out.path,
        "--root-dir",
        root_dir,
        input_path,
    ]
    arguments += ["{}:={}".format(arg, val) for arg, val in ctx.attr.arguments.items()]

    ctx.actions.run(
        inputs = symlink_paths,
        outputs = [out],
        arguments = arguments,
        executable = ctx.executable._xacro,
        progress_message = "Running xacro: %s -> %s" % (ctx.file.src.short_path, out.short_path),
        mnemonic = "Xacro",
    )

    return [
        XacroInfo(result = out),
        DefaultInfo(
            files = depset([out]),
            data_runfiles = ctx.runfiles(files = [out]),
        ),
    ]

xacro_file = rule(
    attrs = {
        "arguments": attr.string_dict(),
        "data": attr.label_list(
            allow_files = True,
        ),
        "deps": attr.label_list(providers = [XacroInfo]),
        "out": attr.output(),
        "src": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "_xacro": attr.label(
            default = Label("@xacro//:xacro"),
            cfg = "exec",
            executable = True,
        ),
    },
    implementation = _xacro_impl,
    provides = [XacroInfo, DefaultInfo],
)

def xacro_filegroup(
        name,
        srcs = [],
        data = [],
        tags = [],
        visibility = None):
    """Runs xacro on several input files, creating a filegroup of the output.

    The output filenames will match the input filenames but with the ".xacro"
    suffix removed.

    Xacro is the ROS XML macro tool; http://wiki.ros.org/xacro.

    Args:
      name: The name of the filegroup label.
      srcs: The xacro input files of this rule.
      data: Optional supplemental files required by the srcs.
      tags: Tags to pass to the underlying xacro_file rules.
      visibility: Visibility of the resulting filegroup.
    """
    outs = []
    for src in srcs:
        if not src.endswith(XACRO_EXTENSION):
            fail("xacro_filegroup srcs should be named *.xacro not {}".format(
                src,
            ))
        out = src[:-len(XACRO_EXTENSION)]
        outs.append(out)
        xacro_file(
            name = out,
            src = src,
            data = data,
            tags = tags,
            visibility = ["//visibility:private"],
        )
    native.filegroup(
        name = name,
        srcs = outs,
        visibility = visibility,
    )
