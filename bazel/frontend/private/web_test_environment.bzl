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

"""Implementation of the web_test_environment rule."""

load("@bazel_lib//lib:copy_to_bin.bzl", "COPY_FILE_TO_BIN_TOOLCHAINS")
load("//bazel/frontend/browser_automation:browser_info.bzl", "BrowserInfo")

visibility(["//bazel/frontend", "//bazel/frontend/private/karma"])

WebTestEnvironmentInfo = provider(
    doc = """Information about a web_test_environment.""",
    fields = {
        "environment": "(dict[string]string) environment variables that should be set to utilize the environment",
        "styles": "(list[File]) stylesheets to load during the test",
    },
)

def _web_test_environment_impl(ctx):
    runfiles = ctx.runfiles(files = ctx.files.data)
    runfiles = runfiles.merge(ctx.attr.browser[DefaultInfo].default_runfiles)
    runfiles = runfiles.merge_all([
        target[DefaultInfo].default_runfiles
        for target in ctx.attr.data
    ])

    return [
        DefaultInfo(
            runfiles = runfiles,
        ),
        WebTestEnvironmentInfo(
            environment = ctx.attr.browser[BrowserInfo].environment,
            styles = ctx.files.styles,
        ),
    ]

web_test_environment = rule(
    implementation = _web_test_environment_impl,
    provides = [WebTestEnvironmentInfo],
    attrs = {
        "browser": attr.label(
            providers = [BrowserInfo],
            mandatory = True,
            doc = """
            Browser that should be used in the test.
            """,
        ),
        "data": attr.label_list(
            allow_files = True,
            doc = """
            Additional files to be made available during testing.
            """,
        ),
        "styles": attr.label_list(
            allow_files = [".css", ".scss"],
            doc = """
            Stylesheets to load during the test.
            """,
        ),
    },
    toolchains = COPY_FILE_TO_BIN_TOOLCHAINS,
    doc = """
    Rule for defining a web test environment.

    Each web test environment specifies a configuration for web tests to run with. Most notably, it
    provides the browser to run the tests in.
    """,
)
