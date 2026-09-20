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

"""Public entry point for web_test_environment."""

load("//bazel/frontend/private:web_test_environment.bzl", _WebTestEnvironmentInfo = "WebTestEnvironmentInfo", _web_test_environment = "web_test_environment")

WebTestEnvironmentInfo = _WebTestEnvironmentInfo

def web_test_environment(browser, data = [], styles = [], **kwargs):
    """Defines a web test environment.

    Args:
        browser: Browser that should be used in the test.
        data: Additional files to be made available during testing.
        styles: Stylesheets to load during the test.
        **kwargs: Arguments to pass to the underlying `web_test_environment` rule.
    """
    _web_test_environment(
        browser = browser,
        data = data,
        styles = styles,
        testonly = True,
        **kwargs
    )
