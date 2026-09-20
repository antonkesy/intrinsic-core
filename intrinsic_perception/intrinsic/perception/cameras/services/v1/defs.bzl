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

"""Helper function to calculate the ROS camera service workdir."""

def ros_camera_service_workdir(binary_label):
    """
    Calculate the workdir needed for ros_camera_service.

    The ROS camera service uses rules_ros2, which will try to dynamically
    load the ROS middleware and typesupport libraries at runtime, with paths
    that begin with "../". For this to work, the workdir needs to be a bit
    different from the typical usage, starting in the bazel runfiles tree
    for the main repo.

    Args:
      binary_label (Label): label of the binary running in ros_camera_service

    Returns:
      str: the calculated workdir to use for ros_camera_service
    """
    package_path = binary_label.package

    # TODO(b/477580650): Update this path calculation after migration from /google3
    if package_path.startswith("google3/"):
        package_path = package_path.removeprefix("google3/")
    return "/%s/%s.runfiles/_main" % (package_path, binary_label.name)
