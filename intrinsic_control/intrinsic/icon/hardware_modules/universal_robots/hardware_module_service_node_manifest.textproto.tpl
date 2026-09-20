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

# proto-file: intrinsic/assets/services/proto/service_manifest.proto
# proto-message: intrinsic_proto.services.ServiceManifest

metadata {
  id {
    package: "ai.intrinsic"
    name: "@ROBOT_ID@_service"
  }
  vendor {
    display_name: "Intrinsic"
  }
  documentation {
    description:
      "A hardware module for the @ROBOT_DISPLAY_NAME@ robot.\n"
      "This module also includes the robot's geometry.\n"
      "Base and tool frame match the UR convention.@CORE_DESCRIPTION@"
  }
  display_name: "Universal Robots @ROBOT_DISPLAY_NAME@@CORE_DISPLAY_SUFFIX@ Hardware Module"
}
