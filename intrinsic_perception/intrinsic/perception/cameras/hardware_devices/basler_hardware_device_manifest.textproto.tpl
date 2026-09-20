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

# proto-file: intrinsic/assets/hardware_devices/proto/v1/hardware_device_manifest.proto
# proto-message: intrinsic_proto.hardware_devices.v1.HardwareDeviceManifest

metadata {
  id {
    package: "ai.intrinsic"
    name: "${asset_id}"
  }
  vendor {
    display_name: "Intrinsic"
  }
  documentation {
    description: "A ${display} camera."
  }
  display_name: "${display}"
  asset_tags: ASSET_TAG_CAMERA
}
graph {
  nodes {
    key: "scene_object"
    value {
      asset: "ai.intrinsic.${scene_object_asset_id}"
    }
  }
  nodes {
    key: "service"
    value {
      asset: "${service_asset_id}"
    }
  }
}
