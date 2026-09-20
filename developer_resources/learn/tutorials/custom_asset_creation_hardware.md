# Custom Asset Creation (Hardware)

Learn how to build an actuated [Asset](../glossary/intrinsic_terms.md#asset) from traditional 3D CAD models and data sheets. This module walks you through adding a Weiss Robotics GRIPKIT-EASY (WSG 32 parallel gripper) to your Intrinsic Core [Solution](../glossary/intrinsic_terms.md#solution).

## Prerequisites

* Complete the [Import New Part](import_new_part.md) tutorial.
* Download `gripkit_easy_step_customer.zip` from [Weiss Robotics Downloads](https://weiss-robotics.com/gripkit/gripkit-easy/#downloads) along with technical drawings and documentation.

## Actuation approaches covered

* **Approach 1: Digital I/O (DIO) command** — Open and close fingers via 24-volt tool-[flange](../glossary/general_terms.md#flange) digital I/O lines (GRIPKIT-EASY Basic).
* **Approach 2: Native Intrinsic C++ driver (Wsg32Gripper)** — Actuate fingers via Intrinsic's native C++ driver communicating over TCP/IP (port 50042) using the standard `control_pinch_gripper` [Skill](../glossary/intrinsic_terms.md#skill).

### Architecture overview

In Intrinsic Core, hardware peripherals require two parallel layers:

1. **Spatial representation (scene graph)**: The physical 3D mesh, collision hulls, and tool center point (TCP) offset mounted onto the [robot](../glossary/general_terms.md#robot) flange.
2. **Driver actuation pipeline**: The software bridge commanding finger travel.

![Custom Hardware Architecture](../../img/learn/tutorials/custom_hardware_architecture.png)

## Step 1: Configure hardware Asset

Hardware peripherals are defined by:

* `model.sdf`: Multibody physics model, finger [joint](../glossary/general_terms.md#joint) constraints, and optical tool [frame](../glossary/general_terms.md#frame) (`<frame name="tool_frame">`).
* `manifest.textproto`: Platform Asset metadata declaring `type: HARDWARE_DEVICE`.
* `gripper_attachment.updates.pbtxt`: Declarative coordinate transforms mounting the gripper onto `ur_module/flange`.

### Hardware Asset manifest (`manifest.textproto`)

```protobuf
# proto-file: intrinsic/assets/proto/asset_manifest.proto
# proto-message: intrinsic_proto.assets.AssetManifest

id {
  package: "ai.intrinsic.hardware"
  name: "weiss_gripkit_easy"
}
type: HARDWARE_DEVICE
display_name: "Weiss Robotics GRIPKIT-EASY / WSG 32 Gripper"
vendor {
  display_name: "Weiss Robotics"
}
documentation {
  description: "Weiss Robotics GRIPKIT-EASY / WSG 32 parallel gripper supporting Digital I/O and ROS 2 drivers."
}
```

### Model the Weiss gripper in SDFormat (`model.sdf`)

Create `assets/weiss_gripkit/model.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="weiss_gripkit_easy">
    <!-- Main Gripper Body -->
    <link name="gripper_base">
      <inertial>
        <mass>0.850</mass>
        <inertia>
          <ixx>0.0011</ixx><iyy>0.0013</iyy><izz>0.0007</izz>
          <ixy>0.0</ixy><ixz>0.0</ixz><iyz>0.0</iyz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><mesh><uri>meshes/gripkit_base.dae</uri></mesh></geometry>
      </visual>
      <collision name="collision">
        <geometry><mesh><uri>meshes/gripkit_base_collision.stl</uri></mesh></geometry>
      </collision>
    </link>

    <!-- Left Prismatic Jaw -->
    <link name="finger_left">
      <pose relative_to="gripper_base">0 0.020 0.075 0 0 0</pose>
      <inertial>
        <mass>0.065</mass>
        <inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><mesh><uri>meshes/finger_left.dae</uri></mesh></geometry>
      </visual>
      <collision name="collision">
        <geometry><mesh><uri>meshes/finger_left_collision.stl</uri></mesh></geometry>
      </collision>
    </link>

    <!-- Right Prismatic Jaw -->
    <link name="finger_right">
      <pose relative_to="gripper_base">0 -0.020 0.075 0 0 0</pose>
      <inertial>
        <mass>0.065</mass>
        <inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><mesh><uri>meshes/finger_right.dae</uri></mesh></geometry>
      </visual>
      <collision name="collision">
        <geometry><mesh><uri>meshes/finger_right_collision.stl</uri></mesh></geometry>
      </collision>
    </link>

    <!-- Prismatic Joints (34mm travel per finger = 68mm total stroke) -->
    <joint name="finger_left_joint" type="prismatic">
      <parent>gripper_base</parent>
      <child>finger_left</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>0.0</lower><upper>0.034</upper></limit>
      </axis>
    </joint>

    <joint name="finger_right_joint" type="prismatic">
      <parent>gripper_base</parent>
      <child>finger_right</child>
      <axis>
        <xyz>0 -1 0</xyz>
        <limit><lower>0.0</lower><upper>0.034</upper></limit>
      </axis>
    </joint>

    <!-- Optical Tool Center Point (TCP) Frame -->
    <frame name="tool_frame" attached_to="gripper_base">
      <pose>0 0 0.125 0 0 0</pose>
    </frame>
  </model>
</sdf>
```

## Step 2: Package the Asset

1. Create a `BUILD` file in `assets/weiss_gripkit/`:

```python
load("//intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset")
load("//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")

package(default_visibility = ["//visibility:public"])

sdf_scene_object(
    name = "weiss_gripkit_sdf",
    src = "model.sdf",
    sdf_assets = glob(["meshes/**"]),
)

intrinsic_asset(
    name = "weiss_gripkit_easy_asset",
    manifest = "manifest.textproto",
    deps = [":weiss_gripkit_sdf"],
)
```

2. Replace the mounting information in `intrinsic-omts/configs/ur_module.attachments.updates.pbtxt`:

```protobuf
# proto-file: intrinsic/world/public/proto/object_world_updates.proto
# proto-message: intrinsic_proto.world.ObjectWorldUpdates

# Align Gripper Base with UR Flange
updates {
  update_transform {
    node_a { by_name { frame { object_name: "ur_module" frame_name: "flange" } } }
    node_b { by_name { object { object_name: "gripper" } } }
    node_to_update { by_name { object { object_name: "gripper" } } }
    a_t_b {
      position { x: 0.0 y: 0.0 z: 0.0 }
      orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }
    }
  }
}

# Attach Gripper Kinematically to UR Arm
updates {
  reparent_object {
    object { by_name { object_name: "gripper" } }
    new_parent {
      reference { by_name { object_name: "ur_module" } }
      entity_filter { include_final_entity: true }
    }
  }
}

# Define Tool Center Point (TCP) Frame
updates {
  update_transform {
    node_a { by_name { object { object_name: "gripper" } } }
    node_b { by_name { frame { object_name: "gripper" frame_name: "tool_frame" } } }
    node_to_update { by_name { frame { object_name: "gripper" frame_name: "tool_frame" } } }
    a_t_b {
      position { x: 0.0 y: 0.0 z: 0.125 }
      orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }
    }
  }
}
```

## Actuation method 1: Digital I/O (DIO) control

The GRIPKIT-EASY Basic connects directly to the robot tool-flange I/O pins or controller digital lines:

* **Output pin 0 (`standard_out[0]`)**: Open command (HIGH: open fingers; LOW: idle).
* **Output pin 1 (`standard_out[1]`)**: Close command (HIGH: grip workpiece; LOW: idle).
* **Input pin 0 (`standard_in[0]`)**: Part-gripped handshake (HIGH: workpiece securely held; LOW: no part or empty stroke).

### Approach 1, Step 1: Create the SBL Python adapter

Create `//solutions/omts/weiss_dio_gripper.py`:

```python
"""Weiss Robotics GRIPKIT-EASY Digital I/O Adapter for SBL Behavior Trees."""

import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments

class WeissGripkitDio:
    """Controls the Weiss GRIPKIT-EASY using deterministic Digital I/O lines."""

    def __init__(
        self,
        solution,
        open_pin: int = 0,
        close_pin: int = 1,
        part_gripped_pin: int = 0,
        block_out: str = "standard_out",
        block_in: str = "standard_in",
    ):
        self._skills = solution.skills.ai.intrinsic
        self._open_pin = open_pin
        self._close_pin = close_pin
        self._part_gripped_pin = part_gripped_pin
        self._block_out = block_out
        self._block_in = block_in

    def build_open_task(self, name: Optional[str] = None) -> bt.Node:
        """Constructs a Behavior Tree sequence to pulse the Open command."""
        return bt.Sequence(
            name=name or "Weiss_GRIPKIT_Open_Sequence",
            children=[
                self._skills.dio_set_output(
                    block_name=self._block_out,
                    bit_index=self._close_pin,
                    value=False,
                ),
                self._skills.dio_set_output(
                    block_name=self._block_out,
                    bit_index=self._open_pin,
                    value=True,
                ),
            ]
        )

    def build_close_and_verify_task(
        self, timeout_sec: float = 3.0, name: Optional[str] = None
    ) -> bt.Node:
        """Constructs a Behavior Tree sequence to close and verify part grasp."""
        return bt.Sequence(
            name=name or "Weiss_GRIPKIT_Grasp_Sequence",
            children=[
                self._skills.dio_set_output(
                    block_name=self._block_out,
                    bit_index=self._open_pin,
                    value=False,
                ),
                self._skills.dio_set_output(
                    block_name=self._block_out,
                    bit_index=self._close_pin,
                    value=True,
                ),
                # Wait for Weiss GRIPKIT 'Part Grasped' hardware handshake
                self._skills.dio_wait_for_input(
                    block_name=self._block_in,
                    bit_index=self._part_gripped_pin,
                    expected_value=True,
                    timeout_seconds=timeout_sec,
                ),
            ]
        )
```

### Approach 1, Step 2: Test DIO actuation sequence

```python
def main():
    solution = deployments.connect_to_selected_solution()
    executive = solution.executive
    gripper = WeissGripkitDio(solution)

    test_cycle = bt.Sequence(
        name="Weiss_DIO_Test_Cycle",
        children=[
            gripper.build_open_task(),
            gripper.build_close_and_verify_task(),
            gripper.build_open_task(),
        ]
    )

    print("[Weiss DIO] Commanding test cycle...")
    executive.reset()
    executive.run(test_cycle)
    print("[✓] Weiss GRIPKIT DIO cycle completed successfully.")

if __name__ == "__main__":
    main()
```

Expected output:

```text
[Weiss DIO] Commanding test cycle...
[+] Executing: dio_set_output (standard_out[1]=False)... [DONE]
[+] Executing: dio_set_output (standard_out[0]=True)... [DONE] -> Fingers Open.
[+] Executing: dio_set_output (standard_out[1]=True)... [DONE] -> Fingers Closing.
[+] Executing: dio_wait_for_input (standard_in[0]==True)...
[✓] Part grasped signal asserted HIGH (Handshake confirmed).
[✓] Weiss GRIPKIT DIO cycle completed successfully.
```

## Actuation method 2: Native Intrinsic C++ driver (Wsg32Gripper)

Intrinsic includes a native, first-party C++ driver for the Weiss Robotics WSG series (`Wsg32Gripper`), located in `intrinsic/hardware/gripper/wsg32`:

* **Driver class**: `intrinsic::gripper::Wsg32Gripper` (implements `PinchGripperInterface`).
* **Socket client**: `Wsg32Client` wrapping vendor library `@wsg32//wsg_32_driver:libwsg32`.
* **TCP connection**: Connects directly to the Weiss controller over TCP/IP (`IP: 192.168.1.20`, `Port: 50042`).

### Approach 2, Step 1: Configure the driver textproto

Create `configs/wsg32_pinch_gripper.config.textproto`:

```protobuf
# proto-file: google/protobuf/any.proto
# proto-message: google.protobuf.Any

[type.googleapis.com/intrinsic_proto.gripper.PinchGripperPart] {
  config: {
    # Factory settings of the Weiss Robotics WSG32 / GRIPKIT gripper
    generic_pinch_gripper_config: {
      comm_config: {
        # Weiss Robotics default IP and TCP command port
        ip_address: "192.168.1.20"
        port: 50042
      }
      physical_config: {
        position: {
          si_properties: {
            range: { minimum: 0.0 maximum: 0.068 }  # 68 mm stroke
            increment: 0.001
            unit: "meters"
          }
          hardware_value_properties: {
            range: { minimum: 0 maximum: 68 }
            increment: 1
            unit: "mm"
          }
          default_si_value: 0.05
        }
        velocity: {
          si_properties: {
            range: { minimum: 0.005 maximum: 0.400 } # 5 to 400 mm/s
            increment: 0.001
            unit: "m/s"
          }
          hardware_value_properties: {
            range: { minimum: 5 maximum: 400 }
            increment: 1
            unit: "mm/s"
          }
          default_si_value: 0.050
        }
        effort: {
          si_properties: {
            range: { minimum: 5.0 maximum: 50.0 }   # 5 to 50 N
            increment: 1.0
            unit: "Newton"
          }
          hardware_value_properties: {
            range: { minimum: 5 maximum: 50 }
            increment: 1
            unit: "N"
          }
          default_si_value: 40.0
        }
      }
      additional_config: {
        cyclic_routine_frequency_hz: 10.0
        status_topic: "wsg32_gripper/gripper/status"
      }
      # Matches the C++ driver class registered in PinchGripperFactory
      pinch_gripper_driver_name: "Wsg32Gripper"
    }
  }
  grpc_config: {
    grpc_timeout: { seconds: 10 }
  }
}
```

### Approach 2, Step 2: Instantiate the native driver in your Solution

In `assets/weiss_gripkit/BUILD`:

```python
# Instantiate the native WSG 32 driver Asset
intrinsic_asset_instance(
   name = "gripper_instance",
   asset = "ai.intrinsic.wsg32_gripper_strauss",
   config = ":wsg32_pinch_gripper.config.textproto",
   instance_name = "gripper",
)
```

Modify the Solution `BUILD` file:

```python
intrinsic_solution(
    name = "omts_solution",
    assets = [
        # ... other assets
        "//intrinsic/skills/apps/gripper:control_pinch_gripper_skill",
    ],
    instances = [
        # ... other instances
        "//assets/weiss_gripkit:gripper_instance",
    ],
    object_world_updates = [
        "//configs:gripper_attachment.updates.pbtxt",
    ],
)
```

Redeploy the Solution once the `BUILD` file is modified.

### Approach 2, Step 3: SBL Behavior Tree execution via control_pinch_gripper

Because `Wsg32Gripper` implements `PinchGripperInterface`, it is controlled natively using the standard Intrinsic Skill `ai.intrinsic.control_pinch_gripper`.

Create `//solutions/omts/test_wsg32_native.py`:

```python
"""Tests the Weiss WSG 32 / GRIPKIT via the Native Intrinsic C++ Driver."""

import os
# Must be set BEFORE importing any grpc, google.protobuf, or intrinsic modules
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import time
from intrinsic.solutions import deployments
from intrinsic.solutions import behavior_tree as bt

def main():
    # 1. Connect to Intrinsic Core Solution Deployment
    solution = deployments.connect_to_selected_solution()
    executive = solution.executive
    skills = solution.skills.ai.intrinsic

    print("[Weiss WSG 32 Native] Connected to Solution. Preparing actuation commands...")

    # 2. Open fingers to 50 mm (0.050 m) at 50 mm/s with 30 N max force
    open_task = skills.control_pinch_gripper(
        position=0.050,
        speed=0.050,
        force=30.0,
    )

    # 3. Close fingers to grasp workpiece (target 0 mm with 40 N grip force)
    grasp_task = skills.control_pinch_gripper(
        position=0.000,
        speed=0.050,
        force=40.0,
    )

    # 4. Assemble Behavior Tree Sequence: Open -> Grasp -> Open
    test_sequence = bt.Sequence(
        name="Weiss_WSG32_Native_Test_Cycle",
        children=[
            open_task,
            grasp_task,
            open_task,
        ]
    )

    print("[Weiss WSG 32 Native] Dispatching actuation cycle to Executive...")
    executive.reset()
    start_time = time.time()
    
    executive.run(test_sequence)
    
    elapsed = time.time() - start_time
    print(f"[✓] Native Wsg32Gripper actuation cycle completed successfully in {elapsed:.2f} seconds.")

if __name__ == "__main__":
    main()
```

Run the native test:

```bash
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python3 solutions/omts/test_wsg32_native.py
```

Expected output:

```text
[Weiss WSG 32 Native] Connected to Solution. Preparing actuation commands...
[+] Connecting to Weiss WSG controller at 192.168.1.20:50042... Connected.
[+] Executing: control_pinch_gripper(position=0.050m, speed=0.050m/s, force=30.0N)... [DONE]
[+] Executing: control_pinch_gripper(position=0.000m, speed=0.050m/s, force=40.0N)...
[✓] Workpiece gripped at position: 0.0214m | Holding Force: 39.8N | Status: SUCCEEDED
[+] Executing: control_pinch_gripper(position=0.050m, speed=0.050m/s, force=30.0N)... [DONE]
[✓] Native Wsg32Gripper actuation cycle completed successfully in 2.84 seconds.
```

## Comparison: Digital I/O vs. native C++ driver

| Dimension | Approach 1: Digital I/O (DIO) | Approach 2: Native C++ Driver (Wsg32Gripper) |
| :--- | :--- | :--- |
| **Physical Wiring** | 24-volt tool-flange cable | Ethernet TCP/IP (`192.168.1.20:50042`) |
| **Interface Standard** | Discrete Digital I/O Pins | Native Intrinsic gRPC / `PinchGripperInterface` |
| **SBL Skill** | `dio_set_output` / `dio_wait_for_input` | `control_pinch_gripper` |
| **Feedback Resolution** | Binary status (Part Grasped: True/False) | Continuous float position (meters) & force (N) |
| **Stroke Control** | Binary (Full Open / Full Close) | Continuous (0.0 mm to 68.0 mm) |
| **Force Control** | Hardware potentiometer / Fixed | Programmable dynamically per grasp (5 N to 50 N) |
| **Latency** | Extremely low (< 2 ms) | Deterministic socket response (< 5 ms) |
| **External Nodes** | None (pure hardware HAL) | None (native C++ driver in PinchGripperServer) |
| **Best Used For** | Fast cycle time, simple machine tending | Precision assembly, fragile workpieces, dimensional inspection |

## Troubleshooting

| Symptom / Error | Root Cause | Fast Resolution Command |
| :--- | :--- | :--- |
| `Wsg32Client::Connect failed: Connection refused` | Gripper not powered or wrong IP/port | Verify 24V supply to gripper; check Ethernet link; verify IP 192.168.1.20 and port 50042 |
| `inctl icon list-parts does NOT show 'gripper'` | Gripper instance omitted from `instances = [...]` in `intrinsic_solution` | Add `:gripper_instance` to `instances = [...]` in Solution `BUILD` file |
| `PinchGripperFactory: unknown driver 'Wsg32Gripper'` | Target `:wsg32_gripper` omitted from `service/BUILD` deps | Ensure `//intrinsic/hardware/gripper/wsg32:wsg32_gripper` is in deps of `service/BUILD` |
| `dio_wait_for_input timed out after 3.0s` | Workpiece missing or sensitivity threshold incorrect | Check part seating in fixture; adjust Weiss GRIPKIT sensitivity screw |
| `Joint finger_left_joint not found in Gazebo` | Joint name in `model.sdf` does not match control topic | Verify prismatic joint definitions in `assets/weiss_gripkit/model.sdf` |

## Wrap up

In this module, you modeled the Weiss Robotics GRIPKIT-EASY / WSG 32 and kinematically mounted it to the robot flange with an explicit tool center point via `ObjectWorldUpdates`. You then implemented two headless actuation workflows: binary open and close signaling over 24-volt tool-flange digital I/O and variable aperture and force control using Intrinsic's native C++ `Wsg32Gripper` driver.

## Next steps

Return to the [Tutorials index](README.md) or explore [MoveIt Grasp Planning](moveit_grasp_planning.md).
