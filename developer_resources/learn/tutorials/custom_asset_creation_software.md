# Custom Asset Creation (Software)

This tutorial provides a progressive guide to developing, packaging, installing, and executing custom Python [skills](../glossary/intrinsic_terms.md#skill) on Intrinsic Core. You will start by building a minimal "Hello World" skill (`say_skill`) to master the core skill lifecycle and cooperative cancellation. You will then advance to a spatial [digital twin](../glossary/intrinsic_terms.md#digital-twin) skill (`validate_pose`) that interacts directly with the Intrinsic [ObjectWorld](../glossary/intrinsic_terms.md#world) transform graph and reports execution status through the [Behavior Tree](../glossary/intrinsic_terms.md#behavior-tree) engine.

---

## 1. Prerequisites and Environment Setup

Before starting, verify that your development environment has the following:

1. Complete the [Getting Started](getting_started.md) tutorial
1. **[inctl](../glossary/intrinsic_terms.md#inctl-intrinsic-control-cli) CLI**: Installed and accessible in your `$PATH`.
1. **Active [Solution](../glossary/intrinsic_terms.md#solution)**: A running cluster listening on the local Envoy ingress gateway at `localhost:17080`.
1. **Repositories**:
    - [`sdk-examples`](https://github.com/intrinsic-ai/sdk-examples): Cloned into `~/sdk-examples`.
    - [`omts`](https://github.com/intrinsic-ai/intrinsic-omts): Cloned locally: `git clone https://github.com/intrinsic-ai/intrinsic-omts.git`

Verify your environment readiness:

```bash
# Verify inctl is installed and responds
inctl version
# Verify the local gateway is reachable
inctl asset list --address=localhost:17080
# Verify cluster pod health
kubectl get pods -n app-intrinsic-base
```
---

## 2. Tutorial Contents

### Module 1: Build and Deploy a Baseline Skill (`say_skill`)

The `say_skill` provides the canonical starting point for custom skills. It has zero hardware dependencies, and implements cooperative cancellation.

#### 1.1 Inspect the Skill Definition

Navigate to `~/sdk-examples/skills/say_skill`:

- **`say_skill.proto`**: Defines the parameter schema:

```protobuf
syntax = "proto3";
package com.example;
message SaySkillParams {
  string text = 1;
  uint64 wait_ms = 2;
}
```
- **`say_skill.py`**: Inherits from `skill_interface.Skill` and handles cancellation:

```python
from absl import logging
from intrinsic.skills.python import skill_interface
from intrinsic.util.decorators import overrides
class SaySkill(skill_interface.Skill):
  @overrides(skill_interface.Skill)
  def execute(self, request, context):
    # Declare that the skill is initialized and listening for cancel requests
    context.canceller.ready()
    # Wait for the requested duration while watching for cancellation signals
    if context.canceller.wait(request.params.wait_ms / 1000.0):
      raise skill_interface.SkillCancelledError("say_skill was cancelled.")
    logging.info(request.params.text)
```
- **`say_skill_py.manifest.textproto`**: Registers metadata and entry points:

```textproto
id {
  package: "com.example"
  name: "say_skill_py"
}
display_name: "Say skill"
vendor { display_name: "Intrinsic" }
options {
  supports_cancellation: true
  python_config {
    skill_module: "skills.say_skill.say_skill"
    proto_module: "skills.say_skill.say_skill_pb2"
    create_skill: "skills.say_skill.say_skill.SaySkill"
  }
}
parameter { message_full_name: "com.example.SaySkillParams" }
```
- **`BUILD`**: Packages the skill using the `py_skill` macro:

```python
py_skill(
    name = "say_skill_py_image",
    manifest = ":say_skill_py_manifest",
    deps = [
        ":say_skill_py",
        ":say_skill_py_pb2",
    ],
)
```
#### 1.2 Test and Build the [Asset](../glossary/intrinsic_terms.md#asset) Bundle

Run unit tests locally, then compile the containerized OCI asset [bundle](../glossary/intrinsic_terms.md#bundle-bundletar):

```bash
cd ~/sdk-examples
# 1. Run hermetic unit tests
bazel test //skills/say_skill:say_skill_py_test
# 2. Build the skill bundle tarball
bazel build //skills/say_skill:say_skill_py_image
```
The generated artifact is located at `bazel-bin/skills/say_skill/say_skill_py_image.bundle.tar`.

#### 1.3 Install `say_skill` into the Solution

Use `inctl` to upload and install the bundle to the running Solution:

```bash
inctl skill install bazel-bin/skills/say_skill/say_skill_py_image.bundle.tar --address=localhost:17080
```
---

### Module 2: Build a Spatial Validation Skill (`validate_pose`)

Now that you have deployed a baseline skill, advance to `validate_pose`. This skill queries the Intrinsic ObjectWorld (digital twin) scene graph, computes 3D Euclidean distances between [coordinate frames](../glossary/general_terms.md#frame), and enforces geometric [tolerances](../glossary/general_terms.md#tolerance).

#### 2.1 Inspect the Active Scene Graph

Before writing or running spatial skills, inspect the coordinate frames registered in the active solution. Run `inspect_world` from `~/intrinsic-omts`:

```bash
cd ~/intrinsic-omts
bazel run //tools/world:inspect_world
```
Sample output:

```text
Connecting to solution at localhost:17080...
=== Objects in Active Belief World ===
 - root: WorldObject(id=root)
   Frames on root:
     |-> grasp: Frame(id=ofid_2)
     |-> pre_grasp: Frame(id=ofid_3)
     |-> view: Frame(id=ofid_7)
 - ur_module: KinematicObject(id=ofid_4055040002)
   |-> flange: Frame(id=ofid_4055040003)
 - gripper: KinematicObject(id=ofid_784007170)
   |-> tool_frame: Frame(id=ofid_784007171)
=== Transforms Inspection ===
Tool Frame in root: Pose3(Rotation3([...]), [0.4510, 0.3864, 0.8333])
Grasp frame in root: Pose3(Rotation3([...]), [0.1500, 0.2500, 0.7100])
```
The spatial delta between `world.gripper.tool_frame` and `world.root.grasp` is **~0.3528 meters**.

#### 2.2 Inspect the Skill Implementation

Navigate to ~/sdk-examples/skills/validate_pose

Examine `skills/validate_pose/validate_pose.py`:

```python
import numpy as np
from intrinsic.skills.python import skill_interface
from intrinsic.util.decorators import overrides
class ValidatePoseSkill(skill_interface.Skill):
  @overrides(skill_interface.Skill)
  def execute(self, request, context):
    context.canceller.ready()
    # Query the live ObjectWorld transform graph
    actual_to_expected = context.object_world.get_transform(
        request.params.actual_object, request.params.expected_pose
    )
    # Compute Euclidean distance
    translation_error = np.linalg.norm(actual_to_expected.translation)
    if translation_error > request.params.position_tolerance:
      raise ValueError(
          f"Translation error of {translation_error} meters exceeds tolerance"
          f" of {request.params.position_tolerance}"
      )
```
#### 2.3 Build and Install `validate_pose`

```bash
cd ~/sdk-examples
# 1. Run unit tests
bazel test //skills/validate_pose:validate_pose_py_test
# 2. Build the asset bundle
bazel build //skills/validate_pose:validate_pose_py_image
# 3. Install into the running Solution
inctl skill install bazel-bin/skills/validate_pose/validate_pose_py_image.bundle.tar --address=localhost:17080
```
---

### Module 3: Compose Skills in a Behavior Tree (SBL)

The Intrinsic [Solution Building Library](../glossary/intrinsic_terms.md#solution-building-library-sbl) (`intrinsic.solutions`) allows you to connect to the active solution, bind parameters to installed skills, and execute complex workflows through the Behavior Tree [Executive](../glossary/intrinsic_terms.md#executive).

Create an execution script at `tests/progressive_skills_demo.py`:

```python
import sys
from absl import app, logging
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments
def main(argv):
  del argv
  logging.info("Connecting to Solution at localhost:17080...")
  solution = deployments.connect(address="localhost:17080")
  solution.update_skills()
  # Verify skills exist
  for skill_id in ["com.example.say_skill_py", "com.example.validate_pose_py"]:
    if skill_id not in solution.skills:
      logging.error("Required skill %s is not installed!", skill_id)
      sys.exit(1)
  say_skill = solution.skills["com.example.say_skill_py"]
  validate_pose_skill = solution.skills["com.example.validate_pose_py"]
  world = solution.world
  # Bind skill parameters
  greeting_action = say_skill(
      text="Starting automated workcell verification...", wait_ms=200
  )
  # Validate tool frame is within reach of target grasp frame (< 1.0m tolerance)
  validate_action = validate_pose_skill(
      actual_object=world.gripper.tool_frame,
      expected_pose=world.root.grasp,
      position_tolerance=1.0,
      rotation_tolerance=3.14,
  )
  completion_action = say_skill(
      text="Workcell spatial verification passed successfully!", wait_ms=200
  )
  # Compose into a sequential Behavior Tree
  verification_tree = bt.BehaviorTree(
      name="Progressive Skills Verification Tree",
      root=bt.Sequence([
          bt.Task(action=greeting_action, name="Greet Operator"),
          bt.Task(action=validate_action, name="Validate Spatial Alignment"),
          bt.Task(action=completion_action, name="Confirm Verification"),
      ]),
  )
  logging.info("Dispatching Behavior Tree to Executive...")
  solution.executive.run(verification_tree)
  logging.info("Behavior Tree execution finished successfully!")
if __name__ == "__main__":
  app.run(main)
```
Register the binary in `tests/BUILD`:

```python
py_binary(
    name = "progressive_skills_demo",
    srcs = ["progressive_skills_demo.py"],
    srcs_version = "PY3",
    deps = [
        "@ai_intrinsic_sdks//intrinsic/solutions:deployments",
        "@ai_intrinsic_sdks//intrinsic/solutions:solutions_lib",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/logging",
    ],
)
```

Run the demo via Bazel:

```bash
cd ~/sdk-examples
bazel run //tests:progressive_skills_demo
```
---

## 3. Verify Results

Learn to use multiple layers of introspection to debug and confirm skill execution.

### 3.1 Verify via `inctl` CLI

List all installed skills registered with the Solution:

```bash
inctl skill list --address=localhost:17080
```
Expected output includes:

- `com.example.say_skill_py.0.0.1+<hash>`
- `com.example.validate_pose_py.0.0.1+<hash>`

### 3.2 Verify via Kubernetes Pod Status

Check that the runtime [pods](../glossary/general_terms.md#pod) for both skills are Running in the [Kubernetes](../glossary/general_terms.md#kubernetes-k8s) `skills` namespace:

```bash
kubectl get pods -n skills
```
Expected output:

```text
NAME                                       READY   STATUS    RESTARTS   AGE
say-skill-py-com-example-6f765b6f78-w7d9z   1/1     Running   0          5m
validate-pose-py-com-example-fdd9464b5-kk   1/1     Running   0          3m
```
### 3.3 Verify Container Startup Logs

Confirm the internal gRPC skill server is initialized and listening on port 8003:

```bash
kubectl logs -n skills <validate-pose-pod> --tail=10
```
Expected log output:

```text
I0915 23:20:32.758611 skill_service_config_utils.py:40] Reading skill configuration proto from: /skills/skill_service_config.proto.bin
I0915 23:20:32.758870 skill_service_config_utils.py:45] Importing service_config from file.
I0915 23:20:32.967529 skill_init.py:152] Adding skill information server with modular skill validate_pose_py
I0915 23:20:32.969895 skill_init.py:174] -- Skill service listening on [::]:8003
```
### 3.4 Verify Live Execution Logs

Inspect the live execution logs after running the Behavior Tree to verify parameter receipt and calculations:

```bash
kubectl logs -n skills <say-skill-pod> --tail=5
```
Expected log output:

```text
I0915 23:40:12.119780 say_skill.py:25] Starting automated workcell verification...
I0915 23:40:12.450120 say_skill.py:25] Workcell spatial verification passed successfully!
```
---

## 4. How This Fits Into the Platform

Custom skills execute as modular, isolated microservices managed by Intrinsic Core. The diagram below illustrates how client applications, the Behavior Tree Executive, and the ObjectWorld digital twin interact during execution:

![Custom skill architecture](../../img/learn/tutorials/custom_skill_architecture.png)

1. **Client Layer**: Uses SBL to compose tasks and trees without needing to know low-level Kubernetes pod networking.
1. **Platform Gateway (Envoy `17080`)**: Routes incoming requests to internal services.
1. **Executive**: Coordinates Behavior Tree execution, monitors [node](../glossary/intrinsic_terms.md#node) success/failure/running states, and propagates cancellation tokens.
1. **Skill Containers**: Run in isolated namespaces (`skills`), receiving requests via standardized gRPC interfaces generated by `inbuild`.
1. **ObjectWorld**: Maintains the single source of truth for the physical and simulated [robot](../glossary/general_terms.md#robot) workcell state.

---

## 5. Things to Try Next (Next Steps)

After completing this tutorial, explore more advanced skill capabilities:

1. **Implement Dynamic Preemptions / Cancellation**:
    - Modify `say_skill` to run with `wait_ms = 10000` (10 seconds).
    - Compose a parallel Behavior Tree (`bt.Parallel`) with a condition node that cancels `say_skill` halfway through.
    - Confirm that `context.canceller.wait()` triggers the cancellation path and exits cleanly.
1. **Exercise a Skill with Return Values (Output Parameters)**: * Build and deploy one of the SDK examples that returns data to the Behavior Tree blackboard, such as [get_random_number](https://github.com/intrinsic-ai/sdk-examples/tree/main/skills/get_random_number). * Inspect how `get_random_number.proto` defines its `GetRandomNumberResponse` output message and how `get_random_number.py` populates `response.params.random_number`. * Bind the generated output parameter into downstream Behavior Tree tasks using SBL blackboard variables.
1. **Integrate with Hardware Equipment Drivers**: The next tutorial covers how to import [hardware assets](custom_asset_creation_hardware.md) including driver services

## Important architectural note: Machine learning models and stateless Skills

A dedicated tutorial and reference example demonstrating how to integrate custom machine learning models into Intrinsic Core is coming soon, but in the meantime, here is a breakdown of the recommended architecture.


In the Intrinsic platform, the Skill runtime intentionally creates a fresh, stateless instance of your Skill class for every single interaction (get_footprint(), preview() and execute()). Because of this per-interaction lifecycle, loading large model weights inside a Skill's __init__() constructor causes the runtime to re-read files from disk and reconstruct the inference session on every invocation, leading to severe latency spikes and memory churn. Instead, we recommend deploying model inference inside a long-running, persistent [service](../glossary/intrinsic_terms.md#service) (such as a containerized gRPC microservice or Triton Inference Server) that initializes and pins the neural network in GPU or CPU memory upon Solution launch. Your custom Skill should then be authored as a thin, lightweight wrapper whose sole job is to forward inputs and receive inference predictions via fast gRPC or IPC calls to that background service.

---

## 6. Troubleshooting

Address the most common configuration and execution issues:

1. **`AttributeError: 'World' object has no attribute 'gripper'`**
    - **Cause**: The running Solution does not have a gripper resource named `gripper` registered in its scene graph.
    - **Fix**: Run `bazel run //tools/world:inspect_world` from `~/intrinsic-omts` to view the exact registered object names and frame identifiers. Update your script to match the active frames (e.g., `world.ur_module.flange`).
1. **`Execution failed. StatusCode: com.example.validate_pose_py:13`**
    - **Cause**: The spatial distance between the specified frames exceeded `position_tolerance`.
    - **Fix**: Inspect the pod logs (`kubectl logs -n skills <validate-pose-pod>`) to view the actual error distance. Increase `position_tolerance` or jog the robot closer to the target frame.
1. **`inctl skill install: upload failed (404 or connection reset)`**
    - **Cause**: Omitting `--address=localhost:17080` causes `inctl` to attempt connecting to a cloud control plane instead of the local standalone IC cluster.
    - **Fix**: Always append `--address=localhost:17080` when executing `inctl` commands with Intrinsic Core.
1. **`ModuleNotFoundError: No module named 'intrinsic.skills.python'`**
    - **Cause**: Unit tests or binaries were run using standard `python3` instead of the hermetic Bazel toolchain (`bazel test` or `bazel run`).
    - **Fix**: Always execute scripts through Bazel (`bazel run //tests:<target>`) to guarantee dependencies and SDK paths are resolved.
1. **`ProtoSchemaMismatchError: Unknown field name`**
    - **Cause**: Keyword arguments passed in the SBL client script do not match the field names defined in the skill `.proto` file.
    - **Fix**: Inspect the `.proto` parameter message and ensure your SBL Python call uses matching keyword arguments (e.g., `say_skill(text="...", wait_ms=500)`).
1. **`inctl skill install: manifest validation error`**
    - **Cause**: The `message_full_name` declared in `manifest.textproto` does not match the `package` and `message` name in the `.proto` file.
    - **Fix**: Verify that `parameter { message_full_name: "<package>.<MessageName>" }` in `manifest.textproto` exactly matches the package namespace (e.g., `com.example.SaySkillParams`).
1. **`Skill execution hangs indefinitely`**
    - **Cause**: Unhandled infinite loops, blocking I/O, or using `time.sleep()` instead of cooperative cancellation in `execute()`.
    - **Fix**: Always call `context.canceller.ready()` and use `context.canceller.wait(timeout_seconds)` for delays to ensure the Behavior Tree Executive can preempt or cancel the skill.
