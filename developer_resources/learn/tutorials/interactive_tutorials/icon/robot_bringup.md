# Robot Bringup: KUKA KR6 R900-2 & Universal Robots (Native Hardware Modules)

Welcome to the **Native Industrial Robot Bringup** tutorial.

This tutorial guides you through deploying a physical industrial robot using Intrinsic Core's native hardware modules (`intrinsic_hardware_device`) and **ICON (the realtime control service)**. We use the **KUKA AGILUS KR6 R900-2** (via KUKA's low-latency Robot Sensor Interface, **RSI**) as our primary step-by-step walkthrough in Steps 0–4, followed by a complete adaptation guide for **Universal Robots (UR3e, UR5e, and UR10e via RTDE)** in [Adapting This Workflow for Universal Robots](#adapting-this-workflow-for-universal-robots-ur). For an architectural overview of ICON, Hardware Modules, and real-time control concepts, see the [ICON Introduction](icon_introduction/icon_introduction.md).

> [!TIP]
> **Using a Universal Robots (UR) Manipulator?**
> In this tutorial, we demonstrate the declarative Bazel `BUILD` deployment model (`intrinsic_solution` and `intrinsic_asset_instance`) as one way to assemble and deploy a solution—another equally valid approach is the Python **Solution Building Library (SBL)**, which is how the [Open Machine Tending Solution (`intrinsic-omts`)](https://github.com/intrinsic-ai/intrinsic-omts) is built. Because all native hardware modules share the same configuration and deployment patterns under both approaches, the workflow in Steps 0–4 applies directly to **UR3e, UR5e, and UR10e** manipulators. Read through the core concepts below and refer to [Adapting This Workflow for Universal Robots](#adapting-this-workflow-for-universal-robots-ur) for the UR-specific PolyScope checklist, 500 Hz RTDE configurations, and `BUILD` targets.

> [!NOTE]
> **Prerequisites & Workspace Setup**
> Before starting this tutorial, ensure you have:
> 1. **Set up your Bzlmod application workspace** (for example, `~/kuka_bringup/`), following the workspace setup in the [Open Machine Tending Solution (OMTS)](https://github.com/intrinsic-ai/intrinsic-omts/blob/main/README.md). In Intrinsic's Bzlmod architecture, cell-specific configurations (`BUILD`, `.textproto`) and custom skills live in your own standalone application repository—which declares a dependency on `intrinsic-core` under the repository name `@ioc` alongside the required root toolchain and version overrides (see the reference [`MODULE.bazel`](https://github.com/intrinsic-ai/intrinsic-omts/blob/main/MODULE.bazel) and [`.bazelrc`](https://github.com/intrinsic-ai/intrinsic-omts/blob/main/.bazelrc) in `intrinsic-omts`)—keeping your workcell code cleanly separated from the core platform repository.
> 2. **(Optional) Configure a session-local `inctl` shell function ([`env.sh`](files/env.sh))**: In Intrinsic Core, the `inctl` CLI is executed via Bazel (`bazel run @ioc//intrinsic/tools/inctl:inctl_external -- <args>`). You are free to run `bazel run @ioc//intrinsic/tools/inctl:inctl_external -- ...` directly whenever `inctl` is referenced in this tutorial, or you can download [`env.sh`](files/env.sh) into your workspace and source it (`source ./env.sh`) to define a session-local `inctl` shell alias that persists only for your current terminal session without affecting future shells.
> 3. **Provisioned your robot controller**: For KUKA, install the **KUKA.RobotSensorInterface** (RSI) and **KUKA.EthernetKRL** (EKI) option packages following the [Physical & Network Wiring](#physical--network-wiring) and [KUKA Controller Configuration](#kuka-controller-configuration-workvisual-rsi--eki) sections below (for Universal Robots, see the [UR PolyScope Checklist](#1-ur-controller-configuration-checklist-polyscope)).

---

## Target Environments: Enterprise vs. Intrinsic Core

This guide is designed for two primary user personas:
* **Intrinsic Enterprise (Default)**: Developers operating on a shared remote or on-premises industrial cluster with cloud connectivity.
* **Intrinsic Core (Local)**: Open-source ROS and robotics developers operating on a local single-node cluster.

With the declarative Bazel `BUILD` workflow (`intrinsic_solution`) used in this tutorial (as well as with SBL), the bringup process is identical across both environments, differing only in the target cluster flags passed to `bazel run` and `inctl`:

| Parameter | Intrinsic Enterprise (Default) | Intrinsic Core (Local) |
| :--- | :--- | :--- |
| **Cluster Address Flags** | `--cluster <cluster-name> --org <org@project>` | `--address localhost:17080` |
| **Host Real-Time Setup** | Pre-configured during installation on industrial compute nodes | Requires running `setup_realtime.sh` (see below) |
| **KUKA Network Interface** | Configured via Flowstate IPC Manager web UI or `inctl device config` | Configured with static IPs on KUKA RSI & EKI subnets (see below) |
| **KUKA Controller Setup** | KSS 8.x + KUKA.RSI & KUKA.EthernetKRL deployed | KSS 8.x + KUKA.RSI & KUKA.EthernetKRL deployed |

All command examples in this tutorial show Enterprise syntax alongside Intrinsic Core (`--address localhost:17080`) or can be adapted by substituting `--cluster <cluster-name> --org <org@project>` with `--address localhost:17080`.

### Real-Time Host Configuration (Intrinsic Core Only)
When deploying on local PC or workstation hardware using Intrinsic Core, your Linux host must be tuned for `PREEMPT_RT` scheduling priority and CPU core isolation to prevent cycle jitter.

You can run the [`setup_realtime.sh`](../../../../../incode/ioc/setup_realtime.sh) script either from your Bzlmod workspace via `bazel info` or directly from a local `intrinsic-core` checkout:

> [!IMPORTANT]
> **Never invoke `bazel` as `root` (`sudo bazel ...`)**. Running Bazel under `sudo` creates root-owned cache directories inside `~/.cache/bazel`, which will break subsequent non-root builds. Instead, evaluate `bazel info output_base` as your normal user and pipe the script to a root shell:

* **Option 1: From your Bzlmod workspace (via `bazel info`)**
  ```bash
  bazel fetch @ioc//...
  cat "$(bazel info output_base)/external/insrc+/incode/ioc/setup_realtime.sh" | sudo -E bash
  ```

* **Option 2: From a local `intrinsic-core` git checkout**
  ```bash
  sudo ~/intrinsic-core/incode/ioc/setup_realtime.sh
  ```

**Reboot your machine (`sudo reboot`) after the script completes.** If you fail to run this script and reboot into the real-time kernel parameters, the hardware module will fail to start with:
`Hardware module main failed: FAILED_PRECONDITION: Failed to parse [/proc/cmdline]`.

### Physical & Network Wiring

1. **Ethernet Connection (`KLI` Port)**:
   Connect a RJ45 CAT6 Ethernet cable from the dedicated real-time (non-EtherCAT) network port on your host PC / IPC to the **KUKA Line Interface (`KLI`)** port on the KUKA Robot Controller (KRC):
   * **KUKA KR C4**: Connect to port **`X66`**. *(For a Windows PC running KUKA WorkVisual via DHCP, connect to port `X69`.)*
   * **KUKA KR C5**: Connect to port **`XF1`**.

2. **External Control Physical I/O Loopback (`EXT` Mode)**:
   To enable drives, clear faults, and start the RSI program automatically from an external controller over EthernetKRL (EKI) in **`EXT` (External)** mode, KUKA requires physical digital outputs to be wired back to physical digital inputs and mapped to KR C I/O indices **`421–423`**.
   * **Recommended EtherCAT I/O Hardware**: Connect a Beckhoff **EK1100** EtherCAT coupler equipped with an **EL1008** (8-channel digital input) terminal and an **EL2008** (8-channel digital output) terminal to the EtherCAT port (**`X65`**) on the KUKA controller.
   * **Physical Loopback Wiring & KRC Signal Mapping**: Using three jumper wires, connect outputs 1–3 of the `EL2008` directly to inputs 1–3 of the `EL1008`. The Intrinsic Extended option package maps these channels to KR C inputs/outputs `421–423`:

     | KUKA System Signal | Function | KR C Input (`EL1008`) | Wired From KR C Output (`EL2008`) |
     | :--- | :--- | :--- | :--- |
     | **`$MOVE_ENABLE`** | General motion enable (must be `1` / active) | `$IN[421]` (EL1008 Input 1) | $\leftarrow$ `$OUT[421]` (EL2008 Output 1) |
     | **`$DRIVES_ON`** | Rising-edge pulse to switch drives on | `$IN[422]` (EL1008 Input 2) | $\leftarrow$ `$OUT[422]` (EL2008 Output 2) |
     | **`$CONF_MESS`** | Pulse to acknowledge and clear controller faults | `$IN[423]` (EL1008 Input 3) | $\leftarrow$ `$OUT[423]` (EL2008 Output 3) |

     ![I/O wiring diagram for external KUKA control](files/kuka_eki_io_wiring.png)

     ![I/O wiring for external KUKA control](files/kuka_io_wiring.png)

   * *(Other KUKA-compatible I/O modules may also be used, provided three physical outputs are wired to three physical inputs and mapped to `KR C I/Os` indices `421–423` in WorkVisual.)*

### Network Configuration: Connecting to the KUKA Controller
KUKA controllers separate real-time motion control (**RSI**, UDP) and non-real-time status/control (**EKI**, TCP) onto two distinct virtual network interfaces (`virtual6` / RSI and `KLI` / EKI) on different subnets. Consequently, your host computer's dedicated robot Ethernet interface must be configured with **two static IP addresses** (one on each subnet):
* **RSI Subnet (Real-Time UDP)**: Configure the host interface with e.g. `192.168.10.123/24` to communicate with the KRC's RSI virtual NIC (`192.168.10.122/24`).
* **EKI Subnet (Non-Real-Time TCP)**: Configure a secondary IP/alias on the same host interface with e.g. `192.168.11.123/24` to communicate with the KRC's KLI/EKI interface (`192.168.11.122/24`).
* **Host Setup Instructions**:
  * **Intrinsic Core (Ubuntu Host)**: Assign both static IPs (`192.168.10.123/24` and `192.168.11.123/24`) to your robot-facing NIC via [Ubuntu Netplan](https://ubuntu.com/server/docs/explanation/networking/configuring-networks/) or the [Ubuntu Network GUI](https://help.ubuntu.com/stable/ubuntu-help/net-manual.html.en).
  * **Intrinsic Enterprise (IPC)**: Configure both static IPs (`192.168.10.123/24` and `192.168.11.123/24`) and set `"realtime": true` on your robot-facing NIC using `inctl device config set` (or via the IPC Manager).
* **Connection Verification**: Test physical layer and IP reachability to both controller interfaces by pinging the KUKA controller from the host machine:
  ```bash
  ping 192.168.10.122  # KRC RSI virtual NIC
  ping 192.168.11.122  # KRC KLI/EKI NIC
  ```

### KUKA Controller Configuration (WorkVisual, RSI & EKI)

Before connecting Intrinsic Core to physical hardware, your KUKA controller must be provisioned with KUKA's RSI and EKI option packages along with the Intrinsic controller configuration files.

#### 1. Required KUKA System Software & Option Packages
* **KR C4 Controller**: KUKA System Software (KSS) **8.6**, **KUKA.RobotSensorInterface (RSI) 4.x**, and **KUKA.EthernetKRL (EKI) 3.1**.
* **KR C5 Controller**: KUKA System Software (KSS) **8.7**, **KUKA.RobotSensorInterface (RSI) 5.0**, and **KUKA.EthernetKRL (EKI) 3.2**.

#### 2. Download & Install the Intrinsic KUKA Option Packages (`.kop`)
Intrinsic provides two WorkVisual option packages (`.kop`) and standalone reference configuration files located in [`files/`](files/):
* [**`IntrinsicBase-1.1.1.kop`**](files/IntrinsicBase-1.1.1.kop) (**Base Option Package**): Installs the RSI context ([`RSIContext.rsix`](files/RSIContext.rsix)), the RSI XML protocol definition ([`RSIEthernetConfig.xml`](files/RSIEthernetConfig.xml)), the KRL motion program ([`RSI_MC.src`](files/RSI_MC.src)), two EKI protocol XML definitions, and the `intrinsic.sub` submit interpreter for EKI status/control.
* [**`IntrinsicExtended-1.1.0.kop`**](files/IntrinsicExtended-1.1.0.kop) (**Extended Option Package**): Builds on the Base package to automatically configure the `RSI` and `KLI` virtual network interfaces, bind the system signals `$MOVE_ENABLE`, `$DRIVES_ON`, and `$CONF_MESS` to `$IN[421..423]`, provide the `Intrinsic I/Os using EK1100, EL1008, EL2008` bus configuration, and start the Intrinsic submit interpreter at position 3.

> [!CAUTION]
> **Back Up Your WorkVisual Project First**
> Always save a backup of your existing WorkVisual project before installing or updating option packages. `IntrinsicExtended-1.1.0.kop` overwrites existing `KLI`/`RSI` virtual NIC configurations, the first user-defined submit interpreter (position 3), and the signal bindings for `$DRIVES_ON`, `$MOVE_ENABLE`, and `$CONF_MESS`.

To configure your controller in **KUKA WorkVisual 6.0**:
1. Open **Extras $\rightarrow$ Option Package Management** in WorkVisual and install both [`IntrinsicBase-1.1.1.kop`](files/IntrinsicBase-1.1.1.kop) and [`IntrinsicExtended-1.1.0.kop`](files/IntrinsicExtended-1.1.0.kop) into your active KRC project.
2. If using the recommended Beckhoff EK1100 + EL1008 + EL2008 loopback hardware, drag the **`Intrinsic I/Os using EK1100, EL1008, EL2008`** component from the Options catalog onto the controller in WorkVisual so that the 3 loopback inputs and outputs are mapped to `KR C I/Os` indices **`421–423`**.

   ![Initial RSI context](files/rsi_context.png)

   ![WorkVisual I/O mapping example](files/wov_io_mapping_example_kr6.png)

   ![WorkVisual RSI context I/O mapping example](files/wov_io_rsicontext_example_kr6.png)

3. Verify the KRC virtual network interfaces in WorkVisual:
   * **RSI virtual NIC (`virtual6`, UDP real-time)**: Set to `192.168.10.122` (subnet mask `255.255.255.0`). *(Note: `IntrinsicExtended` defaults to the `192.170.10.x` / `192.170.11.x` subnets; adjust either the KRC virtual NIC IPs in WorkVisual to `192.168.10.122` / `192.168.11.122` or use `192.170.10.x` / `192.170.11.x` consistently across both the KRC and host PC.)*
   * **KLI virtual NIC (TCP non-real-time / EKI)**: Set to `192.168.11.122` (subnet mask `255.255.255.0`).
4. In the project's `C:\Config\User\Common\SensorInterface\RSIEthernetConfig.xml` file (reference: [`RSIEthernetConfig.xml`](files/RSIEthernetConfig.xml)), verify that `<IP_NUMBER>` and `<PORT>` match your host PC's RSI IP address (`192.168.10.123`) and UDP port (`5852`):
   ```xml
   <CONFIG>
     <!-- IP address and UDP port of the Intrinsic host PC on the RSI subnet -->
     <IP_NUMBER>192.168.10.123</IP_NUMBER>
     <PORT>5852</PORT>
     <SENTYPE>Intrinsic</SENTYPE>
     <ONLYSEND>FALSE</ONLYSEND>
   </CONFIG>
   ```
   *(Note: `RSIEthernetConfig.xml` also defines 4 digital inputs `DigIn.DI1..DI4` and 2 digital outputs `DigOut.DO1..DO2` by default, which must match the number of entries in `digital_input_names` and `digital_output_names` in `kuka_rsi_config.textproto` in [Step 2](#step-2-creating-the-kuka-hardware-configuration-kuka_rsi_configtextproto)).*
5. **Deploy the WorkVisual project** to the KUKA controller and activate it on the SmartPAD.

#### 3. Enabling the Robot on the KUKA SmartPAD
* **Standard EKI Mode (`EXT`)**: With the physical I/O loopback wired (`421–423`) and `eki_control_client_params` configured, simply turn the physical mode selector switch on the KUKA SmartPAD to **`EXT` (External)**. When the Intrinsic hardware module starts, it automatically clears faults, enables drives, and selects/runs [`RSI_MC.src`](files/RSI_MC.src).
* **Manual / Teach Pendant Mode (Without Loopback I/O)**: If you are testing without the physical I/O loopback (`421–423`), omit `eki_control_client_params` from `kuka_rsi_config.textproto` and start [`RSI_MC.src`](files/RSI_MC.src) manually on the SmartPAD:
  1. Release all emergency stops and confirm all messages on the SmartPAD.
  2. Turn the key switch to **`AUT`** (or **`T1`**).
  3. Navigate to `R1/Program`, highlight **`RSI_MC`**, and press **Select** (do *not* press *Open*).

     ![Select the RSI_MC program](files/kuka_select_program.png)

  4. Enable the drives by pressing the **`O`** drives status icon at the top of the screen and pressing **`I`** until it turns green.

     ![Enable the drives](files/kuka_teach_pendant_enable_drives.png)

  5. Hold/press the green **Play** button until the program pointer reaches `RSI_MOVECORR()`.

---

## Step 0: Stop Any Running Solution

Because we will declare and deploy our complete robot solution (`robot` hardware device + `icon` realtime controller) declaratively from our Bzlmod workspace in [Step 4](#step-4-declaring-and-deploying-the-solution-via-build), you do **not** need to pre-create an empty cloud solution or manually install individual asset bundles beforehand.

To start from a clean, identical baseline on both **Intrinsic Enterprise** and **Intrinsic Core**, simply ensure that **no solution is currently running** on your target cluster:

```bash
# Intrinsic Enterprise:
inctl solution stop --cluster <cluster-name> --org <org@project>

# Intrinsic Core (Local):
inctl solution stop --address localhost:17080
```

> [!IMPORTANT]
> **Physical Execution Mode (`--operation_mode=real`)**
> When we deploy our solution in [Step 4](#step-4-declaring-and-deploying-the-solution-via-build), we will pass `--operation_mode=real` so the solution launches in **physical hardware execution mode** (`RUNNING_ON_HW`) rather than simulation (`--operation_mode=sim`). Stopping any prior solution now ensures the cluster is clear of previous simulation or hardware sessions.

---

## Step 1: Locating KUKA Hardware Devices in `@ioc`

Intrinsic Core encapsulates robot integrations into catalog assets known as **Hardware Devices** (`intrinsic_hardware_device`). A Hardware Device bundles two complementary components into a single deployable asset:
1. **Scene Object (`intrinsic_scene_object`)**: Contains the 3D meshes, URDF/SDFormat kinematic chains, joint limits, and inverse kinematics (IK) solver definitions.
2. **Service (`intrinsic_service`)**: The containerized daemon that runs the low-level real-time driver communicating with the physical robot controller.

All pre-configured KUKA RSI hardware devices are defined in [`incode/intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi/BUILD`](../../../../../incode/intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi/BUILD).

From your Bzlmod workspace (e.g., `~/kuka_bringup`), query the available KUKA hardware device targets in `@ioc` using Bazel:

```bash
bazel query 'kind("intrinsic_hardware_device", @ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:*)'
```

*Expected Output:*
```text
@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr10_r1100_2_hardware_module
@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr16_r2010_2_hardware_module
@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr20_r1810_2_hardware_module
@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr50_r2500_hardware_module
@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr6_r900_2_fake_hardware_module
@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr6_r900_2_hardware_module
@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr6_r900_2_loopback_hardware_module
```

You will find definitions for several popular KUKA robot models:

| Robot Model | Hardware Device Target (`@ioc`) | Asset ID | Payload / Reach |
| :--- | :--- | :--- | :--- |
| **KUKA KR6 R900-2 (AGILUS)** | `@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr6_r900_2_hardware_module` | `ai.intrinsic.kuka_kr6_hardware_module` | 6 kg / 901 mm |
| **KUKA KR10 R1100-2 (AGILUS)** | `@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr10_r1100_2_hardware_module` | `ai.intrinsic.kuka_kr10_hardware_module` | 10 kg / 1101 mm |
| **KUKA KR16 R2010-2 (Cybertech)** | `@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr16_r2010_2_hardware_module` | `ai.intrinsic.kuka_kr16_hardware_module` | 16 kg / 2013 mm |
| **KUKA KR20 R1810-2 (Cybertech)** | `@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr20_r1810_2_hardware_module` | `ai.intrinsic.kuka_kr20_hardware_module` | 20 kg / 1810 mm |
| **KUKA KR50 R2500 (IONTEC)** | `@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr50_r2500_hardware_module` | `ai.intrinsic.kuka_kr50_hardware_module` | 50 kg / 2501 mm |

The underlying geometric models, meshes, and joint limit textprotos for these robots are located under [`incode/intrinsic_control/intrinsic/models/robot_definitions/kuka`](../../../../../incode/intrinsic_control/intrinsic/models/robot_definitions/kuka). For our target robot, the kinematic description lives in [`incode/intrinsic_control/intrinsic/models/robot_definitions/kuka/kr6_r900_2`](../../../../../incode/intrinsic_control/intrinsic/models/robot_definitions/kuka/kr6_r900_2).

---

## Step 2: Creating the KUKA Hardware Configuration (`kuka_rsi_config.textproto`)

The runtime driver for KUKA RSI is parameterized via a `HardwareModuleConfig` Protocol Buffer configuration file. While the KUKA hardware module ships with a generic reference template ([`kuka_rsi_default_config.textproto`](../../../../../incode/intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_default_config.textproto)) designed for default simulation and standalone service setups, bringing up a physical robot cell requires providing a cell-specific `service_config` file in your workspace so you can:
1. **Match your physical cell's network topology and I/O mapping**: Configure the exact host and controller IP addresses (`local_rsi_host: "192.168.10.123"`, `eki_address: "192.168.11.122"`) and digital I/O channel names wired in your workcell.
2. **Automatically bind to your declarative `BUILD` instance name**: By omitting the explicit module `name` override present in the standalone template, the driver automatically inherits the `intrinsic_asset_instance` name (`name = "robot"`) declared in your `BUILD` file—ensuring the hardware module, scene object, and ICON `MainConfig` (`hardware_module_names: ["robot"]`) remain seamlessly aligned.

In your Bzlmod workspace directory (`~/kuka_bringup/`), create `kuka_rsi_config.textproto`:

```textproto
# proto-file: intrinsic/icon/hal/proto/hardware_module_config.proto
# proto-message: intrinsic_proto.icon.HardwareModuleConfig

[type.googleapis.com/intrinsic_proto.icon.HardwareModuleConfig] {
  drives_realtime_clock: true
  control_frequency_hz: 250
  module_config {
    [type.googleapis.com/intrinsic_proto.icon.KukaRsiModule] {
      local_rsi_host: "192.168.10.123"
      local_rsi_port: 5852
      velocity_settling_cutoff_frequency: 100
      digital_input_names: ["di_1", "di_2", "di_3", "di_4"]
      digital_output_names: ["do_1", "do_2"]
      eki_status_client_params: {
        eki_address: "192.168.11.122"
        eki_port: 54600
        update_frequency: 15  # Hz
      }
      eki_control_client_params: {
        eki_address: "192.168.11.122"
        eki_port: 54601
        update_frequency: 2   # Hz
      }
    }
  }
}
```

### Configuration Breakdown & Parameter Effects

* **`drives_realtime_clock: true`**:
  * **Effect**: Designates this module as the master real-time clock provider for ICON. When set to `true`, ICON does not use an internal monotonic timer; instead, each cycle of the real-time loop is triggered by the arrival of a UDP packet from the KUKA RSI interface.
* **`control_frequency_hz: 250`**:
  * **Effect**: Dictates the expected fieldbus cycle rate. Standard KUKA RSI operates with a deterministic 4 ms tick interval, corresponding to exactly **250 Hz**.
* **`local_rsi_host` and `local_rsi_port`**:
  * **Effect**: Defines the local IP address and UDP port on the host computer where the driver listens for incoming RSI XML packets. This must match the `<IP_NUMBER>` and `<PORT>` specified in the KUKA controller's [`RSIEthernetConfig.xml`](files/RSIEthernetConfig.xml) file (deployed as described in [KUKA Controller Configuration](#kuka-controller-configuration-workvisual-rsi--eki) on the subnet established in [Network Configuration](#network-configuration-connecting-to-the-kuka-controller)).
* **`velocity_settling_cutoff_frequency: 100`**:
  * **Effect**: Configures a first-order low-pass filter (in Hz) applied to finite-difference velocity estimates. This filters out high-frequency quantization noise from discrete encoder readings without adding significant phase lag.
* **`eki_status_client_params` & `eki_control_client_params`**:
  * **Effect**: Configures EthernetKRL (EKI) communication channels to the KRC's KLI IP address (`192.168.11.122`) on TCP ports `54600` and `54601`. EKI handles non-real-time operations such as querying controller operational mode, reading safety states, enabling drives, and resetting alarm conditions at 15 Hz and 2 Hz respectively.
* **`digital_input_names` & `digital_output_names`**:
  * **Effect**: Exposes controller digital I/O lines to the Intrinsic HAL, allowing digital gripper actuation or fixture handshakes to be commanded synchronously. The number of inputs (4) and outputs (2) must match the `<ELEMENT TAG="DigIn.DI..." />` and `<ELEMENT TAG="DigOut.DO..." />` definitions in [`RSIEthernetConfig.xml`](files/RSIEthernetConfig.xml).

---

## Step 3: Creating the ICON MainLoop Configuration (`kuka_icon_main_config.textproto`)

When the `robot` hardware device runs, it streams raw joint positions and listens for real-time setpoints over shared memory. To execute coordinated trajectories, the hardware module must be claimed by **ICON (the realtime control service)** (`@ioc//intrinsic_control/intrinsic/icon/machines/common:generic_icon_mainloop_type`). For a deeper overview of ICON's architecture, parts, and real-time execution model, see the [ICON Introduction](icon_introduction/icon_introduction.md) tutorial.

In your Bzlmod workspace directory (`~/kuka_bringup/`), save the following configuration file as `kuka_icon_main_config.textproto`:

```textproto
# proto-file: google/protobuf/any.proto
# proto-message: google.protobuf.Any

[type.googleapis.com/intrinsic_proto.icon.IconMainConfig] {
  intrinsic_runtime {
    name: "intrinsic_runtime"
  }

  # Match the 4 ms KUKA RSI fieldbus clock (250 Hz)
  control_frequency_hz: 250.0

  # Real-time I/O timeout threshold
  hardware_module_read_write_timeout_seconds: 1

  # Connect to world and kinematics services
  services {
    world_service_from_grpc { world_id: "world" }
    kinematics_from_world_service: true
    assembly_from_world_service: true
  }

  # Register the hardware module instance name ("robot") declared in Step 4
  hardware_module_names: ["robot"]

  # Instruct ICON to clock its control ticks directly to KUKA RSI UDP packet arrivals
  hardware_module_that_drives_clock: "robot"

  realtime_control_config {
    parts_by_name {
      key: "arm"
      value {
        part_type_name: "HalArmPart"
        safety_action_type_name: "intrinsic.stop"
        hardware_resource_name: "robot"
        config {
          [type.googleapis.com/intrinsic_proto.icon.HalArmPartConfig] {
            joint_position_command { module_name: "robot" interface_name: "joint_position_command" }
            joint_position_state   { module_name: "robot" interface_name: "joint_position_state" }
            joint_velocity_state   { module_name: "robot" interface_name: "joint_velocity_state" }
          }
        }
      }
    }
    parts_by_name {
      key: "adio"
      value {
        part_type_name: "HalADIOPart"
        safety_action_type_name: "intrinsic.empty"
        config {
          [type.googleapis.com/intrinsic_proto.icon.HalADIOPartConfig] {
            digital_outputs { interface { module_name: "robot" interface_name: "digital_output_command" } export_name: "outputs" }
            digital_inputs  { interface { module_name: "robot" interface_name: "digital_input_status" } export_name: "inputs" }
            # The KUKA RSI hardware module echoes current physical output states back as inputs
            digital_inputs  { interface { module_name: "robot" interface_name: "digital_output_status" } export_name: "current_outputs" }
          }
        }
      }
    }
  }
}
```

### Parameter Context: What Goes Where and Why

* **`intrinsic_runtime { name: "intrinsic_runtime" }`**:
  * Configures the service discovery handle used by ICON to connect to the platform services mesh.
* **`control_frequency_hz: 250.0`**:
  * Must match the hardware module's `control_frequency_hz` (250 Hz). Any mismatch between the main loop frequency and the fieldbus cycle will trigger timing jitter warnings.
* **`hardware_module_read_write_timeout_seconds: 1`**:
  * Defines the timeout threshold before reporting a read/write fault if the hardware module stops responding.
* **`services`**:
  * **`world_service_from_grpc { world_id: "world" }`**: Connects ICON to the World Service, providing scene object kinematics and geometry.
  * **`kinematics_from_world_service: true` & `assembly_from_world_service: true`**: Instructs ICON to query robot kinematics models and application joint limits directly from the world model.
* **`hardware_module_names: ["robot"]`**:
  * Tells ICON which hardware modules' shared-memory namespaces to connect to upon startup. The string `"robot"` must match the `name = "robot"` attribute of the `intrinsic_asset_instance` target declared in [Step 4](#step-4-declaring-and-deploying-the-solution-via-build).
* **`hardware_module_that_drives_clock: "robot"`**:
  * Informs the ICON execution manager that hardware module `"robot"` drives the real-time clock. When the KUKA controller transmits an RSI packet, ICON immediately wakes to process sensor inputs, compute actions, and reply with next-cycle joint targets.
* **`parts_by_name { key: "arm" ... }` & `hardware_resource_name: "robot"`**:
  * Exposes the robot arm as a high-level logical part named `"arm"` and binds `hardware_resource_name: "robot"` to the corresponding robot resource instance in the solution's World Service (`name = "robot"`). Motion planning services and client applications claim `"arm"` to command 6-DoF joint trajectories.
* **`HalArmPartConfig`**:
  * Maps the abstract arm part's command and state channels to the concrete hardware interfaces exposed by the `"robot"` KUKA RSI driver (`joint_position_command`, `joint_position_state`, and `joint_velocity_state`).
* **`parts_by_name { key: "adio" ... }`**:
  * Exposes the robot controller's digital I/O lines as an analog/digital I/O part named `"adio"`.
* **`HalADIOPartConfig`**:
  * Maps logical I/O signals to the `"robot"` KUKA RSI driver interfaces:
    * `outputs`: Commands digital outputs via the `digital_output_command` interface.
    * `inputs`: Reads digital inputs via the `digital_input_status` interface.
    * `current_outputs`: Reads the actual commanded state of digital outputs echoed back by KUKA RSI via `digital_output_status`.

---

## Step 4: Declaring and Deploying the Solution via `BUILD`

With `kuka_rsi_config.textproto` and `kuka_icon_main_config.textproto` saved in your Bzlmod workspace (`~/kuka_bringup/`), you can now bind both configurations to their respective upstream `@ioc` assets using `intrinsic_asset_instance` and bundle them into a complete solution using `intrinsic_solution`.

### 1. Create the `BUILD` File

Create a `BUILD` file in `~/kuka_bringup/BUILD`:

```python
load("@ioc//intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("@ioc//intrinsic/assets/build_defs:solution.bzl", "intrinsic_solution")

intrinsic_asset_instance(
    name = "robot",
    asset = "ai.intrinsic.kuka_kr6_hardware_module",
    instance_name = "robot",
    service_config = ":kuka_rsi_config.textproto",
)

intrinsic_asset_instance(
    name = "icon",
    asset = "ai.intrinsic.generic_realtime_control_service",
    instance_name = "icon",
    service_config = ":kuka_icon_main_config.textproto",
)

intrinsic_solution(
    name = "kuka_solution",
    add_compose_world_test = False,
    assets = [
        "@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr6_r900_2_hardware_module",
        "@ioc//intrinsic_control/intrinsic/icon/machines/common:generic_icon_mainloop_type",
    ],
    default_operation_mode = "real",
    instances = [
        ":robot",
        ":icon",
    ],
)
```

At build time:
* `intrinsic_solution` bundles the upstream `@ioc` asset targets listed in `assets` (`kr6_r900_2_hardware_module` and `generic_icon_mainloop_type`).
* `intrinsic_asset_instance` binds your `.textproto` files (`service_config`) to their respective Asset IDs (`ai.intrinsic.kuka_kr6_hardware_module` and `ai.intrinsic.generic_realtime_control_service`) under the instance names `robot` and `icon`.

### 2. Deploy the Solution to the Cluster (`--operation_mode=real`)

Deploy `//:kuka_solution` directly to your target cluster in physical hardware mode (`--operation_mode=real`) with a single `bazel run` command:

```bash
# Intrinsic Enterprise:
bazel run //:kuka_solution -- \
  --cluster <cluster-name> --org <org@project> --operation_mode=real

# Intrinsic Core (Local):
bazel run //:kuka_solution -- \
  --address localhost:17080 --operation_mode=real
```

*(Tip: To test the exact same solution in kinematic simulation without physical robot hardware, simply pass `--operation_mode=sim` instead of `--operation_mode=real`.)*

*Expected Output:*
```text
INFO: Running command line: bazel-bin/kuka_solution ...
Deploying to cluster
Released solution successfully.
```

### 3. Verify Active Asset Instances and ICON Controller Status

First, confirm that both the `robot` hardware device and `icon` realtime control service instances are active on the cluster:

```bash
# Intrinsic Enterprise:
inctl asset instances list --cluster <cluster-name> --org <org@project>

# Intrinsic Core (Local):
inctl asset instances list --address localhost:17080
```

*Expected Output:*
```text
Name  Asset
icon  ai.intrinsic.generic_realtime_control_service
robot ai.intrinsic.kuka_kr6_hardware_module
```

Finally, verify that ICON has established real-time lockstep with the KUKA RSI clock and reports `ENABLED`:

```bash
# Intrinsic Enterprise:
inctl icon status --instance_name icon \
  --cluster <cluster-name> --org <org@project>

# Intrinsic Core (Local):
inctl icon status --instance_name icon \
  --address localhost:17080
```

*Expected Output:*
```text
Operational Status: ENABLED

Part Statuses:
  adio:
    State: READY
  arm:
    State: READY

Safety Status:
  Mode of Safe Operation: MODE_OF_SAFE_OPERATION_T1
  E-Stop Button Status:   BUTTON_STATUS_RELEASED
  Enable Button Status:   BUTTON_STATUS_PRESSED
  Requested Behavior:     REQUESTED_BEHAVIOR_ENABLED
```

---

## Adapting This Workflow for Universal Robots (UR)

Because Intrinsic provides native hardware modules across multiple robot brands, bringing up a **Universal Robots e-Series manipulator (UR3e, UR5e, or UR10e)** follows the exact same declarative `BUILD` workflow covered in Steps 0 through 4 (see [`incode/intrinsic_control/intrinsic/icon/hardware_modules/universal_robots/BUILD`](../../../../../incode/intrinsic_control/intrinsic/icon/hardware_modules/universal_robots/BUILD)).

To control a UR robot, follow this tutorial while substituting the UR-specific settings and targets below:

| Workflow Step | KUKA KR6 R900-2 (Baseline) | Universal Robots (UR3e / UR5e / UR10e) |
| :--- | :--- | :--- |
| **Controller Prep** | Install `IntrinsicBase` / `IntrinsicExtended` `.kop` packages & configure `virtual6` UDP interface | Configure PolyScope (Remote Control, RTDE/Dashboard services, disable Ethernet Fieldbus) — see [checklist below](#1-ur-controller-configuration-checklist-polyscope) |
| **Real-Time Protocol** | KUKA RSI (UDP) @ **250 Hz** (4 ms cycle) | UR RTDE (TCP/IP) @ **500 Hz** (2 ms cycle) |
| **Hardware Device Target (`@ioc`)** | `@ioc//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr6_r900_2_hardware_module` | `@ioc//intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_ioc` *(or `:ur3e_...` / `:ur10e_...`)* |
| **Asset ID** | `ai.intrinsic.kuka_kr6_hardware_module` | `ai.intrinsic.ur5e_hardware_module_ioc` *(or `ur3e` / `ur10e`)* |
| **Driver Config (`service_config`)** | `kuka_rsi_config.textproto` (`KukaRsiModule`) | `ur_config.textproto` (`UniversalRobotsModuleConfig`) |
| **ICON Config (`service_config`)** | `kuka_icon_main_config.textproto` (250 Hz, single I/O bank) | `ur_icon_main_config.textproto` (500 Hz, split `standard`/`configurable`/`tool` I/O banks) |

### 1. UR Controller Configuration Checklist (PolyScope)

Before starting the UR hardware module, perform the following one-time setup on the UR Control Box and Teach Pendant (switch the Teach Pendant to **`Manual`** mode in the top-right corner first so configuration menus are editable):

1. **Physical Network & Static IP Configuration**:
   * Connect an RJ45 CAT6 Ethernet cable directly from the UR control box to the dedicated real-time (**non-EtherCAT**) network port on your host PC / IPC.
   * On the UR Teach Pendant, open **`Settings | System | Network`**, select **Static Address**, and configure an IP on your real-time subnet (for example, IP address `192.168.10.1`, subnet mask `255.255.255.0`, default gateway `0.0.0.0`, with your host PC NIC set to `192.168.10.123/24`).
2. **PolyScope Firmware**:
   * Ensure your UR e-Series controller is running **PolyScope 5.9.4 or newer**.
3. **Enable Remote Control Mode**:
   * Navigate to **`Settings | System | Remote Control`** and click **Enable**. This adds the `Local / Remote` toggle in the top-right header of the Teach Pendant.
   * Switch the Teach Pendant operational mode from `Local` (or `Manual`) to **`Remote`** before deploying the solution. *(If the toggle button is disabled, close any open configuration dialog first.)*
4. **Enable Required Network Services**:
   * Navigate to **`Settings | Security | Services`**, unlock the settings with your Admin password, and ensure the following three interfaces are **Enabled**:
     * **Dashboard Server** (TCP port `29999`)
     * **Primary Client Interface** (TCP port `30001`)
     * **Real-Time Data Exchange (RTDE)** (TCP port `30004`)
   * Lock the settings and click **Exit**. *(Leaving Secondary Client Interface or Real-Time Client Interface enabled will not cause conflicts.)*
5. **Disable Ethernet Fieldbuses**:
   * In the **`Installation`** tab under **`Fieldbus`** (`Fieldbus over Ethernet`), disable both **EtherNet/IP** and **Modbus**. If left enabled, they claim the `speed_slider_mask` register and prevent the Intrinsic RTDE client from initializing.
6. **Configure Payload, TCP & Safety Planes**:
   * Verify that the active tool payload mass and center of gravity in **`Installation | General | Payload`** match your physical end-effector so the arm does not fault or drift upon enabling.
   * Configure appropriate safety boundaries under **`Installation | Safety | Planes`**.

### 2. UR Driver Configuration (`ur_config.textproto` — Step 2)

UR manipulators communicate via Universal Robots' Real-Time Data Exchange (RTDE) protocol at **500 Hz** (`control_frequency_hz: 500`). Just as with KUKA, rather than using the standalone simulation template ([`default_config_with_scene_object.pbtxt`](../../../../../incode/intrinsic_control/intrinsic/icon/hardware_modules/universal_robots/default_config_with_scene_object.pbtxt)), create a cell-specific `ur_config.textproto` in your Bzlmod workspace so `robot_ip` matches your UR controller's IP address (`192.168.10.1`) and the module automatically inherits the `name = "robot"` instance name from your `BUILD` file:

```textproto
# proto-file: intrinsic/icon/hal/proto/hardware_module_config.proto
# proto-message: intrinsic_proto.icon.HardwareModuleConfig

[type.googleapis.com/intrinsic_proto.icon.HardwareModuleConfig] {
  drives_realtime_clock: true
  control_frequency_hz: 500
  module_config {
    [type.googleapis.com/intrinsic_proto.icon.UniversalRobotsModuleConfig] {
      robot_ip: "192.168.10.1"
    }
  }
}
```

### 3. UR ICON MainLoop Configuration (`ur_icon_main_config.textproto` — Step 3) & `BUILD` (Step 4)

Create `ur_icon_main_config.textproto` in your Bzlmod workspace to match the **500 Hz** RTDE clock (`control_frequency_hz: 500.0`), reference `"robot"`, and configure `HalADIOPart` for Universal Robots' three digital I/O banks (`standard_*`, `configurable_*`, and `tool_*`) plus `analog_input_status`:

```textproto
# proto-file: google/protobuf/any.proto
# proto-message: google.protobuf.Any

[type.googleapis.com/intrinsic_proto.icon.IconMainConfig] {
  intrinsic_runtime {
    name: "intrinsic_runtime"
  }

  # Match the 2 ms UR RTDE fieldbus clock (500 Hz)
  control_frequency_hz: 500.0
  hardware_module_read_write_timeout_seconds: 1

  services {
    world_service_from_grpc { world_id: "world" }
    kinematics_from_world_service: true
    assembly_from_world_service: true
  }

  hardware_module_names: ["robot"]
  hardware_module_that_drives_clock: "robot"

  realtime_control_config {
    parts_by_name {
      key: "arm"
      value {
        part_type_name: "HalArmPart"
        safety_action_type_name: "intrinsic.stop"
        hardware_resource_name: "robot"
        config {
          [type.googleapis.com/intrinsic_proto.icon.HalArmPartConfig] {
            joint_position_command { module_name: "robot" interface_name: "joint_position_command" }
            joint_position_state   { module_name: "robot" interface_name: "joint_position_state" }
            joint_velocity_state   { module_name: "robot" interface_name: "joint_velocity_state" }
            payload_command        { module_name: "robot" interface_name: "payload_command" }
            payload_state          { module_name: "robot" interface_name: "payload_state" }
          }
        }
      }
    }
    parts_by_name {
      key: "adio"
      value {
        part_type_name: "HalADIOPart"
        safety_action_type_name: "intrinsic.empty"
        config {
          [type.googleapis.com/intrinsic_proto.icon.HalADIOPartConfig] {
            digital_inputs  { interface { module_name: "robot" interface_name: "standard_digital_input_status" } export_name: "standard_in" }
            digital_inputs  { interface { module_name: "robot" interface_name: "configurable_digital_input_status" } export_name: "configurable_in" }
            digital_inputs  { interface { module_name: "robot" interface_name: "tool_digital_input_status" } export_name: "tool_in_status" }
            digital_inputs  { interface { module_name: "robot" interface_name: "standard_digital_output_status" } export_name: "standard_out_status" }
            digital_inputs  { interface { module_name: "robot" interface_name: "configurable_digital_output_status" } export_name: "configurable_out_status" }
            digital_inputs  { interface { module_name: "robot" interface_name: "tool_digital_output_status" } export_name: "tool_out_status" }
            digital_outputs { interface { module_name: "robot" interface_name: "standard_digital_output_command" } export_name: "standard_out" }
            digital_outputs { interface { module_name: "robot" interface_name: "configurable_digital_output_command" } export_name: "configurable_out" }
            digital_outputs { interface { module_name: "robot" interface_name: "tool_digital_output_command" } export_name: "tool_out" }
            analog_inputs   { interface { module_name: "robot" interface_name: "analog_input_status" } export_name: "analog_in" }
          }
        }
      }
    }
  }
}
```

Then declare the UR solution in your `BUILD` file and deploy it via `bazel run //:ur_solution -- --address localhost:17080 --operation_mode=real` (or with `--cluster <cluster-name> --org <org@project> --operation_mode=real`):

```python
load("@ioc//intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("@ioc//intrinsic/config:def.bzl", "intrinsic_solution")

intrinsic_asset_instance(
    name = "robot",
    asset = "ai.intrinsic.ur5e_hardware_module_ioc",
    instance_name = "robot",
    service_config = ":ur_config.textproto",
)

intrinsic_asset_instance(
    name = "icon",
    asset = "ai.intrinsic.generic_realtime_control_service",
    instance_name = "icon",
    service_config = ":ur_icon_main_config.textproto",
)

intrinsic_solution(
    name = "ur_solution",
    add_compose_world_test = False,
    assets = [
        "@ioc//intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_ioc",
        "@ioc//intrinsic_control/intrinsic/icon/machines/common:generic_icon_mainloop_type",
    ],
    default_operation_mode = "real",
    instances = [
        ":robot",
        ":icon",
    ],
)
```

### 4. Known UR Error Messages & Troubleshooting

If the UR hardware module fails to initialize or faults during execution, inspect the service logs (`inctl logs` on Intrinsic Enterprise, or `kubectl logs` / `k9s` on Intrinsic Core) for these common causes:

* **`Failed to connect to <IP> in 10s. Ensure the robot is powered on and the IP is correct.`**
  * **Cause**: The driver could not reach the UR Dashboard Server on TCP port `29999` (even if `ping <UR_IP>` succeeds).
  * **Resolution**: Verify physical Ethernet connectivity, confirm the Teach Pendant is switched to **`Remote`** mode, and verify that **Dashboard Server**, **Primary Client Interface**, and **Real-Time Data Exchange (RTDE)** are enabled under `Settings | Security | Services`. You can test Dashboard Server reachability directly from your host machine:
    ```bash
    curl --max-time 3 192.168.10.1:29999
    ```
    * If disabled or unreachable, `curl` times out (`curl: (28) Connection timed out after 3002 milliseconds`).
    * If enabled and reachable, `curl` responds immediately with `curl: (1) Received HTTP/0.9 when not allowed`.

* **`Variable 'speed_slider_mask' is currently controlled by another RTDE client` / `Failed to initialize RTDE client after 3 attempts`**
  * **Cause**: An external fieldbus (**EtherNet/IP** or **Modbus**) or a third-party URCap has claimed the RTDE input recipe registers on the controller (`when using a fieldbus (e.g. Ethernet/IP or Modbus), all outputs are claimed by those`).
  * **Resolution**: Navigate to `Installation | Fieldbus` (`Fieldbus over Ethernet`) on the UR Teach Pendant, disable both **EtherNet/IP** and **Modbus**, and restart the robot controller.

* **`Did not receive data from robot.` (Digital I/O High-Frequency Considerations)**
  * **Cause**: Indicates that the hardware module did not receive RTDE real-time packets from the UR controller for `>100 ms`, causing closed-loop control to fault. On Universal Robots controllers, changing the state of multiple digital outputs every cycle at high frequency (**`>100 Hz`**) causes high controller CPU load, which is a known UR RTDE issue that causes the controller to skip sending network packets. *(Also note that it takes **2 to 4 RTDE cycles** for a digital output command to be reflected in the corresponding digital output status input.)*
  * **Resolution**: Avoid toggling UR digital outputs at rates exceeding 100 Hz. To recover the controller after a packet timeout fault, clear faults (`ClearFaults` / re-enable ICON).

* **Home Position vs. Candlestick Pose (`[0, 0, 0, 0, 0, 0]` rad)**
  * Note that all-zero joint angles (`[0, 0, 0, 0, 0, 0]` rad) on a UR arm places the manipulator in a fully horizontal outstretched posture rather than an upright "Candlestick" pose. A standard upright home configuration for UR manipulators is `[0, -1.5708, 0, -1.5708, 0, 0]` rad.

> [!NOTE]
> **Internal Force-Torque Sensor & ROS 2 Integration**
> * **Internal Force-Torque Sensor**: UR e-Series arms also include an integrated 6-axis wrist force-torque sensor (`HalForceTorqueSensorPart` bound to `force_torque_status` / `force_torque_command`). While omitted here to keep initial robot bringup focused on position and digital/analog I/O, we cover configuring and using the force-torque sensor for contact-sensitive motion in our follow-up [Client API & Move-to-Contact Tutorial](client_api/move_to_signal_tutorial.md).
> * **ROS 2 Integration**: For UR robots—or any other manipulator supported by ROS—you can alternatively integrate via `ros2_control` using the [ROS 2 Control Bridge Tutorial](https://todo.example/ros2_control_bridge).

---

## Next Steps

Congratulations! You have declaratively configured and deployed a native KUKA KR6 R900-2 hardware device and ICON realtime controller in lockstep with the robot's real-time RSI clock.

From here, you can:
* Learn how to write standalone motion scripts and orchestrate trajectories using the [Client API Tutorial](client_api/move_to_signal_tutorial.md).
* Explore connecting unsupported robot models via `ros2_control` using the [ROS 2 Control Bridge Tutorial](https://todo.example/ros2_control_bridge).
* Implement custom actuator drivers or proprietary fieldbuses with the [Custom Hardware Modules Tutorial](https://todo.example/custom_hardware_modules).
