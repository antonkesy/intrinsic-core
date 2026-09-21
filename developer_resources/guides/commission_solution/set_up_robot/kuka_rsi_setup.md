# KUKA RSI setup

> [!NOTE]
> This guide was adapted from the Intrinsic Enterprise documentation for
> Intrinsic Core. Please report any issues or unclear documentation.

[beckhoff-ek1100]: https://www.beckhoff.com/en-en/products/i-o/ethercat-terminals/ek1xxx-bk1xx0-ethercat-coupler/ek1100.html
[beckhoff-el1008]: https://www.beckhoff.com/en-en/products/i-o/ethercat-terminals/el1xxx-digital-input/el1008.html
[beckhoff-el2008]: https://www.beckhoff.com/en-en/products/i-o/ethercat-terminals/el2xxx-digital-output/el2008.html
[kuka-agilus]: https://www.kuka.com/en-de/products/robot-systems/industrial-robots/kr-agilus
[kuka-c4-quick-start]: https://xpert.kuka.com/ID/PB1587
[kuka-c5-quick-start]: https://xpert.kuka.com/ID/PB19056
[kuka-eki-3_1]: https://xpert.kuka.com/ID/AR35887
[kuka-eki-3_2]: https://xpert.kuka.com/ID/AR52165
[kuka-rsi-4]: https://xpert.kuka.com/ID/AR35884
[kuka-rsi-5]: https://xpert.kuka.com/ID/AR52166
[kuka-start-up-course]: https://xpert.kuka.com/ID/AR71874
[kuka-system-software-8_6]: https://xpert.kuka.com/ID/PB11620
[kuka-system-software-8_7]: https://xpert.kuka.com/ID/PB14656
[kuka-workvisual]: https://xpert.kuka.com/ID/AR31290
[icon-skills]: ../../../../intrinsic_control/intrinsic/icon/skills

KUKA industrial robots (e.g. [AGILUS KR6][kuka-agilus]) can be controlled with
Intrinsic Core through the [RobotSensorInterface][kuka-rsi-4] (RSI) option
provided by KUKA.

> [!WARNING]
> This guide does **not** include guidance on **safety**. Set up adequate safety
> systems and conduct all risk assessments before deploying to real hardware.

## Overview

The Intrinsic hardware module for KUKA robots uses the KUKA features
RobotSensorInterface (RSI) and Ethernet KRL Interface (EKI) to communicate with
the KUKA Robot Control(er) (KRC).
The main communication channel is RSI, which is the real-time control feature of
KUKA and is used for motion execution.
EKI is used to retrieve additional non-real-time status information and to send
control commands such as enable, disable or clear faults.

The setup consists of the following steps:

- Setting up the [network wiring](#step-1-network-wiring-between-the-ipc-and-the-kuka-robot-controller)
  between the KUKA Robot Controller (KRC) and the Intrinsic IPC
- [Setting up](#physical-io-device-for-external-control) the I/Os for external
  control of a KUKA robot
- [Configuring](#step-2-kuka-robot-controller-setup) the
  [KUKA WorkVisual][kuka-workvisual] project
- [Adding](#step-3-add-the-assets-to-your-solution) the assets to your solution
- [Enabling](#step-4-enable-the-robot) the robot

![Network setup on IPC and KRC](../../../img/guides/kuka_rsi_setup_overview.png)

## Prerequisites

- Basic knowledge about Intrinsic Core
- Basic knowledge about KUKA robot configuration and usage
  ([KUKA Start-Up Course][kuka-start-up-course])
- A supported KUKA industrial robot that is operational
  (**safety is set up** (the robot is in a safety cell) and KUKA programs can be
  executed). See the
  [hardware module README](../../../../intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi/README.md)
  for the list of supported models
- One of
  - [KUKA C4][kuka-c4-quick-start] controlbox (KRC) with
    [KUKA System Software 8.6][kuka-system-software-8_6] and
    [RobotSensorInterface 4.X][kuka-rsi-4],
    [Ethernet KRL 3.1][kuka-eki-3_1] options
  - [KUKA C5][kuka-c5-quick-start] controlbox (KRC) with
    [KUKA System Software 8.7][kuka-system-software-8_7] and
    [RobotSensorInterface 5.0][kuka-rsi-5],
    [Ethernet KRL 3.2][kuka-eki-3_2] options
- [KUKA WorkVisual 6.0][kuka-workvisual] running on a
  notebook or desktop pc with an Ethernet connection to the KUKA C4's X69 port
  or to the KR C5's XF1 port, using DHCP
- KUKA compatible I/O device. Recommendation: Beckhoff
  [EK1100][beckhoff-ek1100], [EL1008][beckhoff-el1008],
  [EL2008][beckhoff-el2008]\
  (an example KUKA configuration is provided in the Intrinsic base option
  package)
- An industrial PC (IPC) with Intrinsic Core installed, with real-time tuning
  applied (see
  [`intrinsic_runtime/setup_realtime.sh`](../../../../intrinsic_runtime/setup_realtime.sh))
- 1 RJ45 CAT6 network cable

## In a nutshell for experienced KUKA robot users

1. Install the Intrinsic option packages:
   - [Base package](../../../files/IntrinsicBase-1.1.1.kop)
   - [Extended package](../../../files/IntrinsicExtended-1.1.0.kop)
   - The I/O configuration available in the Extended package
1. Connect an Ethernet cable to the robot-facing Ethernet port of the Intrinsic
   IPC and to the KLI port of the KRC
1. Wire up the [EK1100][beckhoff-ek1100], [EL1008][beckhoff-el1008]
   and [EL2008][beckhoff-el2008] and connect outputs 1-3 of EL2008 to inputs 1-3
   of EL1008 with a wire each
1. Adjust the IPC network configuration following the
   [network setup](#rsieki-virtual-network-card-nic-configuration) section
1. Follow the setup in section
   ["Add the assets to your solution"](#step-3-add-the-assets-to-your-solution)
1. Set the robot to `External` mode using the teach pendant
1. Use the robot

> [!NOTE]
> For testing, you can skip the I/O setup and
> [enable the robot using the teach pendant](#legacy-mode-enable-the-robot-using-the-teach-pendant).

More detailed information is given in the following sections.

## Step 1: Network wiring between the IPC and the KUKA robot controller

The KUKA KR C4 needs to be connected to the Intrinsic IPC. This means the
robot-facing (**non** EtherCAT) Ethernet port of the IPC needs to be connected
to the KUKA Line Interface (KUKA KLI), on the KR C4 to the X66 port.

## Step 2: KUKA robot controller setup

KUKA features are often provided as KUKA option packages, such as
*SafeOperation* or *RobotSensorInterface*.\
Intrinsic provides two KUKA option packages for convenient installation and
configuration of the KUKA Robot Controller (KRC):

- A **Base** package containing basic configuration files for RSI and Ethernet
  KRL Interface (EKI) communication with Intrinsic Core.\
  [**Download Base package**](../../../files/IntrinsicBase-1.1.1.kop)
- An **Extended** package based on the base package but containing additional
  configuration that is more likely to be in conflict with controller
  configurations when the KRC project already has some pre-existing
  configuration changes (see
  [option package description](#contents-of-the-intrinsic-option-packages) for a
  description of the contents of the option packages).\
  [**Download Extended package**](../../../files/IntrinsicExtended-1.1.0.kop)

For easy setup it is recommended to install both option packages. The extended
option package should only be skipped if it causes problems with e.g. the
existing network configuration or IO mappings.

> [!NOTE]
> Create a backup of your WorkVisual project before installing **or** updating
> any of the option packages since the KUKA option package installer overwrites
> existing files that were added or modified manually.

> [!WARNING]
> When installing the Intrinsic **extended** option package, it will overwrite
> the following elements:
>
> - The submit interpreter at position three (the first user defined submit
>   interpreter)
> - The network configuration of virtual devices named `KLI` and `RSI`
>   (case-insensitive)
> - The KUKA signals `$DRIVES_ON`, `$MOVE_ENABLE` and `$CONF_MESS`

Install the Intrinsic option package just as other KUKA option packages are
installed.\
See the [KUKA WorkVisual 6.0][kuka-workvisual] manual (section 6.30) for a
detailed description of the process.

### Physical I/O device for external control

To *enable*, *disable* and *clear faults* from an external source, KUKA requires
the use of physical I/Os.\
This means **3 outputs** need to be wired onto **3 inputs** and configured and
mapped on the KUKA controller or in WorkVisual.

The Intrinsic **extended option package** provides such a configuration and
mapping for the recommended I/O device combination Beckhoff
[EK1100][beckhoff-ek1100], [EL1008][beckhoff-el1008] and
[EL2008][beckhoff-el2008] connected using EtherCAT to the KRC at port X65.

Use the first three outputs and inputs to match the provided configuration as
shown in the following figures.\
The configuration and mapping can be installed in the same way an option package
is installed into a WorkVisual project:

By dragging the `Intrinsic I/Os using EK1100, EL1008, EL2008` component from the
Options catalog onto the controller in WorkVisual.\
The I/Os must be mapped on indices `421-423`.

> [!WARNING]
> Don't change the I/O indexes during installation, or you need to adjust the
> corresponding KRL signals as well!

#### I/O wiring diagram for external KUKA control

![I/O wiring diagram for external KUKA control](../../../img/guides/kuka_eki_io_wiring.png)

#### I/O wiring example for external KUKA control

![I/O wiring for external KUKA control](../../../img/guides/kuka_io_wiring.png)

#### Usage of other I/O devices

It's possible to use other I/O devices, but the detailed configuration is out of
scope for this documentation.\
The minimal requirements are:

- The I/O devices need to be compatible with the KUKA KRC at hand and to be
  configured in WorkVisual.
- Three of the digital inputs and outputs of the device need to be mapped to the
  `KR C I/Os` with indexes `421-423`, each for inputs and outputs.

### RSI/EKI virtual network card (NIC) configuration

EKI and RSI are used to communicate with the KRC.\
KUKA requires using different virtual network devices for these two channels.\
Therefore, the Intrinsic extended option package installs two virtual network
devices:

- RSI with the IP `192.170.10.122` and subnet mask `255.255.255.0`
- EKI and potential other non-real-time communication with the IP
  `192.170.11.122` and subnet mask `255.255.255.0`

Analogously, the Intrinsic IPC needs to have two addresses on the *non-EtherCAT*
robot NIC with fitting IP addresses, for example `192.170.10.123` and
`192.170.11.123`.

![Network setup on IPC and KRC](../../../img/guides/kuka_rsi_network_setup_diagram.png)

> [!NOTE]
> It is recommended to use the suggested IPs. Otherwise, there might be a
> conflict in `C:\Config\User\Common\SensorInterface\RSIEthernetConfig.xml`
> every time the Intrinsic option package is updated.

On a standard Ubuntu installation, both addresses are configured on the
robot-facing NIC with netplan, for example in `/etc/netplan/60-robot.yaml`:

```yaml
network:
  version: 2
  ethernets:
    enp6s0:
      dhcp4: false
      addresses:
        - 192.170.10.123/24
        - 192.170.11.123/24
```

Apply the configuration with `sudo netplan apply`, then check that the IPC can
reach the controller:

```bash
ip address show enp6s0
ping 192.170.10.122
ping 192.170.11.122
```

An interface can stay without an IP address when:

- The other side (e.g. KRC) is not turned on or the NIC is not working
  correctly. Try rebooting the KRC
- No cable is plugged into the network port of the NIC
- The cable is in the wrong port

### Deploy the changes on the robot

Deploy all changes onto the robot using WorkVisual.

## Step 3: Add the assets to your solution

To control a KUKA robot you need two assets in your solution:

- the hardware device for your robot model, which contains the robot geometry
  and the KUKA RSI hardware module, and
- the real-time control service, which runs the hardware module in a real-time
  control loop.

The supported models and their asset IDs are listed in the
[hardware module README](../../../../intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi/README.md),
for example `ai.intrinsic.kuka_kr6_hardware_module` for the KR 6 R900-2.

Define the workcell as an `intrinsic_solution` target that lists both assets
and their instances. A real robot always needs a custom
[configuration](#configure-the-rsi-and-eki-addresses), and the solution target
is where that configuration lives:

```python
load("@intrinsic-core//intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("@intrinsic-core//intrinsic/assets/build_defs:solution.bzl", "intrinsic_solution")

intrinsic_asset_instance(
    name = "robot",
    asset = "ai.intrinsic.kuka_kr6_hardware_module",
    service_config = "kr6_config.textproto",
)

intrinsic_asset_instance(
    name = "realtime_control",
    asset = "ai.intrinsic.generic_realtime_control_service",
)

intrinsic_solution(
    name = "my_workcell",
    assets = [
        "@intrinsic-core//intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi:kr6_r900_2_hardware_module",
        "@intrinsic-core//intrinsic_control/intrinsic/icon/machines/common:generic_icon_mainloop_type",
    ],
    default_operation_mode = "real",
    instances = [
        ":realtime_control",
        ":robot",
    ],
)
```

Deploy the solution to the cluster by running the solution target:

```bash
bazel run //path/to:my_workcell -- --address localhost:17080 --operation_mode=real
```

Use `--operation_mode=sim` to run the same solution in simulation.

> [!NOTE]
> A solution deployed this way is ephemeral: changes made at runtime are lost.
> Make configuration changes in the `intrinsic_solution` target and its
> configuration files, then run the target again.

### Configure the RSI and EKI addresses

The configuration of the hardware module is an
`intrinsic_proto.icon.HardwareModuleConfig` text proto, referenced from the
`service_config` attribute of the instance. Start from the module's default
configuration,
[`kuka_rsi_default_config.textproto`](../../../../intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_default_config.textproto),
which already matches the addresses used by the Intrinsic option packages:

```textproto
# proto-file: intrinsic/icon/hal/proto/hardware_module_config.proto
# proto-message: intrinsic_proto.icon.HardwareModuleConfig

[type.googleapis.com/intrinsic_proto.icon.HardwareModuleConfig] {
  name: "kuka_rsi_hal_module"
  module_config {
    [type.googleapis.com/intrinsic_proto.icon.KukaRsiModule] {
      local_rsi_host: "192.170.10.123"
      local_rsi_port: 5852
      velocity_settling_cutoff_frequency: 100
      digital_input_names: ["di_1", "di_2", "di_3", "di_4"]
      digital_output_names: ["do_1", "do_2"]
      eki_status_client_params: {
        eki_address: "192.170.11.122"
        eki_port: 54600
        # Update rate of the EKI status telemetry. A rate of 10+ is mainly
        # needed for a smooth visualization during teach pendant jogging.
        # Recommended not to change. Limited to 15 Hz due to EKI response time.
        update_frequency: 15  # Hz
      }
      eki_control_client_params: {
        eki_address: "192.170.11.122"
        eki_port: 54601
        # Update rate of the EKI control telemetry. Recommended not to change.
        # Limited to 15 Hz due to EKI response time.
        update_frequency: 2  # Hz
      }
    }
  }
  drives_realtime_clock: true
  control_frequency_hz: 250
}
```

If you are not using the default IPs, adjust:

- `local_rsi_host`: the IP of the IPC NIC that receives the RSI messages, the
  same IP as used in the
  [RSI network configuration](#rsieki-virtual-network-card-nic-configuration)
  (default `192.170.10.123`).
- `eki_address` in `eki_status_client_params` and `eki_control_client_params`:
  the IP of the virtual NIC named "KLI" on the KUKA KRC (not the "RSI" NIC).

> [!NOTE]
> `eki_control_client_params` needs to be set in the configuration.
> Otherwise, the hardware module assumes it is supposed to run without EKI.
> In that case, see
> ["Legacy mode: enable the robot using the teach pendant"](#legacy-mode-enable-the-robot-using-the-teach-pendant).

### (Optional) Customize digital inputs and outputs

The RSI context installed with the Intrinsic base option package contains four
preconfigured inputs and two preconfigured outputs for custom use.\
This can be extended to a maximum of ~50 inputs and ~50 outputs.

> [!NOTE]
> RSI allows 64 inputs and 64 outputs in total and the control commands and
> telemetry need around 10.

Each KUKA input must be connected to an input of the RSI Ethernet object.\
Each KUKA output must be connected to *both* an input and an output of the
Ethernet object.\
The outputs that are connected to inputs contain the actual values
of an output, not just the set values.\
Each input and output must use `Bit` as the value of the `DataType` field.\
This configuration is demonstrated in the RSIContext for some inputs and outputs
and shown in the next figure.

![Initial RSI context](../../../img/guides/rsi_context.png)

Add the same I/Os to
`C:\Config\User\Common\SensorInterface\RSIEthernetConfig.xml` and follow the
demonstrated pattern:

```xml
  <!-- Customizable digital inputs/outputs.
  Input Tags need to be of format "DigIn.DI<number>",
  where the number needs to be ascending without gaps.
  Outputs must be similarly tagged "DigOut.DO<number>" -->
  <ELEMENT TAG="DigIn.DI1" TYPE="LONG" INDX="16" />
  <ELEMENT TAG="DigIn.DI2" TYPE="LONG" INDX="17" />
  <ELEMENT TAG="DigIn.DI3" TYPE="LONG" INDX="18" />
  <ELEMENT TAG="DigIn.DI4" TYPE="LONG" INDX="19" />
  <ELEMENT TAG="DigOut.DO1" TYPE="LONG" INDX="20" />
  <ELEMENT TAG="DigOut.DO2" TYPE="LONG" INDX="21" />
```

There **must not** be any gaps in the numbering.\
The TAG names in `RSIEthernetConfig.xml` are case-sensitive.

Make sure to map those inputs and outputs to appropriate fields in the `Mapping`
view in WorkVisual as needed for your specific application (see
[WorkVisual section 9.4 "Mapping the bus I/Os"][kuka-workvisual]).

Note that you need to map to KUKA's `$IN[]` and `$OUT[]` variables in the
**"KR C I/Os"** tab since the provided `RSIContext.rsix` file uses the RSI
objects `DigIn` and `DigOut`.\
This way the state of the I/Os can also be inspected on the teach pendant.

The following figure shows an example for mapping the integrated outputs of a
KR6 onto the first 2 entries of the **"KR C I/Os"** as used in the provided
`RSIContext.rsix` file:

![WorkVisual I/O mapping example](../../../img/guides/wov_io_mapping_example_kr6.png)

Those indexes of the KUKA `$OUT` variable are to be specified in the RSIVisual
editor window for the `DigOut` and `Map2DigOut` objects as well:

![WorkVisual RSI context example](../../../img/guides/wov_io_rsicontext_example_kr6.png)

Configure the inputs following the same pattern.

The hardware module configuration must have the same number of
`digital_input_names` and `digital_output_names` entries as inputs and outputs
are configured in
`C:\Config\User\Common\SensorInterface\RSIEthernetConfig.xml`.\
The names can be freely chosen. For example, for four inputs and two outputs:

```textproto
module_config: {
  [type.googleapis.com/intrinsic_proto.icon.KukaRsiModule]: {
    local_rsi_host: "192.170.10.123"
    # ...
    digital_input_names: ["di_1", "di_2", "di_3", "di_4"]
    digital_output_names: ["do_1", "do_2"]
  }
}
```

Those digital inputs can be observed with the `dio_wait_for_input` skill and
digital outputs can be changed with the `dio_set_output` skill (see
[the ICON skills][icon-skills]).
Additionally, both can be used with the ICON ADIO action.
The state of the signals can also be inspected from the command line:

```bash
inctl gpio list-signals --address localhost:17080
inctl gpio read-signals --signal_names=di_1,di_2 --address localhost:17080
inctl gpio write-signal --signal_name=do_1 --value=true --address localhost:17080
```

## Step 4: Enable the robot

When the hardware module has been deployed, it immediately starts listening
for the RSI data from the KUKA KRC.

> [!WARNING]
> Robot safety training and an introduction to the KUKA robot system is
> required before executing the following steps!

Switch to `External` (or `Ext`) operation mode using the physical turn switch on
the KUKA teach pendant.

The robot enables automatically when there is no fault.

Check the state of real-time control, and clear faults if there are any:

```bash
inctl icon status --address localhost:17080         # operational status, faults, safety status
inctl icon clear-faults --address localhost:17080
inctl icon enable --address localhost:17080         # enable all parts
```

If the faults can be cleared and the RSI connection is stable, the robot control
stack enables itself. Otherwise, the active fault is shown by
`inctl icon status`.

Further commands that are useful during bring-up:

```bash
inctl icon config --address localhost:17080         # the ICON configuration in use
inctl icon list-parts --address localhost:17080
inctl service state list --address localhost:17080
inctl service state restart robot --address localhost:17080
```

### Legacy mode: enable the robot using the teach pendant

In case you are not using EKI and the physically looped I/Os, you can enable the
robot using the teach pendant.\
In this case, it is important that the `eki_control_client_params` value does
not exist in the configuration of the hardware module.

Follow these steps to enable RSI using the teach pendant:

1. Release all emergency stops (on the teach pendant, cell etc.)
1. Confirm all messages
1. Switch to `automatic` mode (physical turn switch)
1. Choose the `RSI_MC` program from the file view and press the **Select**
   button at bottom of the screen (do *not* press the **Open** button since
   this opens the program for editing)

    ![Select the RSI_MC program](../../../img/guides/kuka_select_program.png)

1. Enable the drives (press the button with `O` at the top. In the dialog press
   `I` and wait until the `O`-button becomes a green `I`-button. If the drives
   are already enabled, you see a green `I` in place of the `O`.)

    ![Enable the drives](../../../img/guides/kuka_teach_pendant_enable_drives.png)

1. Press the green **Play** button (left of screen) until the cursor arrives at
   `RSI_MOVECORR()`

Now you should be able to move the robot. You may first need to run
`inctl icon clear-faults`.

## Troubleshooting

This section describes typical obstacles a user might encounter when using KUKA
robots.

### `General Motion Enable` message on teach pendant

A digital output signal must be connected to the KRC that is mapped to the input
signal `$MOVE_ENABLE`.\
In many cells this input is always active.
If not, the "General Motion Enable" message is shown.\
If the Intrinsic extended option package is installed, the `$MOVE_ENABLE` input
signal is mapped to `$IN[421]` and waits for this input to be active (i.e.
`true` or `1`).

### Robot moves after teach pendant jogging

This is intended behavior of a KUKA robot. It does a `BCO` motion to the last
known position.\
The RSI KRL program needs to be reset when using teach pendant jogging.\
This is done automatically in EKI controlled mode.

### Installation of Intrinsic Extended Option Package fails with `Cannot edit KLIConfig.xml`

The PC where WorkVisual is running likely does not have the XML library
installed.\
There should be a Windows window to install it.\
Afterwards WorkVisual needs to be restarted.

### A version mismatch is reported as a fault

The version of the hardware module and the Intrinsic option package must match
in the major and minor version. Otherwise, the robot cannot be enabled.\
Update both the IPC and the Intrinsic option package to the same version (check
[KRC setup](#step-2-kuka-robot-controller-setup) or
[Archive](#archive-of-option-package-releases)).

### The I/O count does not match

The following message is reported when the I/O configuration on the KRC and in
the hardware module configuration do not match:
`The number of received inputs (4) does not match the number of configured
digital inputs (0).`

This happens when the I/O configuration in
`C:\Config\User\Common\SensorInterface\RSIEthernetConfig.xml` does not match the
configuration in the
[hardware module](#optional-customize-digital-inputs-and-outputs).
To fix this, make sure both configurations have the same number of inputs and
outputs and redeploy the hardware module or the WorkVisual project.

### `Could not bind to socket`

In case a message like `Could not bind to socket: 192.170.10.123:5852: Cannot
assign requested address` is reported, the IP of the NIC on the Intrinsic IPC
used for communication with the KUKA KRC does not fit the configuration.\
In this example, the IP `192.170.10.123` was configured in the hardware module
configuration, but there is no NIC with that IP on the IPC.

This can be resolved in two ways, depending on your network configuration:

- Adjust the IP of the NIC as described in the
  [network configuration](#rsieki-virtual-network-card-nic-configuration)
  section
- Adjust the IP used for RSI/EKI communication in the hardware module
  configuration as shown in the
  [configuration](#configure-the-rsi-and-eki-addresses) section

### `RSI start timed out`

This error message can have several causes:

- A common reason after changing project settings on the KUKA robot is that the
  network card on the KUKA is not working anymore.
  - This can often be solved by rebooting the KUKA controller.
- The RSI_MC KRL program could not be started on the teach pendant.
  - Investigate error messages on the KUKA teach pendant for more information.
- A wrong network IP is configured on the KUKA or the Intrinsic IPC side for the
  KUKA RSI connection.
  - Align the IP addresses to be in the same network and align the hardware
    module configuration with the `RSIEthernetConfig.xml` on the KUKA side.

### `RSI object configured incorrectly` on teach pendant

In general, `RSI object configured incorrectly` means that the KUKA runtime has
found an issue within the RSI configuration.
Inspect the `C:\Config\User\Common\SensorInterface\RSIEthernetConfig.xml` and
the `C:\Config\User\Common\SensorInterface\RSIContext.rsix` for any invalid
configuration and follow the instructions on the teach pendant linked with the
error message (if available).

If all else fails, re-installing the `IntrinsicBase` option package will restore
the original state of the RSI files.

## Contents of the Intrinsic option packages

In case of conflicts during installation or if you would like to know what you
are installing on your KRC, the contents of the Intrinsic option packages are
described below.

### The Intrinsic base option package

The [Base package](../../../files/IntrinsicBase-1.1.1.kop) specifies the
necessary dependencies and their compatible versions.

It contains files for

- the RSI context
- the RSI XML protocol definition
- two EKI protocol definitions
- the RSI KRL program
- the Intrinsic submit interpreter for EKI communication and associated global
  variables

### The Intrinsic extended option package

The [Extended package](../../../files/IntrinsicExtended-1.1.0.kop) depends on
the base package and contains some file modifications:

- Replaces existing KLI/RSI (case-insensitive) virtual NICs on the KRC with a
  configuration suitable for the default Intrinsic configuration
- Replaces the KUKA signals `$DRIVES_ON`, `$MOVE_ENABLE` and `$CONF_MESS` with
  values that the Intrinsic EKI integration expects
- Starts the Intrinsic submit interpreter at position 3

## Archive of option package releases

### v1.1.1 - Aug. 2025

- [Base package](../../../files/IntrinsicBase-1.1.1.kop)

#### Release notes

- Fix error in intrinsic.sub when no messages are present

### v1.1.0 - Sep. 2024

- [Base package](../../../files/IntrinsicBase-1.1.0.kop)
- [Extended package](../../../files/IntrinsicExtended-1.1.0.kop)

#### Release notes

- Add support for setting the payload from the platform on the KRC
- Fix KSS error message numbers null termination in XML string
- Fix KSS dependency version for KR C5 (KSS 8.7.x)

### v1.0.0 - Feb. 2024

- [Base package](../../../files/IntrinsicBase-1.0.0.kop)
- [Extended package](../../../files/IntrinsicExtended-1.0.0.kop)

#### Release notes

Initial release
