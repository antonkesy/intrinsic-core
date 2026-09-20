# Universal Robots RTDE setup

> [!NOTE]
> This guide was adapted from the Intrinsic Enterprise documentation for
> Intrinsic Core. Please report any issues or unclear documentation.

Intrinsic Core integrates [Universal Robots](https://www.universal-robots.com/)
(UR) manipulators through the Real-Time Data Exchange (RTDE) protocol.
This guide describes how to set up and use a UR robot with Intrinsic Core.

> [!WARNING]
> This guide does not include guidance on safety. Set up adequate safety
> systems and conduct all risk assessments before deploying to real hardware.

> [!WARNING]
> A UR can be jogged from Intrinsic Core even if the pendant is in `Local` mode.
> This is a limitation of UR.

## Overview

This documentation guides you through each step of setting up a Universal Robot
with Intrinsic Core:

- [Set up the robot](#initial-robot-setup)
- [Switch the robot to remote control](#switch-to-remote-control)
- [Add the robot to your solution](#add-the-robot-to-your-solution)
- [Operate the robot](#operate-the-robot)

## Prerequisites

- The **minimal** supported version of Polyscope (UR firmware) is **5.9.4**
- A supported robot model: UR3e, UR5e, or UR10e (see the
  [hardware module README](../../../../incode/intrinsic_control/intrinsic/icon/hardware_modules/universal_robots/README.md))
- An industrial PC (IPC) with Intrinsic Core installed, with real-time tuning
  applied (see
  [`incode/ioc/setup_realtime.sh`](../../../../incode/ioc/setup_realtime.sh))
  and with a spare Ethernet port for the robot
- This guide assumes your robot is configured with the IP address
  `192.170.10.1` in the [module configuration](#configure-the-robot-ip), and
  that the robot-facing NIC of the IPC has an address in the same subnet (for
  example `192.170.10.123/24`)
- 1 RJ45 CAT6 network cable

## Initial robot setup

The following sections show the configuration for using a Universal Robot with
Intrinsic Core. The configuration needs to be performed only once.

The robot needs to be in `Manual` mode for settings to be available.
The mode can be changed in the top right corner of the UR Teach Pendant if
**Settings | System | Remote Control** is already enabled.

### Network connection

Connect the UR control box directly to the robot-facing (**non** EtherCAT)
Ethernet port of the IPC using an Ethernet cable.

Give that interface a static address in the robot's subnet. On a standard Ubuntu
installation this is done with netplan, for example in
`/etc/netplan/60-robot.yaml`:

```yaml
network:
  version: 2
  ethernets:
    enp6s0:
      dhcp4: false
      addresses:
        - 192.170.10.123/24
```

Apply it with `sudo netplan apply` and verify with
`ping 192.170.10.1` once the robot is configured.

### Configure remote control

In **Settings | System | Remote Control**, enable `Remote Control` mode.

This adds the `Local/Remote` toggle at the
top of the UR Teach Pendant that you can use to switch between modes.

### Enable remote control services

Ensure that the necessary remote control interfaces are enabled on the robot
controller:

1. In **Settings | Security | Services**, unlock the settings using the admin
   password.
2. Enable the **Dashboard Server**, **Primary Client Interface**, and
   **Real-Time Data Exchange (RTDE)** interfaces.
3. Lock the settings and **Exit**.

> [!NOTE]
> Enabling additional services, such as the **Secondary Client Interface** or
> **Real-Time Client Interface**, alongside the required **Dashboard Server**,
> **Primary Client Interface**, and **Real-Time Data Exchange (RTDE)**
> interfaces should not cause configuration conflicts.

If these services are missing, the connection to the robot will fail with an
error like `Failed to connect to 192.170.10.1 in 10s. Ensure the robot is
powered on and the IP is correct.`, even though the robot is in `remote mode`
and can be `ping`ed successfully.

> [!NOTE]
> You can confirm if the Dashboard Server is unreachable by testing the
> connection to the robot's port `29999`:
>
> ```bash
> curl --max-time 3 192.170.10.1:29999
> ```
>
> If the service is missing or disabled, the command times out:
>
> ```
> curl: (28) Connection timed out after 3002 milliseconds
> ```
>
> If the service is enabled, the command prints:
>
> ```
> curl: (1) Received HTTP/0.9 when not allowed
> ```

### Disable fieldbus over Ethernet

**Installation | Fieldbus over Ethernet** needs to be disabled.
Otherwise Intrinsic Core cannot control the robot.

The hardware module may report `Failed to connect`, or the real-time control
service may show `Deadline exceeded while waiting for hardware modules to finish
Prepare`.

The logs of the hardware module contain:

```
ERROR external/universal-robots-client-library+/src/rtde/rtde_client.cpp 182: Caught exception Variable 'speed_slider_mask' is currently controlled by another RTDE client.
The input recipe can't be used as configured. Note: when using a fieldbus (e.g. Ethernet/IP or Modbus), all outputs are claimed by those., while trying to setup RTDE inputs.
...
ERROR external/universal-robots-client-library+/src/rtde/rtde_client.cpp 130: Failed to initialize RTDE client, retrying in 5 seconds
...
Failed to connect: INTERNAL: UR driver error while connecting to the robot: Failed to initialize RTDE client after 3 attempts
```

### Payload and Tool Center Point

The payload and Tool Center Point (TCP) need to be set to appropriate values
using the **Installation** menu, otherwise the motion performance will be bad
and the robot can drift in *free drive* mode.

While the TCP pose of the settings is not used by Intrinsic Core, it is
recommended to set a reasonable value. This enables proper function of Cartesian
jogging using the UR Teach Pendant.

### Safety setup

While Intrinsic does not offer safety advice, we recommend some basic settings.

> [!TIP]
> **Installation | Safety** allows editing the safety configuration.
> Slowly jogging the robot allows testing the setup (tool, tip, elbow).

1. Add a safety plane (**Safety | Planes**) to represent the table top.
   - Add other planes (walls, ceiling) as required.
1. Add a sphere around the TCP to prevent collisions of the planes with the
   tip and tool.

We recommend using the `Safeguard stop` for non safety critical stopping of the
robot. See the [official website](https://www.universal-robots.com/)
for more information.

### Custom network settings

> [!NOTE]
> This guide uses `192.170.10.1` for the robot.
> Adjust this and the `robot_ip` of the
> [module configuration](#configure-the-robot-ip) to match the address of your
> IPC's robot-facing NIC.

Open the network settings from the UR Teach Pendant
**Settings | System | Network** and enter these settings:

```none
Static Address

IP address: 192.170.10.1
Subnet mask: 255.255.255.0
Default gateway: 0.0.0.0
```

## Switch to remote control

The robot needs to be in [`Remote Control`](#configure-remote-control) mode to
be controlled by Intrinsic Core.

> [!WARNING]
> This can lead to unexpected motion if a solution is already running.

The mode can be changed in the top right corner of the UR Teach Pendant.

If the robot is in `Manual` mode, switch to `Remote Control`.

If the button is disabled, you may need to close the active window.

## Add the robot to your solution

To control a UR robot you need two assets in your solution:

- the hardware device for your robot model, which contains the robot geometry
  and the UR hardware module, and
- the real-time control service, which runs the hardware module in a real-time
  control loop.

| Robot model | Bazel target | Asset ID |
| :--- | :--- | :--- |
| UR3e | `@intrinsic-core//incode/intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur3e_hardware_module_core` | `ai.intrinsic.ur3e_hardware_module_core` |
| UR5e | `@intrinsic-core//incode/intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_core` | `ai.intrinsic.ur5e_hardware_module_core` |
| UR10e | `@intrinsic-core//incode/intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur10e_hardware_module_core` | `ai.intrinsic.ur10e_hardware_module_core` |

Define the workcell as an `intrinsic_solution` target that lists both assets
and their instances. A real robot always needs a custom
[configuration](#configure-the-robot-ip), and the solution target is where that
configuration lives:

```python
load("@intrinsic-core//intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("@intrinsic-core//intrinsic/assets/build_defs:solution.bzl", "intrinsic_solution")

intrinsic_asset_instance(
    name = "robot",
    asset = "ai.intrinsic.ur5e_hardware_module_core",
    service_config = "ur5e_config.textproto",
)

intrinsic_asset_instance(
    name = "realtime_control",
    asset = "ai.intrinsic.generic_realtime_control_service",
)

intrinsic_solution(
    name = "my_workcell",
    assets = [
        "@intrinsic-core//incode/intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_core",
        "@intrinsic-core//incode/intrinsic_control/intrinsic/icon/machines/common:generic_icon_mainloop_type",
    ],
    default_operation_mode = "real",
    instances = [
        ":realtime_control",
        ":robot",
    ],
)
```

The real-time control service derives its part configuration from the hardware
modules that are installed in the solution. Use
[`inctl icon config`](#operate-the-robot) to inspect the configuration it is
running with.

### Configure the robot IP

The configuration of the hardware module is an
`intrinsic_proto.icon.HardwareModuleConfig` text proto, referenced from the
`service_config` attribute of the instance. Start from the module's default
configuration,
[`default_config_with_scene_object.pbtxt`](../../../../incode/intrinsic_control/intrinsic/icon/hardware_modules/universal_robots/default_config_with_scene_object.pbtxt),
and adjust `robot_ip` to the address of your robot. You can find the IP address
of the robot on the `About` page of the UR Teach Pendant.

```textproto
# proto-file: intrinsic/icon/hal/proto/hardware_module_config.proto
# proto-message: intrinsic_proto.icon.HardwareModuleConfig

name: "ur_module"
module_config {
  [type.googleapis.com/intrinsic_proto.icon.UniversalRobotsModuleConfig] {
    robot_ip: "192.170.10.1"
    default_advanced_control: {}
  }
}
drives_realtime_clock: true
control_frequency_hz: 500
```

> [!CAUTION]
> Double check the robot configuration ([payload](#payload-and-tool-center-point),
> safety limits) before executing on hardware.

### Deploy the solution

Deploy the solution to the cluster by running the solution target and pointing
it at your installation:

```bash
bazel run //path/to:my_workcell -- --address localhost:17080 --operation_mode=real
```

Use `--operation_mode=sim` to run the same solution in simulation.

> [!NOTE]
> A solution deployed this way is ephemeral: changes made at runtime are lost.
> Make configuration changes in the `intrinsic_solution` target and its
> configuration files, then run the target again.

## Operate the robot

All commands below talk to the installation at `--address localhost:17080`; use
`--address <ipc-address>:17080` when working from another machine.

Check which asset instances are installed and what configuration they use:

```bash
inctl asset instance list --address localhost:17080
inctl asset instance get robot --view full --address localhost:17080
```

Check, restart, or stop the services:

```bash
inctl service state list --address localhost:17080
inctl service state get robot --address localhost:17080
inctl service state restart robot --address localhost:17080
```

Inspect and operate real-time control:

```bash
inctl icon status --address localhost:17080        # operational status, faults, safety status
inctl icon config --address localhost:17080        # the ICON configuration in use
inctl icon list-parts --address localhost:17080
inctl icon clear-faults --address localhost:17080
inctl icon enable --address localhost:17080        # enable all parts
inctl icon disable --address localhost:17080
```

## Expert: use multiple robots on the same IPC

It is possible to control multiple robots from the same IPC using two hardware
modules.

The setup and configuration follows these steps:

1. Configure an additional network card for real-time communication.
2. Adjust the network settings of the second robot accordingly.
3. Add and configure a second hardware module instance.

### Additional real-time NIC

Configure an additional network card of your IPC for the second robot, as in
[Network connection](#network-connection).

> [!CAUTION]
> Choose a different subnet, otherwise the communication will not be cleanly
> separated.
>
> If e.g. the first robot uses `192.170.10.1`, use e.g. `192.168.10.1/24`.

### Robot configuration

Adjust the [network settings](#custom-network-settings) of the robot to use an
IP in your new real-time network.
For the example of `192.168.10.1` from above, choose e.g. `192.168.10.100`.

### Hardware module configuration

After adding a second robot to your solution, adjust the following parameters.

- The `name` parameter needs to differ between the modules.
  Otherwise the hardware module drivers compete for a shared memory namespace
  and one module will report an error like `Unable to lock`, or `Timed out
  waiting for lock`.
  Recommendation: Remove the `name` parameter. This way it uses the instance
  name in your solution.
- One module needs to adjust the settings in the optional `reverse_interface` of
  the configuration.

Example configuration for the second module driver.
All default ports were incremented by 1000. You need to ensure those are free on
your IPC.

```textproto
module_config: {
  [type.googleapis.com/intrinsic_proto.icon.UniversalRobotsModuleConfig]: {
    # The IP to connect to the robot. Can be found on the About page on the
    # Teach Pendant.
    robot_ip: "192.168.10.100"
    # Custom settings for the reverse interface (from robot to IPC).
    # Required on conflicts with the default port range [7141, 7144].
    reverse_interface: {
       # IP of the realtime NIC to connect to the robot. Used for reverse
       # connections (robot to IPC).
       # Empty string uses the same IP as the outgoing connection to the robot.
       reverse_ip: "192.168.10.123"
       # Port that will be opened by the driver to allow direct communication
       # between the driver and the robot controller.
       # Defaults to 7141
       reverse_port: 8141
       # The driver will offer an interface to receive the program's URScript on
       # this port. If the robot cannot connect to this port, `External Control`
       # will stop immediately
       # Defaults to 7142
       script_sender_port: 8142
       # Reverse port used to send trajectory points to the robot.
       # Currently not used by the module. Port is still allocated by the UR
       # driver. Required for using `writeTrajectoryPoint`.
       # Defaults to 7143
       trajectory_port: 8143
       # Reverse port used to send script commands to the robot.
       # Currently not used by the module. Port is still allocated by the UR
       # driver. Required for using `sendScript`.
       # Defaults to 7144
       script_command_port: 8144
    }
  }
}
drives_realtime_clock: true
control_frequency_hz: 500
```

## Tips and common issues

### Home position in joint angles

The joint configuration `[0,0,0,0,0,0]` is not `Candlestick`, but a horizontal
arm position.

A more common `home` position for a UR robot that is similar to a `Candlestick`
pose is `[0,-1.5708,0,-1.5708,0,0]` rad.

### Digital inputs and outputs (DIOs) considerations

> [!NOTE]
> It takes between two to four cycles for a DIO command to be reflected in
> the respective status.

The UR robot controller can experience `high load` when the state of many
digital outputs is changed every cycle for multiple cycles.
It is a well known RTDE issue that the Universal Robots controller
skips sending network packages under high load.

In case the module doesn't receive network packages for a long time (>100ms),
the module **faults** with the message: `Did not receive data from robot.`,
because closed loop control is not possible anymore.
Use `inctl icon clear-faults` to clear the error.

### Control robot in automatic mode

The robot can still be controlled when in `Automatic` mode, when the control
mode on the UR Teach Pendant is switched to `Automatic` after Intrinsic Core is
connected to the robot.

## Documentation and support

Manuals and software are available on the
[Universal Robots web site](https://www.universal-robots.com/download/).

General UR questions are answered in the
[UR support forum](https://www.universal-robots.com/support/).
