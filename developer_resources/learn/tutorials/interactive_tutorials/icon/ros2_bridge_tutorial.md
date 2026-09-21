
# ros2_control compatiblity

If you followed the [Robot
Bringup](/developer_resources/learn/tutorials/interactive_tutorials/icon/robot_bringup.md)
tutorial, you know that the Intrinsic Core (IC) contains hardware modules (HWMs)
for a handful of manufacturers. But what if your favorite robot is not supported
out of the box?

If there is a [`ros2_control`](https://control.ros.org/rolling/index.html)
driver for your robot, the easiest way to get started is to use
[`icon_hwm_controller`](https://github.com/intrinsic-ai/icon-hwm-controller) to
connect that driver to ICON, the Intrinsic realtime control service.

## `icon_hwm_controller` introduction

The diagram below shows how `icon_hwm_controller` fits into the `ros2_control`
and ICON ecosystems:

![Architecture diagram for icon_hwm_controller. The controller lives inside the
ROS2 controller_manager, but connects to ICON via shared-memory
IPC.](icon_hwm_controller_architecture.svg)

-`icon_hwm_controller` slots into the existing `ros2_control` _and_ ICON
architecture by implementing two interfaces:

* `ros2_control`'s
  [`Controller`](https://control.ros.org/rolling/doc/getting_started/getting_started.html#controllers)
  interface (like `joint_trajectory_controller` in a regular ROS2 system)
* ICON's
  [`HardwareModuleInterface`](https://github.com/intrinsic-ai/icon-shared-memory/blob/main/icon/hal/hardware_module_interface.h)

Because ICON connects to hardware modules using a shared-memory IPC mechanism,
ICON and `icon_hwm_controller` can connect to each other, even though the latter
is a plugin within the `ControllerManager` ROS2 node.

## Use `icon_hwm_controller` to connect a robot to ICON

`icon_hwm_controller` comes with some
[examples](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main/icon_hwm_controller_examples),
so if your robot is covered by those, you're in luck. If not, don't worry. It's
relatively easy to add support for any robot model that has a `ros2_control`
driver:

1. Clone https://github.com/intrinsic-ai/icon-hwm-controller and build the
   [`icon_hwm_base` docker container]():

   ```bash
   git clone --recurse-submodules --shallow-submodules https://github.com/intrinsic-ai/icon-hwm-controller.git
   cd icon-hwm-controller
   docker compose -f icon_hwm_controller/docker/docker-compose.yml build
   ```
2. Create a new folder for your driver wrapper. This folder needs a few things
   (the links go to [the UR driver
   example](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main/icon_hwm_controller_examples/ur_ros2_icon_hwm). You
   can use these for reference):

   * [`package.xml`](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main/icon_hwm_controller_examples/ur_ros2_icon_hwm/package.xml)
     and
     [`CMakeLists.txt`](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main/icon_hwm_controller_examples/ur_ros2_icon_hwm/CMakeLists.txt)
     files for a ROS2 package (you will create a launch file and, optionally, a
     ROS2 node in the following steps)
   * [`Dockerfile`](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main/icon_hwm_controller_examples/ur_ros2_icon_hwm/docker/Dockerfile)
     and
     [`docker-compose.yml`](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main/icon_hwm_controller_examples/ur_ros2_icon_hwm/docker/docker-compose.yml)
     to pull in your `ros2_control` driver and build/package them in a container
   * [`BUILD`](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main/icon_hwm_controller_examples/ur_ros2_icon_hwm/BUILD)
     to configure the bazel build for the final Intrinsic assets

   * [FANUC
     Example](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main/icon_hwm_controller_examples/fanuc_ros2_icon_hwm/docker)

   Note that your folder can be standalone. Your Dockerfile should depend on the
   [`icon_hwm_base`](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main/icon_hwm_controller/docker)
   image, which contains everything you need to use `icon_hwm_controller`.

3. Start with a working ROS2 launch file for your robot, then remove all
   controllers other than `joint_state_broadcaster` and
   `joint_trajectory_controller`, plus any others that you need in order to
   manage operational state (fault reporting and clearing).

   Test this launch file to make sure it works with your *real* robot.

4. [Optional] Implement an `OperationalStatus` node for your robot.

   If you are just experimenting, and are fine with restarting your driver
   process manually to recover from things like emergency stops or limit
   violations, you can skip this step. However, having nice error reporting and
   the ability to quickly get up and running after a mishap can do wonders for
   your iteration time, so building an `OperationalStatus` node is time spent
   well if you plan to use your robot with the Intrinsic platform a lot. (Or if
   you *weren't* planning to do that, but notice you're doing it anyway!)

   Since `ros2_control` does not have unified error reporting and handling, each
   robot driver needs a bespoke node that provides two things:

   * An `icon_hwm_controller_msgs/msg/OperationalStatus` publisher that tells
     `icon_hwm_controller` if there is a problem (and, with its `message` field,
     what that problem is).
   * An `std_srvs/srv/Trigger` service server that clears any problems to the
     best of its ability, and returns the robot to a "ready" state.

   Look at the example nodes for
   [UR](https://github.com/intrinsic-ai/icon-hwm-controller/blob/main/icon_hwm_controller_examples/ur_ros2_icon_hwm/src/ur_operational_state_node.cpp)
   and
   [FANUC](https://github.com/intrinsic-ai/icon-hwm-controller/blob/main/icon_hwm_controller_examples/fanuc_ros2_icon_hwm/src/fanuc_operational_state_node.cpp)
   robots to get an idea of what this can look like.

5. Configure `icon_hwm_controller`.

   Usually the configuration for `ros2_control` lives in a file called
   `controllers.yaml`. Add the configuration for `icon_hwm_controller` to that
   file. You need to modify two parts of the file:

   First, add `icon_hwm_controller` to the configuration for
   `ControllerManager`. This tells the
   [`controller_manager/spawner`](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html#spawner)
   node which plugin it should load the controller from.

   ```yaml
   /**:
    ros__parameters:
      update_rate: $(var control_frequency_hz)
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

      # Add these lines:
      icon_controller:
        type: icon_hwm_controller/IconHwmController

      cpu_affinity: $(var cpu_affinity)
      lock_memory: $(var lock_memory)
      thread_priority: $(var realtime_priority_low)
   ```

   Next, add the configuration block for the controller itself (outside of the
   `/**` block from before):

   ```yaml
   /**/icon_controller:
    ros__parameters:
      # The launch file supplies the variables we use here
      name: "$(var hwm_name)"
      context_name: "$(var context_name)"
      shm_namespace: "$(var shm_namespace)"
      cpu_affinity: $(var cpu_affinity)
      lock_memory: $(var lock_memory)
      realtime_priority_low: $(var realtime_priority_low)
      realtime_priority_high: $(var realtime_priority_high)
      control_frequency_hz: $(var control_frequency_hz)
      drives_realtime_clock: $(var drives_realtime_clock)

      # You need to manually configure the values for these (see below)
      dof_names:
        - "J1"
        - "J2"
        - "J3"
        - "J4"
        - "J5"
        - "J6"
      command_interfaces:
        - position
      reference_and_state_interfaces:
        - position
        - velocity
      hardware_component_name: "$(var robot_model)"
      operational_status_topic: /operational_status
      clear_faults_trigger_service: /clear_faults
   ```

   This snippet uses substitution to let the Intrinsic platform parameterize
   your wrapped driver. Check [the definition of these
   parameters](https://github.com/intrinsic-ai/icon-hwm-controller/blob/main/icon_hwm_controller/src/icon_hwm_controller_parameters.yaml,
   ) for documentation on each of them. You will add launch arguments later to
   provide the variables that the configuration uses.

   There are also a few parameters that the platform will *not* provide, so you
   need to decide how to populate them:

   * `dof_names`: The `ros2_control` interface prefixes for each joint, in order
     from base to tip
   * `command_interfaces`: The names of the command interfaces for your robot
     driver. In order:

     1. position
     2. velocity (optional)

     Usually, the interface names are literally "position" and "velocity".

     Note that `icon_hwm_controller` will treat the velocity interface as a
     feedforward if present, i.e. it will write commands to both the position
     and velocity interfaces. If your driver does not support this, do not
     specify a velocity interface name.
   * `reference_and_state_interfaces`: Similar to `command_interfaces`, these
     are the names of the position and velocity _state_ interfaces for your
     robot driver.
   * `hardware_component_name`: The name of the `ros2_control`
     `HardwareComponent` for your robot driver. This must match the `name`
     attribute of your `<ros2_control>` xacro tag. If your launch file uses
     parameter substitution to set that attribute, use the same substitution
     here!
   * `operational_status_topic` and `clear_faults_trigger_service`. These should
     match the topic/service names for your `OperationalStatus` node (see
     above). If you chose to __not__ create that node, omit these two.

6. Update your launch file to

   * start your `OperationalStatus` node
   * register the arguments that `icon_hwm_controller` expects (there is a
     [helper
     function](https://github.com/intrinsic-ai/icon-hwm-controller/blob/main/icon_hwm_controller/icon_hwm_controller/launch.py#L20)
     to do so)
   * spawn `icon_hwm_controller` instead of `joint_trajectory_controller` (make
     sure to spawn it as **inactive**)
   * set `allow_substs=True` when loading the `.yaml` configuration

7. Build and export your docker container, and use it to build an Intrinsic
   service asset.

8. Convert your robot's description to an `.sdf` file, and use that to create an
   Intrinsic geometry asset.

   > [!CAUTION] This is very ["draw the rest of the
   > owl"](https://www.reddit.com/r/funny/comments/eccj2/how_to_draw_an_owl/)
   > for now, flesh out later

9. Combine the service and geometry assets into a single Intrinsic
   `HardwareDevice`

   > [!CAUTION] Turns out there was more owl left to draw!

10. Sideload your `HardwareDevice` into your Intrinsic solution.

   > [!CAUTION] Owl 3: The Owlening ;____;

Well, I did say *relatively* easy. Ten steps is nothing to sneeze at, and some
of these involve writing new code (although most of it is launch files, build
configuration and container definitions). Read on for a detailed walkthrough of
the process with a simple example robot.

### RRBot walkthrough

> [!CAUTION]
> Using RRBot for now, because it's very simple. If it turns out that ICON is
> unhappy with just two joints, we can switch to
> [r6bot](https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html)

Let's walk through all steps using the
[`RRBot`](https://control.ros.org/master/doc/ros2_control_demos/example_1/doc/userdoc.html)
example that `ros2_control` uses. This is a very simple virtual robot that
doesn't require any external hardware or simulators, so there are not many
dependencies.


#### Workspace setup

First, ensure that you have [the tools you will
need](https://github.com/intrinsic-ai/icon-hwm-controller/tree/main#prerequisites). Starting
from a regular Ubuntu installation, you will need to acquire

* [**Git**](https://git-scm.com/install/linux) (`sudo apt install git`) to clone
  the `icon_hwm_controller` repo.
* [**Docker Engine & Docker Compose**](https://docs.docker.com/engine/install/)
  to build and containerize the ROS 2 driver environments.
* [**Bazelisk / Bazel**](https://bazel.build/install/bazelisk) to build the
  Intrinsic Service and Hardware Device assets.
* **`inctl`** (Intrinsic's CLI tool) to install and manage assets on the
  Intrinsic cluster. You can use
  [`env.sh`](/developer_resources/learn/tutorials/interactive_tutorials/icon/files/env.sh)
  to create a convenient alias to use inside your checkout of the Intrinsic Core
  repo.

*(Alternatively, use the [Intrinsic
DevContainer](https://flowstate.intrinsic.ai/docs/guides/build_with_code/set_up_your_development_environment/local_environment/). The
setup instructions include Docker, and the container itself includes Bazel and
`inctl`.)*

Now, clone https://github.com/intrinsic-ai/icon-hwm-controller, including its
submodules:

```bash
git clone --recurse-submodules --shallow-submodules https://github.com/intrinsic-ai/icon-hwm-controller.git
```

For a quick sanity check, make sure that the `icon_hwm_base` container builds to
start with (this can take a while):

```bash
cd icon-hwm-controller
docker compose -f icon_hwm_controller/docker/docker-compose.yml build
```

Once this succeeds, `icon_hwm_base` is available in your local Docker registry,
and you'll be able to depend on it in your own Dockerfile.

> [!NOTE]
> Do not attempt to build the examples using `bazel`. That will **not** work at
> this point, because they rely on docker images that are too big to commit to
> Git, and impossible to build cleanly with Bazel. If you want to run one of the
> examples, follow the steps in its `README.md` file.

#### Create a folder for your robot

Simple enough:

```bash
mkdir rrbot_ros2_icon_hwm
cd rrbot_ros2_icon_hwm

# You don't have to use this exact folder structure, but it's a good start.
mkdir -p config docker include/rrbot_ros2_icon_hwm launch proto src test

touch BUILD CMakeLists.txt package.xml

cp ../ur_ros2_icon_hwm/.dockerignore .
```

#### Set up the build configuration

Throughout this walkthrough, expand the sections for each file name to see its
full content.

<details> <summary><strong>package.xml</strong></summary>

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rrbot_ros2_icon_hwm</name>
  <version>0.1.0</version>
  <description>
    Configuration files for an ICON hardware module based on the RRBot ros2_control example.
  </description>
  <maintainer email="your.email@goes.here">Your Name</maintainer>

  <!-- Of course, feel free to use whatever license fits your use case -->
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- build-time dependencies for the OperationalStatus node -->
  <depend>icon_hwm_controller_msgs</depend>
  <depend>icon_shared_memory_vendor</depend>

  <!-- standard ROS2 deps -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>

  <!-- RRBot is defined in this package. Most real robots require several packages -->
  <depend>ros2_control_demo_example_1</depend>

  <!-- execution-time dependency for icon_hwm_controller itself -->
  <exec_depend>icon_hwm_controller</exec_depend>

  <!-- other dependencies for the launch file -->
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>ros2_control_demo_description</exec_depend>
  <exec_depend>ros2launch</exec_depend>
  <exec_depend>xacro</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
</details>

<details> <summary><strong>CMakeLists.txt</strong></summary>

```cmake
cmake_minimum_required(VERSION 3.8)
project(rrbot_ros2_icon_hwm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20 standard.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(icon_hwm_controller_msgs REQUIRED)
find_package(icon_shared_memory_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros2_control_demo_example_1 REQUIRED)

add_executable(
  rrbot_operational_state_node
  src/rrbot_operational_state_node.cpp
)
target_include_directories(rrbot_operational_state_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rrbot_operational_state_node PUBLIC
  ${icon_hwm_controller_msgs_TARGETS}
  icon_shared_memory_icon_utils_mutex
  icon_shared_memory_icon_utils_status
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

install(TARGETS
  rrbot_operational_state_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```
</details>

<details> <summary><strong>Dockerfile</strong></summary>

```dockerfile
FROM icon_hwm_base:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

COPY . ${AMENT_WORKSPACE_DIR}/src/rrbot_ros2_icon_hwm

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && git clone --depth 1 https://github.com/ros-controls/ros2_control_demos.git \
    ${AMENT_WORKSPACE_DIR}/src/ros2_control_demos \
 && git clone --depth 1 https://github.com/pal-robotics/urdf_test.git \
    ${AMENT_WORKSPACE_DIR}/src/urdf_test \
 && apt-get update \
 && rosdep update

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep install -iy \
    --from-path $(colcon list --packages-up-to rrbot_ros2_icon_hwm --paths-only) \
    --ignore-src \
    --rosdistro ${ROS_DISTRO} \
 && colcon build --base-paths ${AMENT_WORKSPACE_DIR} \
    --packages-up-to rrbot_ros2_icon_hwm \
 && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog
```
</details>

<details> <summary><strong>docker-compose.yml</strong></summary>

```yaml
services:
  rrbot_ros2_icon_hwm:
    build:
      context: ..
      dockerfile: docker/Dockerfile
    container_name: rrbot_ros2_icon_hwm
    image: rrbot_ros2_icon_hwm:latest
    tty: true
    entrypoint: ["/bin/bash", "-c", "source ./install/setup.bash && exec \"$@\"", "--"]
    command: >
      colcon test --packages-select rrbot_ros2_icon_hwm
        --event-handlers console_direct+ &&
      colcon test-result --all
```
</details>

With these files set up, we can almost test the container build (we will
populate `BUILD` later). Add some dummy code to
`src/rrbot_operational_state_node.cpp` to make sure it compiles:

```c++
int main(int argc, char ** argv)
{
  return 0;
}
```

Now build the container, and run it (this runs the automatic linter tests from
`CMakeLists.txt`):

```bash
docker compose -f docker/docker-compose.yml run --build --rm --remove-orphans \
    rrbot_ros2_icon_hwm
```

#### Add a minimal launch file

[`rrbot.launch.py`](https://github.com/ros-controls/ros2_control_demos/blob/master/example_1/bringup/launch/rrbot.launch.py)
is already pretty bare-bones.  There's not much to trim here, but we won't need
RViz.

After removing RViz, your launch file should look like this:

<details> <summary><strong>launch/rrbot.launch.py</strong></summary>

```python
# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'joint_prefix',
                default_value='joint',
                description='Prefix for joint names (used with allow_substs in spawner).',
            ),
            # Control node
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[{'update_rate': 10}],
                output='both',
            ),
            # robot_state_publisher with robot_description from xacro
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='both',
                parameters=[
                    {
                        'robot_description': Command(
                            [
                                'xacro',
                                ' ',
                                PathSubstitution(FindPackageShare('ros2_control_demo_example_1'))
                                / 'urdf'
                                / 'rrbot.urdf.xacro',
                            ]
                        )
                    }
                ],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--param-file',
                    PathSubstitution(FindPackageShare('ros2_control_demo_example_1'))
                    / 'config'
                    / 'rrbot_controllers.yaml',
                ],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                parameters=[
                    {'joint_prefix': LaunchConfiguration('joint_prefix')},
                    ParameterFile(
                        PathSubstitution(FindPackageShare('ros2_control_demo_example_1'))
                        / 'config'
                        / 'rrbot_controllers.yaml',
                        allow_substs=True,
                    ),
                ],
                arguments=[
                    'forward_position_controller',
                ],
            ),
        ]
    )
```
</details>

Make sure the launch file starts without errors in your container:

```bash
docker compose -f docker/docker-compose.yml run --build --rm --remove-orphans \
    rrbot_ros2_icon_hwm \
    ros2 launch rrbot_ros2_icon_hwm rrbot.launch.py
```


#### `OperationalStatus` node

`RRBot` doesn't actually report any status beyond the managed node lifecycle
state, so this `OperationalStatus` node is a dummy implementation that you can
build on for your own hardware:

<details>
<summary><strong>src/rrbot_operational_status_node.cpp</strong></summary>

```c++
#include <chrono>
#include <memory>

#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

void clear_faults(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*unused*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Received ClearFaults request, sending back a successful response");
  response->success = true;
}

void publish_happy_status(
  rclcpp::Publisher<icon_hwm_controller_msgs::msg::OperationalStatus>::SharedPtr publisher)
{
  icon_hwm_controller_msgs::msg::OperationalStatus operational_status;
  operational_status.state = icon_hwm_controller_msgs::msg::OperationalStatus::ENABLED;
  // The `message` field can be empty for `ENABLED`, but should hold a helpful
  // error message in other states.
  operational_status.message = "";
  publisher->publish(operational_status);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("rrbot_operational_status");

  // Create ClearFaults service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
    node->create_service<std_srvs::srv::Trigger>("clear_faults", &clear_faults);

  // Create OperationalStatus publisher and timer
  auto publisher = node->create_publisher<icon_hwm_controller_msgs::msg::OperationalStatus>(
    "operational_status", 10);
  auto timer = node->create_wall_timer(
    500ms, [&publisher](){
      publish_happy_status(publisher);
    });

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
```
</details>

#### `icon_hwm_controller` configuration

Next, we have to make a copy of [`rrbot_controllers.yaml` from the
`ros2_control`
examples](https://github.com/ros-controls/ros2_control_demos/blob/master/example_1/bringup/config/rrbot_controllers.yaml),
and add configuration value for `icon_hwm_controller`:

<details> <summary><strong>config/rrbot_controllers.yaml</strong></summary>

```yaml
joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster
    update_rate: 50  # Hz
    
forward_position_controller:
  ros__parameters:
    type: forward_command_controller/ForwardCommandController
    joints:
      - $(var joint_prefix)1
      - $(var joint_prefix)2
    interface_name: position

icon_hwm_controller:
  ros__parameters:
    type: icon_hwm_controller/IconHwmController
    # The launch file supplies the variables we use here
    name: "$(var hwm_name)"
    context_name: "$(var context_name)"
    shm_namespace: "$(var shm_namespace)"
    cpu_affinity: $(var cpu_affinity)
    lock_memory: $(var lock_memory)
    realtime_priority_low: $(var realtime_priority_low)
    realtime_priority_high: $(var realtime_priority_high)
    control_frequency_hz: $(var control_frequency_hz)
    drives_realtime_clock: $(var drives_realtime_clock)

    # RRBot uses a launch argument to generate joint names!
    dof_names:
      - $(var joint_prefix)1
      - $(var joint_prefix)2
    command_interfaces:
      - position
    reference_and_state_interfaces:
      # RRBot only provides a position state interface
      - position # This name is hard-coded in #
    https://github.com/ros-controls/ros2_control_demos/blob/master/example_1/description/urdf/rrbot.urdf.xacro#L27
    hardware_component_name: "RRBot" operational_status_topic:
    /operational_status clear_faults_trigger_service: /clear_faults ```
    </details>

#### Update the launch file

We want to update our launch file to do a few things:

1. Add the new launch arguments that `rrbot_controllers.yaml` now uses.

   To do that, use the `get_icon_hwm_launch_arguments()` helper:
   
   ```python
   # At the top of the launch file
   from icon_hwm_controller.launch import get_icon_hwm_launch_arguments   

   # ...

   def generate_launch_description():
       return LaunchDescription(
           # Prepend the launch arguments to the other launch description items.
           get_icon_hwm_launch_arguments() +
           [ ...
   ```
   
2. Spawn `icon_hwm_controller` instead of `forward_position_controller`, and use
   the modified `rrbot_controllers.yaml` to provide parameters:
   
   ```python
   # This replaces the spawner that launched `forward_position_controller`.
   Node(
       package='controller_manager',
       executable='spawner',
       parameters=[
           {'joint_prefix': LaunchConfiguration('joint_prefix')},
           ParameterFile(
               PathSubstitution(FindPackageShare('rrbot_ros2_icon_hwm'))
               / 'config'
               / 'rrbot_controllers.yaml',
               allow_substs=True,
           ),
       ],
       arguments=[
           '--inactive',
           'icon_hwm_controller',
       ],
   ),
   ```

3. Start the OperationalStatus node

   ```python
   Node(
       package='rrbot_ros2_icon_hwm',
       executable='rrbot_operational_state_node',
       output='screen',
   ),
   ```

With these changes, rebuild and run your container again:

```bash
docker compose -f docker/docker-compose.yml run --build --rm --remove-orphans \
    rrbot_ros2_icon_hwm \
    ros2 launch rrbot_ros2_icon_hwm rrbot.launch.py hwm_name:="icon_hwm"
```

> [!NOTE]
> The new `hwm_name` parameter is important! Without it, the launch file will
> not start.


Your launch file should start without errors, and you should see log messages
like these (note that the socket path includes the `hwm_name` you specified when
you started the launch file):

```
[ros2_control_node-1] [INFO] [1788973704.630322092] [controller_manager]: Loading controller : 'icon_hwm_controller' of type 'icon_hwm_controller/IconHwmController'
[ros2_control_node-1] [INFO] [1788973704.630430432] [controller_manager]: Loading controller 'icon_hwm_controller'
[ros2_control_node-1] [INFO] [1788973704.638885842] [controller_manager]: Controller 'icon_hwm_controller' node arguments: --ros-args --params-file /tmp/launch_params_ay22624n --params-file /tmp/launch_params_nduyr2eq --params-file /tmp/launch_params_56e8orby 
[spawner-4] [INFO] [1788973704.696788409] [spawner_icon_hwm_controller]: Loaded icon_hwm_controller
[ros2_control_node-1] [INFO] [1788973704.699077199] [controller_manager]: Configuring controller: 'icon_hwm_controller'
[ros2_control_node-1] [INFO] [1788973704.708156458] [icon_hwm_controller]: Acquired exclusive lock '/tmp/intrinsic_icon/icon_hwm.lock'.
[ros2_control_node-1] [INFO] [1788973704.710050708] [icon_hwm_controller]: Preparing message 1 of 1
[ros2_control_node-1] [INFO] [1788973704.710122918] [icon_hwm_controller]: Message 1 of 1 has 27 FDs
[ros2_control_node-1] [INFO] [1788973704.710189478] [icon_hwm_controller]: Socket '/tmp/intrinsic_icon/icon_hwm.sock' is serving 27 descriptors.
[ros2_control_node-1] [INFO] [1788973704.710405408] [icon_hwm_controller]: Waiting for new connection
```

#### Build an Intrinsic service asset from your ROS2 container

To package the `rrbot_ros2_icon_hwm` docker container into an Intrinsic service
asset, first create and populate a `MODULE.bazel` and `BUILD` files at the root
of your directory:

<details> <summary><strong>MODULE.bazel</strong></summary>


> [!CAUTION]
> The commit hash is outdated. This version of the SDK doesn't work without a
> manual patch, which is very ugly. Once
> https://github.com/intrinsic-ai/insrc/pull/54779 lands, we can use that.

```bazel
module(name = "rrbot_ros2_hwm_asset")

bazel_dep(name = "ai_intrinsic_sdks")
archive_override(
    module_name = "ai_intrinsic_sdks",
    # To pin a version change the following to, e.g.:
    #   urls = "https://github.com/intrinsic-ai/sdk/archive/refs/tags/intrinsic.platform.20221231.RC00.tar.gz",
    #   strip_prefix = "sdk-intrinsic.platform.20221231.RC00/"
    # This is the exact commit that the patch below was created for.
    strip_prefix = "sdk-9ebd695b70a594096f0bbaa6a34a81ff45c5e32d/",
    urls = ["https://github.com/intrinsic-ai/sdk/archive/9ebd695b70a594096f0bbaa6a34a81ff45c5e32d.tar.gz"],
    integrity = "sha256-oeyuYEHnV7pzOgpVOTuyCNflBaz4YVhh2duk9e3hkoY=",
)
bazel_dep(name = "icon_hwm_controller")
local_path_override(
    module_name = "icon_hwm_controller",
    path = "../icon-hwm-controller",
)

bazel_dep(name = "platforms", version = "1.1.0")
bazel_dep(name = "protobuf", version = "36.0.bcr.1", repo_name = "com_google_protobuf")
```

</details>

<details> <summary><strong>BUILD</strong></summary>

```bazel
load("@ai_intrinsic_sdks//bazel:container.bzl", "container_image", "container_import")
load("@ai_intrinsic_sdks//bazel:python_oci_image.bzl", "python_layers")
load("@ai_intrinsic_sdks//intrinsic/assets/services/build_defs:services.bzl", "intrinsic_service")
load("@ai_intrinsic_sdks//intrinsic/icon/hal/bzl:resources.bzl", "hardware_module_manifest")

package(default_visibility = ["//visibility:public"])

container_import(
    name = "fanuc_ros2_icon_hwm_oci",
    # Export this image from your local docker registry using
    # docker image save rrbot_ros2_icon_hwm:latest -o icon_hwm.tar
    tarball = "icon_hwm.tar",
)

# Next, combine the ROS2 container with the Intrinsic entry point
entrypoint_layers = python_layers(
    name = "entrypoint_layers",
    binary = "@icon_hwm_controller//icon_hwm_controller:entrypoint_bin",
)
container_image(
    name = "rrbot_ros2_icon_hwm_image",
    base = ":rrbot_ros2_icon_hwm_oci",
    entrypoint = ["/icon_hwm_controller/entrypoint_bin"],
    layers = entrypoint_layers,
)

# The manifest tells the Intrinsic platform which capabilities a service has
hardware_module_manifest(
    name = "rrbot_ros2_icon_hwm_manifest",
    image = ":rrbot_ros2_icon_hwm_image.tar",
    image_sim = ":rrbot_ros2_icon_hwm_image.tar",
    manifest = "proto/rrbot_ros2_icon_hwm_manifest.textproto",
)
intrinsic_service(
    name = "rrbot_ros2_icon_hwm_service",
    default_config = "proto/rrbot_ros2_icon_hwm_default_config.textproto",
    images = [
        ":rrbot_ros2_icon_hwm_image.tar",
    ],
    manifest = ":rrbot_ros2_icon_hwm_manifest",
    deps = [
        "@icon_hwm_controller//icon_hwm_controller:ros2_hwm_config_proto",
        "@ai_intrinsic_sdks//intrinsic/assets/services/proto/v1:service_state_proto",
        "@ai_intrinsic_sdks//intrinsic/icon/hal:hardware_module_config_proto",
    ],
)
```

</details>

Next, create the two new files in the `proto` directory that the `BUILD` file
references:

The [`ServiceManifest`
proto](/intrinsic_apis/intrinsic/assets/services/proto/service_manifest.proto#L24)
tells the Intrinsic platform how to run your service, and also contains metadata
about an Intrinsic service, like the name and vendor, as well as a short
description.

In this case, you only need to manually provide the metadata, since the
[`intrinsic_service`
rule](/intrinsic/assets/services/build_defs/services.bzl#L117)
fills in the functional parts of the manifest. Check out the proto definition
for `ServiceManifest` and its submessages to see some of the advanced options,
like offering
[gRPC](/intrinsic_apis/intrinsic/assets/services/proto/service_manifest.proto#L44)
and
[HTTP](/intrinsic_apis/intrinsic/assets/services/proto/service_manifest.proto#L63)
servers.

<details>
<summary><strong>rrbot_ros2_icon_hwm_manifest.textproto</strong></summary>

```textproto
metadata {
  id {
    package: "org.ros.example"
    name: "rrbot_ros2_icon_hwm"
  }
  vendor {
    display_name: "$YOUR_NAME"
  }
  documentation {
    description: "RRBot ROS 2 Control Hardware Module."
  }
  display_name: "RRBot ROS 2 HWM"
}
```
</details>

The default configuration is what new instances of a service start out with. You
should craft this so that things work out of the box as much as possible and,
failing that, add helpful comments to make it easy for users to fill in any
missing parts.

The configuration for a service is an [`Any`
proto](https://github.com/protocolbuffers/protobuf/blob/main/src/google/protobuf/any.proto)
because each service can have its own configuration message. That said, Hardware
Modules (HWMs) all use
[`intrinsic_proto.icon.HardwareModuleConfig`](/intrinsic_apis/intrinsic/icon/hal/proto/hardware_module_config.proto#L13)
because they share some configuration options.

That proto again has an `Any` member called `module_config` for HWM-specific
configuration. For `icon_hwm_controller` HWMs, `module_config` is an
[`intrinsic_proto.services.Ros2HwmConfig`](https://github.com/intrinsic-ai/icon-hwm-controller/blob/main/icon_hwm_controller/proto/ros2_hwm_config.proto)
proto, which tells the entry point script which launch file to start, and what
additional launch arguments to set.

<details>
<summary><strong>rrbot_ros2_icon_hwm_default_config.textproto</strong></summary>

```textproto
# proto-file: google/protobuf/any.proto
# proto-message: google.protobuf.Any

[type.googleapis.com/intrinsic_proto.icon.HardwareModuleConfig] {
  drives_realtime_clock: true
  # cf. https://github.com/ros-controls/ros2_control_demos/blob/master/example_1/bringup/config/rrbot_controllers.yaml#L4
  control_frequency_hz: 50.0
  module_config {
    [type.googleapis.com/intrinsic_proto.services.Ros2HwmConfig] {
      launch_package: "rrbot_ros2_icon_hwm"
      launch_file: "rrbot.launch.py"
      # rrbot doesn't need them, but for more complex launch files you can 
      # define additional launch arguments like so (each entry of launch_parameters
      # consists of a string key and a string value):
      # launch_parameters {
      #   key: "ip_address"
      #   value: "192.170.10.1"
      # }
      # launch_parameters {
      #   key: "num_joints"
      #   value: "2"
      # }
      icon_hwm_controller_config {
        lock_memory: true
        shm_namespace: ""
        realtime_priority_low: 40
        realtime_priority_high: 45
      }
    }
  }
}
```
</details>

With these files in place, you can build your service:

```bash
bazel build rrbot_ros2_icon_hwm_service
```
