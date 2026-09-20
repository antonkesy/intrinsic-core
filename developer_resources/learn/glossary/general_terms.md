# General terms

This glossary contains general, technical terminology that is used across the
stack, in documents and in communication. These are standard terms from external
literature. In many cases, there is some variation in terminology for the same
thing. Therefore, Intrinsic in some cases makes distinct choices of one term to
represent one thing. For example, "[joint configuration](#joint-configuration)"
also goes by many other names in different contexts and authors ([joint](#joint)
position, position, joint vector to name a few). Intrinsic uses the term
"[joint configuration](#joint-configuration)".

For Intrinsic-specific terminology, see [Intrinsic terms](intrinsic_terms.md).

### 1P (first party) vs 3P (third party)

1P components are entities (e.g. [assets](intrinsic_terms.md#asset),
[skills](intrinsic_terms.md#skill), [services](intrinsic_terms.md#service)) that
have been implemented by the Intrinsic engineering team. In contrast, 3P modules
have been contributed by external parties, such as partners or users.

### Actuator

An actuator is a physical motor that drives the motion of a [robot](#robot)
[joint](#joint) between two [robot](#robot) [links](#link). An actuator can also
refer to the combination of a motor, transmission mechanism, (such as a gearbox)
and in-built sensors (such as hall-effect sensors and encoders).

### Application Programming Interface (API)

A well-defined interface between components which are specified as
[services](intrinsic_terms.md#service) or [libraries](#library).

### Approach pose

A location close to a work point or [path](#path) that is confidently
collision-free, given allowable or expected tolerances from a
[nominal](#nominal) state.

### Architecture

An organizational structure of a system that describes the relationships and
interactions between the system's elements. Architectural aspects can be found
at different levels of abstraction. Software architecture acts as the skeleton
of a system, influences quality attributes, is orthogonal to functionality, and
uses constraints to influence a system's properties.

### Circularity

A [revolute joint](#revolute-joint) contains a circularity if its joint limits
allow it to rotate by 360 degrees or more, allowing one or more [joint](#joint)
positions to produce the same physical angle. [Joints](#joint) with
circularities therefore result in more possible inverse kinematics solutions
than [joints](#joint) without circularities.

### Cluster

A set of computers that work together so that they can be viewed as a single
system.

### Configuration

A configuration may refer to:

*   A set of values to parametrize a [software component](#software-component)
*   A set of [joint](#joint) angles or
    [joint configuration](#joint-configuration)

### Control domain

A collection of [degrees of freedom (DOF)](#degree-of-freedom-dof) that can be
controlled in tight coordination.

### Controller

Generically, any device or software that drives real or simulated
[actuators](#actuator).

### Cycle time

The expected repeatable execution time for a specific task.

### Degree of freedom (DOF)

One of the independent parameters that define the configuration of a system. In
the Intrinsic context, it is typically an axis of motion of a robot.

### End effector

Synonymous with end of arm tool (EoAT).

### End of arm tool (EoAT)

A device that is attached to a robot's [flange](#flange). An example of an EoAT
is a spot welder, dispense nozzle, or a gripper. It is also called an
end-effector.

### Fault analysis

When a fault is detected, the overall system is analyzed to determine the cause
of the fault and its implications.

### Fault detection

Detecting when actual execution diverts from the nominal expected execution.
This generates a general signal that something is going wrong.

### Flange

Typically, the mounting area on the end of a robot's arm which the
[end of arm tool (EoAT)](#end-of-arm-tool-eoat) is attached to.

### Force/torque sensor

A sensor capable of measuring the force and torque applied to it. These sensors
are typically mounted on the robot as part of the
[end of arm tools](#end-of-arm-tool-eoat) to detect the amount of force being
applied by the robot on a workpiece. The typical output includes forces along 3
Cartesian axes measured in Newtons, and torques on the same axes measured in
Newton-Meters.

### Frame

A named pose relative to some other pose. Usually used to denote common
attachment points or expected tool locations. For example, one common frame is
the "flange" of a robot, which refers to the robot's
[end-effector](#end-effector).

### Framework

A software abstraction provided through [libraries](#library) and
[Application Programming Interfaces (APIs)](#application-programming-interface-api)
which enable users to selectively extend the functionality of specific
components. A key feature of a framework is inversion of control. In Intrinsic's
architecture, an example is that users can provide additional behavior via
[skills](intrinsic_terms.md#skill) and
[services](intrinsic_terms.md#service), but the framework invokes them through
the [executive](intrinsic_terms.md#executive).

### Inverse kinematics

The process of computing one or more
[joint configurations](#joint-configuration) in a
[kinematic chain](#kinematic-chain) that result in a given transform between two
rigid bodies in the chain.

### Joint

The moveable parts in the [robot](#robot) hardware. The most common joint types
are those that can rotate, like a hinge, called
[revolute joints](#revolute-joint), and those that can extend linearly, like a
drawer slide or telescopic pole, called [prismatic joints](#prismatic-joint).
Joints contain the actuators that make the robot move.

### Joint configuration

The positions of one or more [joints](#joint), usually corresponding to
[joints](#joint) in a [kinematic chain](#kinematic-chain) (cf.
[degree of freedom (DOF)](#degree-of-freedom-dof)). Note that for
[revolute joints](#revolute-joint), this is not synonymous with the
[joint's](#joint) angle, since [joints](#joint) with
[circularities](#circularity) have multiple positions that produce the same
angle.

### Joint torque sensor

A sensor mounted between two robot links as part of the [joint](#joint)
mechanism that allows direct measurement of the torque applied by the
[actuator](#actuator) on the output link. The sensed quantity is output force
for a [prismatic joint](#prismatic-joint), and output torque for a
[revolute joint](#revolute-joint) or spherical [joint](#joint). Joint torque
sensors are required for closed-loop joint torque control on [robots](#robot).

### Kinematic chain

An alternating sequence of rigid bodies ([links](#link)) and [joints](#joint)
that constrain the relative poses between their neighboring bodies. At
Intrinsic, a kinematic chain does not contain branches (more than two
[joints](#joint) connected to the same body) or loops (multiple paths of
[joints](#joint) connecting the same pair of bodies).

### Kinematic branch

A classification of a [joint configuration](#joint-configuration), such that
[joint configurations](#joint-configuration) that are separated by a
[singularity](#singularity) are said to lie on different kinematic branches.
Constraining a [robot](#robot) to remain on one or more kinematic branches can
therefore prevent the robot from crossing some [singularities](#singularity).

### Kubernetes (K8s)

A system for automating deployment, scaling, and management of containerized
applications.

### Library

A collection of programs and software packages made generally available, often
loaded and stored on disk for immediate use.

### Link

A rigid mechanical part connecting the [joints](#joint). The [links](#link) give
the [robot](#robot) its shape and make up the bulk of what is visible when
looking at a [robot](#robot).

### Low latency

Operations which must be carried out in a short time, typically (far) less than
one second. Some jitter in loop or execution time is acceptable. For stricter
requirements, see [real-time](#real-time).

### Margin

See [tolerance](#tolerance).

### MES (Manufacturing Execution System)

Software that tracks and documents parts as they move through a plant or
facility, possibly routing and managing the production flow (ISA-95, Level 3).

### Mean Time Between Failure (MTBF)

The average time elapsed between failures. Calculated as total operational time
divided by total number of failures.

### Mean Time to Repair (MTTR)

A basic measure of the maintainability of repairable items. It represents the
average time required to repair a failed component or device - including repair,
testing, and return to normal operating condition. Calculated as total
maintenance time divided by total number of repairs.

### Motion planning

*   Input: start and target [pose](#pose), work space representations with
    obstacles
*   Output: collision-free [path](#path) (or error if none can be found)
*   Find a sequence of [joint configurations](#joint-configuration) that take an
    [end-effector](#end-effector) from the start to the target [pose](#pose)
    without colliding with an obstacle.
*   Motion planning can be done [offline](#offline) (perform planning ahead of
    time before the robot starts operating) or [online](#online) (calculate or
    adapt the path, or both, during robot motions in order to react to and
    interact with dynamic environments; this means a robot may move along a path
    that has not necessarily been computed completely, and which may change
    during the movement). The term real-time path planning is used for online
    path planning algorithms that are deterministic and can be executed within a
    worst-case computation time.
*   May include aspects of [trajectory generation](#trajectory-generation).

### Nominal

The expected or ideal state without real-world input or error.

### Offline

Describes aspects performed ahead of time before execution starts.

### Offline Programming (OLP)

A [robot](#robot) programming method in which the program for the
[robot](#robot) is written outside the production process on an external PC and
independent of the actual hardware.

### Online

Describes aspects performed at run-time while the [robot](#robot) is operating.

### Original Equipment Manufacturer (OEM)

A company that produces non-aftermarket parts and equipment that may be marketed
by another manufacturer.

### Path

A path defines a set of points with regards to a reference frame from a start to
a target (e.g., Cartesian [pose](#pose) or [joint](#joint) coordinates). It does
not contain any information about numeric data such as time or velocity (cf.
[trajectory](#trajectory)). For example, paths can be represented by splines,
piecewise consistent polynomials, or in the most simple case, a set of
waypoints.

### Parallel-chain robot

Refers to the kinematic topology of a [robot](#robot) where at least one of the
[links](#link) is connected to another [link](#link) that is neither its parent
[link](#link) nor its child [link](#link) to form a closed loop. An example is
the delta [robot](#robot). In contrast to serial-chain [robots](#robot),
parallel-chain [robots](#robot) typically have more movable [joints](#joint)
than the number of [actuators](#actuator). This is because the number of
[degrees of freedom (DOFs)](#degree-of-freedom-dof) is smaller than the number
of [joints](#joint) (Grubler's Formula) when a kinematic closed loop is present.
Parallel-chain robots typically have a smaller workspace compared to
serial-chain robots, but can achieve higher movement speeds and handle larger
payloads.

### Perception

The extraction of semantic meaning from raw sensor data. Software modules that
are concerned with acquiring information about the world. Perception
[services](intrinsic_terms.md#service) and [skills](intrinsic_terms.md#skill)
can store perceived information in the
[world](../platform_introduction/world_concepts.md).

### Planning

Planning is a term with many meanings in a robotics context. It should hence be
used with care and avoided for new concepts. Examples of well-known established
uses are:

*   Task planning: Causal relations
*   Temporal planning: Folds (some aspects of) scheduling into the task planning
    process
*   [Motion planning](#motion-planning): Combined phrase for path planning and
    [trajectory generation](#trajectory-generation)

### Pod

A set of one or more containerized services running on a
[Kubernetes](#kubernetes-k8s) [cluster](#cluster).

### Pose

A pose is position and orientation in Euclidean space.

### Position control

A method of control where the [controller](#controller) regulates the position
of one or more [degrees of freedom (DOF)](#degree-of-freedom-dof) to track a
wanted reference position vector. Velocity control is a related method where the
regulated quantity is the first time-derivative of the position, such as the
velocity. This is the only control mode supported on
[position-controlled robots](#position-controlled-robot).

### Position-controlled robot

While all robots support [position control](#position-control), the term
position-controlled robot is typically used to describe robot arms without
[joint torque sensors](#joint-torque-sensor). [Joints](#joint) on
position-controlled robots are typically actuated using servo or stepper motors
with high gear ratios. This leads to high precision but poor backdrivability.

### Prismatic joint

A [joint](#joint) mechanism between two [robot](#robot) [links](#link) that
allow translation between the two [links](#link). This [joint](#joint) adds one
[degree of freedom (DOF)](#degree-of-freedom-dof) motion between the
[links](#link) and to the overall [robot](#robot) DOF. Prismatic joint positions
(i.e. distances) are measured in meters as part of the
[joint configuration](#joint-configuration). This type of [joint](#joint) is
typically actuated by a linear [actuator](#actuator) or a ball
screw/rack-and-pinion/belt drive connected to a rotary actuator.

### Product Lifecycle Management (PLM)

Software that manages all information regarding a product. Often integrates
business processes and data into a single platform.

### Programmable Logic Controller (PLC)

A rugged computer used for industrial automation to automate different
electro-mechanical processes. It receives information from connected input
devices and sensors, processes the received data, and triggers required outputs
as per its pre-programmed parameters.

### Reactive

Operation in the now that infers an immediate action to respond to sensor
inputs. Durations are typically sub-second.

### Real-time

Software that runs in real-time executes some piece of code and delivers the
output within a guaranteed deadline. Real-time software usually runs some sort
of loop at a high rate, but note that the defining characteristic here is
deterministic or bounded execution time, not the high rate. Motion
[controllers](#controller) need to operate in real-time in order to provide
timely instructions.

### Revolute joint

A joint mechanism between two [robot](#robot) [links](#link) that allow relative
rotation between the two [links](#link) on a fixed axis ([joint](#joint) axis).
Revolute joint positions (i.e. angles) are measured in radians as part of the
[joint configuration](#joint-configuration). Also known as a hinge or pin
[joint](#joint). This type of joint is typically actuated by a rotary
[actuator](#actuator), either attached directly or through a belt or cable or
capstan drive.

### Robot

A unit of multiple [degrees of freedom (DOF)](#degree-of-freedom-dof) actuation
hardware driven by a [controller](#controller).

### Sampled waypoints

All of the [waypoints](#waypoint) required for a robot to complete a
[path](#path).

### SCADA (Supervisory Control and Data Acquisition)

A system that provides insight into what is taking place on a plant floor
(ISA-95, Level 2).

### SDK

An SDK is a set of APIs potentially accompanied by command line tools.

### Sensing

The act of acquiring information about the world. Active sensing takes action in
order to acquire data, e.g., looking at an object from multiple sides. Passive
sensing observes the world as the regular task is carried out. Passive is the
more typical case.

### Simulation

Imitation of the operation of a real-world process or system over time.
Simulations require the use of models; the model represents the key
characteristics or behaviors of the selected system or process, whereas the
simulation represents the evolution of the model over time. Physics and/or
sensor simulation also requires a simulator or simulation engine capable of
modeling physics-based dynamics, actuator emulation and sensor emulation.

### Singularity

For a given [kinematic chain](#kinematic-chain), a singularity is a transform
between two bodies in the chain that results in an infinite number of
[inverse kinematics](#inverse-kinematics) solutions. In literature, this is
sometimes called an internal singularity or a "joint space singularity". See
also [kinematic branch](#kinematic-branch).

### Software component

*   Software components describe software entities or blocks in the overall
    system.
*   A software component is the unit of composition that provides functionality
    to the system through formally defined services at a certain level of
    abstraction.
*   In the literature, some properties of software components are described:
    *   A software component is a binary (non-source-code) unit of deployment.
    *   A software component implements (one or more) well-defined interfaces.
    *   A software component provides access to an interrelated set of
        functionalities.
    *   A software component may have its behavior customized in well-defined
        manners without access to the source code.
*   Intrinsic also uses this term to refer to the unit of source code that
    implements a software component.
    *   A software component can also refer to the abstract concept of a module,
        which provides that functionality, and which is why the properties above
        can guide the discussion.
    *   A software module can implement one or more conceptual software
        components by implementing one or more of the specified interfaces.

### Tolerance

The freedom or allowable error in a geometric constraint, also known as
[margin](#margin).

### Tool Center Point (TCP)

A frame that is defined relative to a [robot's](#robot) [flange](#flange) based
on the useful work that an [end-effector](#end-effector) can perform. There can
be multiple per [end-effector](#end-effector).

### Torque control

A method of [robot](#robot) [joint](#joint) control where the output torque on
each [joint](#joint) is directly regulated, typically with the actual torque
being sensed through an integrated [joint torque sensor](#joint-torque-sensor).
Torque controlled [robots](#robot) are capable of precisely sensing external
forces such as contact at any part of the [robot's](#robot) body. They are also
capable of precisely regulating the applied force at the
[end effector](#end-effector). See also
[position control](#position-control) and
[position-controlled robot](#position-controlled-robot).

### Trajectory

A trajectory is a [path](#path) that also includes velocities, accelerations,
and jerks along the path (information about time).

### Trajectory generation

*   Input: [path](#path)
*   Output: [trajectory](#trajectory)
*   Enrich a path with information such as accelerations or velocities that make
    it feasible for execution by a [controller](#controller).

### Waypoint

A representation of a point with regards to a coordinate frame (e.g., in
Cartesian or joint space) along a [path](#path) or a point along a
[trajectory](#trajectory) that also includes velocity/acceleration/jerk.

### World model

Synonymous with [digital twin](intrinsic_terms.md#digital-twin). See
[World concepts](../platform_introduction/world_concepts.md).
