# Universal Robots (UR) Mesh Models

The visual (`.dae`) and base collision (`.stl`) mesh models in `ur3e/`, `ur5e/`, `ur10e/`, and `ur16e/` were taken from the open-source [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) repository (`ur_description/meshes/`), licensed under the BSD 3-Clause License (`Copyright 2019-2024 Universal Robots A/S`).

## Modifications (`Portions Copyright 2026 Intrinsic Innovation LLC`)

- **Wrapped Collision Meshes (`*.wrapped.1cm.stl` and `upperarm_simplified.stl`)**: Generated from the upstream collision `.stl` meshes by applying a 1 cm offset mesh-wrapping envelope (`Portions Copyright 2026 Intrinsic Innovation LLC`) to produce simplified, watertight collision geometries suitable for motion planning and collision checking.
