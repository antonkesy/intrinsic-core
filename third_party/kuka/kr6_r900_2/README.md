# KUKA KR 6 R900-2 (Agilus)

The visual and collision mesh models and kinematic definitions were taken from the open-source [kuka_robot_descriptions](https://github.com/kroshu/kuka_robot_descriptions) repository:

- **Package**: `kuka_agilus_support`
- **Model Macro**: `urdf/kr6_r900_2_macro.xacro`
- **Visual Meshes**: `meshes/kr6_r900_sixx/visual/` (`.dae`)
- **Collision Meshes**: `meshes/kr6_r900_2/collision/` (`.stl`)

The upstream kinematic definition (urdf/kr6_r900_2_macro.xacro) is included in this folder for provenance documentation. It is not used directly at runtime. Intrinsic's own xacro definitions were developed using this file as a reference for geometric alignment.
