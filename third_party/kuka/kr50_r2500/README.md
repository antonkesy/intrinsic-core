# KUKA KR 50 R2500 (Iontec)

The visual and collision mesh models and kinematic definitions were taken from the open-source [kuka_robot_descriptions](https://github.com/kroshu/kuka_robot_descriptions) repository:

- **Package**: `kuka_iontec_support`
- **Model Macro**: `urdf/kr50_r2500_macro.xacro`
- **Visual Meshes**: `meshes/kr50_r2500/visual/` (`.dae`)
- **Collision Meshes**: `meshes/kr50_r2500/collision/` (`.stl`)

The upstream kinematic definition (urdf/kr50_r2500_macro.xacro) is included in this folder for provenance documentation. It is not used directly at runtime. Intrinsic's own xacro definitions were developed using this file as a reference for geometric alignment.
