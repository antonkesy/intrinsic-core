# KUKA KR 20 R1810-2 (Cybertech)

The visual and collision mesh models and kinematic definitions were taken from the open-source [kuka_robot_descriptions](https://github.com/kroshu/kuka_robot_descriptions) repository. 
The models were adjusted to not include the dresspack.

- **Package**: `kuka_cybertech_support`
- **Model Macro**: `urdf/kr20_r1810_2_macro.xacro`
- **Visual Meshes**: `meshes/kr20_r1810_2/visual/` (`.dae`)
- **Collision Meshes**: `meshes/kr20_r1810_2/collision/` (`.stl`)

The upstream kinematic definition (urdf/kr20_r1810_2_macro.xacro) is included in this folder for provenance documentation. It is not used directly at runtime. Intrinsic's own xacro definitions were developed using this file as a reference for geometric alignment.
