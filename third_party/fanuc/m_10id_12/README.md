# FANUC M-10iD/12

The visual and collision mesh models and kinematic definitions were taken from the open-source [fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description) repository:

- **Package**: `fanuc_m10_description`
- **Model Macro**: `fanuc_m10_description/urdf/m10_12_14d_urdf_macro.xacro`
- **Visual Meshes**: `fanuc_m10_description/meshes/m10_12_14d/visual/` (`.dae`)
- **Collision Meshes**: Generated from the visual `.dae` meshes using alpha-wrap (with alpha 0.01, offset 0.005 and 400 faces for links 1–3, offset 0.002 and 300 faces for base, and offset 0.002 and 600 faces for links 4–6) to produce watertight `.stl` collision envelopes preventing self-collisions.

The `m_10id_12_base.xacro` file is derived from the upstream URDF file (`fanuc_m10_description/urdf/m10_12_14d_urdf_macro.xacro`).
