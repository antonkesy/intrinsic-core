# FANUC M-710iD/50M

The visual and collision mesh models and kinematic definitions were taken from the open-source [fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description) repository:

- **Package**: `fanuc_m710_description`
- **Model Macro**: `fanuc_m710_description/urdf/m710id_50m_urdf_macro.xacro`
- **Visual Meshes**: `fanuc_m710_description/meshes/m710id_50m/visual/` (`.dae`)
- **Collision Meshes**: Generated from the visual `.dae` meshes using alpha-wrap (with alpha 0.01, offset 0.005 and 400 faces for links 1–3, offset 0.002 and 300 faces for base, and offset 0.002 and 600 faces for links 4–6) to produce watertight `.stl` collision envelopes preventing self-collisions.

The `m_710id_50m_base.xacro` file is derived from the upstream URDF file (`fanuc_m710_description/urdf/m710id_50m_urdf_macro.xacro`).
