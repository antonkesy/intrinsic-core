# FANUC CRX-10iA

The visual and collision mesh models and kinematic definitions were taken from the open-source [fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description) repository:

- **Package**: `fanuc_crx_description`
- **Model Macro**: `fanuc_crx_description/urdf/crx10ia_urdf_macro.xacro`
- **Visual Meshes**: `fanuc_crx_description/meshes/crx10ia/visual/` (`.dae`)
- **Collision Meshes**: Generated from the visual `.dae` meshes using alpha-wrap (with alpha 0.01, offset 0.005 and 300 faces for links 1–3, offset 0.002 and 200 faces for base, and offset 0.002 and 400 faces for links 4–6) to produce watertight `.stl` collision envelopes preventing self-collisions.

The `crx_10ia_base.xacro` file is derived from the upstream URDF file (`fanuc_crx_description/urdf/crx10ia_urdf_macro.xacro`).
