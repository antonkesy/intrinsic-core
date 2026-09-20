# FANUC LR Mate 200iD

The visual and collision mesh models and kinematic definitions were taken from the open-source [fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description) repository:

- **Package**: `fanuc_lrmate_description`
- **Model Macro**: `fanuc_lrmate_description/urdf/lrmate200id_urdf_macro.xacro`
- **Visual Meshes**: `fanuc_lrmate_description/meshes/lrmate200id/visual/` (`.dae`)
- **Collision Meshes**: Generated from the visual `.dae` meshes using alpha-wrap (with alpha 0.01, offset 0.005 and 400 faces for links 1–3, offset 0.002 and 300 faces for base, and offset 0.002 and 600 faces for links 4–6) to produce watertight `.stl` collision envelopes preventing self-collisions.

The `lr_mate_200id_base.xacro` file is derived from the upstream URDF file (`fanuc_lrmate_description/urdf/lrmate200id_urdf_macro.xacro`).
