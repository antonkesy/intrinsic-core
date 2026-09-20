# Orbbec Camera Meshes

- **Upstream Repository**: [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)
- **Branch**: `v2-main`
- **License**: Apache-2.0 (see [LICENSE](LICENSE))
- **Third-Party Notices**: Sourced from upstream Orbbec SDK ROS 2 (see [NOTICE](NOTICE))

## Models

### Gemini 335Le (`gemini_335le/`)
- **Original Source Files**: `orbbec_description/meshes/gemini335Le/*.STL`
- **Kinematic Reference**: `orbbec_description/urdf/gemini_335_Le.urdf.xacro`

#### Local Modifications
1. **Assembly**: Upstream distributes the camera as 6 individual untextured STL files intended for a multi-link URDF description. Four of these components were assembled into a single consolidated `visual.glb` at their respective URDF joint origins:
   - `base_Link.STL`: Main chassis at origin `(0, 0, 0)`.
   - `canera_left_IR_ROS_frame.STL`: Left IR optics translated to `(+0.034005, +0.0475, +0.014048) m`.
   - `canera_right_IR_ROS_frame.STL`: Right IR optics translated to `(+0.034005, -0.0475, +0.014048) m` (establishing the 95 mm stereo baseline).
   - `canera_RGB_ROS_frame.STL`: RGB optics translated to `(+0.034014, +0.02375, +0.014048) m`.
2. **Omitted Hidden Geometry**: Omitted `canera_IMU_ROS_frame.STL` (internal circuit board) since it is completely enclosed inside the opaque chassis and not visible externally.
3. **Materials & Shading**: Converted from plain monochromatic STL triangles to glTF PBR materials (matte dark housing, glass optical lenses, aperture black).
4. **Collision**: Collision detection uses an analytical box primitive in `orbbec_gemini_335le.sdf` matching the physical outer dimensions.
