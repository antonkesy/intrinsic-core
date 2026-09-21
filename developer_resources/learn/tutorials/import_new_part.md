# Import new part

Learn how to import new workpiece geometries, such as raw stock materials for simulated CNC machine tending, using text-based configurations. If you are using your own CAD files, see [Converting 3D models](../../assets/create_new_assets/convert_3d_models.md) for instructions on generating and optimizing meshes for simulation.

## Prerequisites

If you haven't already, follow [Getting started](getting_started.md) to fetch the sources for the Intrinsic [OMTS](../glossary/intrinsic_terms.md#open-machine-tending-solution-omts) [Solution](../glossary/intrinsic_terms.md#solution) and start it.

## Understanding SceneObject schemas and SDFormat

In Intrinsic Core, workpiece geometries are represented as Scene Objects. These store the geometry alongside other properties useful for [simulation](../glossary/general_terms.md#simulation) or visualization. For more details, see [Scene Object](../glossary/intrinsic_terms.md#scene-object) definition.

SceneObjects are defined via two files:

1. `my_part.sdf`: Physical properties (inertia, mass, visual and collision geometry). This file can optionally reference a geometric mesh file.
2. `manifest.textproto`: Platform metadata (ID, name, version, description). You can inspect the metadata for the running Solution with:
   ```bash
   inctl asset list --address localhost:17080 --output json | jq
   ```

## Step 1: Set up a new Bazel Asset package

We'll create a dedicated [Asset](../glossary/intrinsic_terms.md#asset) [package](../glossary/intrinsic_terms.md#package) (e.g. `@omts//models/raw_stock`) with the following directory structure:

```text
models/raw_stock_logo/
├── BUILD
├── raw_stock_logo.sdf
├── manifest.textproto
└── visual.glb
```

Start with the directories and geometry:

```bash
cd ~/intrinsic-omts
mkdir -p models/raw_stock_logo
cp ~/intrinsic-core/intrinsic/models/assets/imts/raw_stock/raw_stock_green_logo.glb models/raw_stock_logo/visual.glb
```

> [!NOTE]
> This copies over a model of a different piece of raw stock from the `intrinsic-core` repo, which we'll use for testing. Despite the resemblance, "imts" is not related to "OMTS".

## Step 2: Create the manifest file

This metadata file does not refer directly to the other files; it is linked by the BUILD file.

```bash
cd ~/intrinsic-omts
cat > models/raw_stock_logo/manifest.textproto << EOF
# proto-file: intrinsic/assets/scene_objects/proto/scene_object_manifest.proto
# proto-message: intrinsic_proto.scene_objects.SceneObjectManifest
metadata {
  id {
    package: "ai.intrinsic"
    name: "raw_stock_logo"
  }
  vendor {
    display_name: "Intrinsic"
  }
  documentation {
    description: "Machinable raw stock with a green logo."
  }
  display_name: "Raw Stock (logo)"
}
EOF
```

## Step 3: Create the SDF model (raw_stock_logo.sdf)

Define the physical properties of the object. This refers to the visual geometry file (`.glb`) that we fetched earlier.

```bash
cd ~/intrinsic-omts
cat > models/raw_stock_logo/raw_stock_logo.sdf << EOF
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="raw_stock_logo">
    <static>false</static>
    <link name="base_link">
      <!-- Parameters for physics simulation -->
      <inertial>
        <mass>0.450</mass>
        <inertia>
          <ixx>0.0003</ixx>
          <iyy>0.0005</iyy>
          <izz>0.0006</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>

      <!-- High-fidelity visual rendering. Also supports .dae -->
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>visual.glb</uri>
          </mesh>
        </geometry>
      </visual>

      <!--
      Low-poly approximation (a cube) for fast collision checking.
      This can also use <mesh> if you have a simplified 3D model.
      -->
      <collision name="cube_collider_box">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.05 0.075 0.05</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF
```

## Step 4: Define Bazel BUILD targets

Create a BUILD file for the Asset:

```bash
cd ~/intrinsic-omts
cat > models/raw_stock_logo/BUILD << EOF
load("@intrinsic-core//intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")

package(default_visibility = ["//visibility:public"])

sdf_scene_object(
    name = "raw_stock_logo_scene_object",
    src = "raw_stock_logo.sdf",
    sdf_assets = ["visual.glb"],
)

intrinsic_scene_object(
    name = "raw_stock_logo",
    manifest = "manifest.textproto",
    scene_object = ":raw_stock_logo_scene_object",
)
EOF
```

## Step 5: Build and deploy the SceneObject Asset

Compile the Asset bundle. This combines the files you created into a single `.bundle.tar` file.

```bash
cd ~/intrinsic-omts
bazel build //models/raw_stock_logo:raw_stock_logo
```

Deploy the bundle to the running Solution. This adds it to the Asset registries, but doesn't yet add it to the Object World [Service](../glossary/intrinsic_terms.md#service).

```bash
cd ~/intrinsic-omts
inctl asset install --address localhost:17080 \
  bazel-bin/models/raw_stock_logo/raw_stock_logo.bundle.tar
```

Then, you need to add an instance of the Asset. Creating instances after adding the Asset itself allows multiple instances of the same part, for example in a bin-picking application.

```bash
inctl service add ai.intrinsic.raw_stock_logo --address localhost:17080 --name raw_stock_logo
```

The new part has only been added to the initial world, but not to the execution state. Reset the execution state to add `raw_stock_logo` to the Object World Service:

```bash
inctl world reset --address localhost:17080
```

[Open RViz](visualize_the_robot.md) and see if you can spot the new part near the origin of the [scene](../glossary/intrinsic_terms.md#scene):

![Imported part near scene origin in RViz](../../img/learn/tutorials/rviz_imported_part.png)

To make it easier to find, consider using ObjectWorldUpdates to move it to a new position ([Cell Customization](cell_customization.md)).

## Step 6: Verify Asset registration

```bash
inctl asset instance list --address localhost:17080
```

Output:

> [!NOTE]
> Note that this includes both Scene Objects (such as this part) and services (such as the camera driver).

```text
Name                            Asset
[...]
raw_stock_2x3x5                 ai.intrinsic.raw_stock_2x3x5
raw_stock_logo                  ai.intrinsic.raw_stock_logo
train_service                   ai.intrinsic.ioc_train_service
```

## Next steps

For a real Solution, a few extra steps are required:

* For the part to persist even if you stop and restart the Solution, you need to edit the Solution BUILD file to add the new part to the Solution definition.
* If you have access to CAD software, you could try creating your own meshes to import. See [Convert 3D models](../../assets/create_new_assets/convert_3d_models.md) for guidance.
* For static parts like the [robot's](../glossary/general_terms.md#robot) enclosure, you'd need to use either the SDF [<static> property](https://sdformat.org/spec/1.12/model/#model_static) or ObjectWorldUpdates with [is_static: true](https://github.com/intrinsic-ai/sdk/blob/73c9f8b2182e373cc331a5ecac783ebae7f0f6ee/intrinsic/world/proto/simulation_component.proto#L30) to prevent it from falling under gravity. You can see this if you run `inctl world reset --address localhost:17080` while watching the Gazebo visualization.

The next tutorial is [Custom Asset Creation (Software)](custom_asset_creation_software.md).
