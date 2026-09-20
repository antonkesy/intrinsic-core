# Convert 3D models into SDF

This guide explains how to convert a 3D model from CAD software (e.g.,
SolidWorks, Fusion 360, Inventor) into an SDF (Simulation Description Format)
file, ready to be turned into a scene object asset. This is necessary because
robots or equipment with movable components are imported from SDF files.

## What is an SDF file?

SDF is an XML file format that describes all aspects of a simulation, including
robots, lights, sensors, and the environment. Think of it as a blueprint for a
virtual world. It's a core component of the Gazebo robotics simulator, which is
the primary simulator used by Intrinsic Core.

Here's how you'd typically author an SDF file:

*   **Use a plain text editor:** As SDF files are XML, any text editor (e.g., VS
    Code, Sublime Text, Notepad) can be used. Plugins for many editors offer XML
    syntax highlighting.
*   **Start with SDF tags:** Every SDF file begins and ends with `<sdf>` tags,
    where you specify the SDF format version.
*   **Define a model:** Inside the `<sdf>` tags, define a `<model>` to represent
    your robot, furniture, or any simulation object.
*   **Add links and joints:** A robot model consists of links (rigid parts) and
    joints (connecting links and enabling movement). Define each of these.
*   **Define visual and collision properties:** For each link, specify a
    `<visual>` tag (appearance) and a `<collision>` tag (physical shape for
    interactions).
*   **Add sensors and plugins:** Incorporate sensors (e.g., cameras, IMUs) and
    plugins to control your model's behavior.

Here is a very basic example of an SDF file for a simple cart:

```xml
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='simple_cart'>
    <link name='chassis'>
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.04</ixx>
          <iyy>0.2</iyy>
          <izz>0.2</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.4 0.3 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.4 0.3 0.1</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

### Key points for the template

* **pose:** This essential tag defines the offset from the link's origin to its
  center of mass. Input the **center of mass** coordinates directly from your
  CAD software.
* **uri:** This specifies the file path to your mesh relative to the `.sdf`
  file. If the mesh file is in the same folder as the `.sdf` file, you can use
  just its name.
* **visual vs. collision:** For simple parts, the same mesh can be used for
  both. For complex parts, use a high-detail mesh for the visual representation
  and a simpler, performance-friendly mesh for collision geometry.

For more details on SDF, refer to the
[SDFormat specification](http://sdformat.org/spec?ver=1.12&elem=sdf).

## Simplify your CAD model

Follow these steps to optimize your CAD model, ensuring it's efficient and ready
for use in real-time applications like simulators or renderers.

### Step 1: Assign and consolidate materials

Before you export, it's crucial to organize your model's materials. The goal is
efficiency, not perfect physical accuracy.

* **Assign materials:** Make sure every part of your model has a material
  assigned to it.
* **Consolidate materials:** This is the most important part. All parts that
  look the same should share the **exact same material**. For instance:
    * If you have ten bare aluminum parts, they should all use a single *Bare
      Aluminum* material.
    * If you have five anodized green parts, they should all use a single
      *Anodized Green* material.

Avoid creating copies of materials (e.g., *Aluminum-1*, *Aluminum-2*). Using a
single material for identical surfaces significantly improves performance in the
final application.

### Step 2: Optimize your model's geometry

To ensure your model runs smoothly, you need to remove any unnecessary geometry.
A lower polygon count leads to smaller file sizes and better performance.

* **Remove hidden parts:** Delete all internal components that are never visible
  from the outside, as they are not needed and impact performance.
* **Remove insignificant details:** Eliminate small parts that don't add
  significant visual value (e.g., **bolts, screws, internal threads, rivets**).
  This leads to faster uploads and more optimal calculations.

### Step 3: Convert your model to a mesh

Your solid CAD model needs to be converted into a surface mesh made of polygons
(triangles). This process is often called **tessellation**.

* **Manual conversion:** It's best to manually convert your model to a mesh
  within your CAD software. This gives you control over the final quality and
  polygon count.
* **Adjust settings by part:** For large assemblies, don't use a single
  conversion setting for everything. A large, simple panel doesn't need the same
  mesh density as a small, complex part. Convert different parts with different
  settings to get the best results.
* **Follow polycount guidelines:** *Polycount* refers to the number of triangles
  (tris) in your mesh. Aim for these general targets to maintain good
  performance:
    * **Small parts** (fittings, adapter plates): **< 10,000 tris**
    * **Medium parts** (sensors, grippers): **< 25,000 tris**
    * **Large parts** (robot arms, major components): **< 100,000 tris**
    * **Very large parts** (enclosures, machine frames): **< 200,000 tris**

### Step 4: Export the final mesh

Once your model is optimized and converted, you're ready to export it. The file
format you choose matters.

* **Export as a mesh:** Use your CAD software's `Export` or `Save As` function
  and choose a mesh file format.
* **Choose the best format:** Select the most ideal format that your CAD
  software supports, following this order of preference:
    1. **GLB (.glb):** **(Preferred format)** This format is effective because
       it can bundle everything—the mesh, materials, and textures—into a single
       file.
    2. **FBX (.fbx):** A very common and robust format in the 3D industry that
       handles mesh and material data well.
    3. **OBJ (.obj):** A widely supported and reliable format, though it may
       handle materials less gracefully than GLB or FBX.
    4. **STL (.stl):** **(Not recommended)** Use this only as a last resort. It
       typically contains only raw geometry data and no material or color
       information.

At this point, if your object does not have any kinematics, you can convert the
mesh directly into a scene object, see
[Turn the SDF into a scene object asset](#turn-the-sdf-into-a-scene-object-asset).
If you want to further refine parts, we recommend following the mesh refinement
instructions in Blender.

If you are satisfied with the mesh refinement and want to create a model with
kinematics, you can author the SDF by hand, see
[Manual SDF file creation](#manual-sdf-file-creation).

If you would like to use the Blender plugin to automatically author the SDF,
follow the steps below.

---

## Use Blender for mesh optimization and SDF generation

This section explains how to use the custom SDF_Gen plugin to author the SDF and
perform additional mesh simplification steps if needed.

### Part 1 - Initial setup & model preparation

Install the required addon and prepare your mesh geometry for the SDF generation
process by following these steps:

#### Step 1: Add the plugin to Blender

The following steps show how to activate the SDF format add-on inside Blender.

1. **Download the repository:** First, you need to download the plugin and add
   it to Blender. The repository can be found at
   [cole-bsmr/SDF_Gen](https://github.com/cole-bsmr/SDF_Gen).
2. In Blender, **open Preferences**: In the top menu bar, click `Edit` >
   `Preferences`.
3. **Go to Add-ons**: In the new window, select the `Add-ons` section from the
   left-hand menu.
4. Click on **Install from Disk** and search for the file to upload it.
5. **Enable the add-on**: The **Import-Export: SDF format** add-on will appear.
   Click the checkbox next to it to enable it.
6. **Close Preferences**: The change is saved automatically. You can now close
   the Preferences window.

![Installing the SDF_Gen plugin in Blender](../../img/guides/create_new_assets/image6.gif)

#### Step 2: Install the STEPper addon

This addon is required to properly process CAD files.

* After installing SDF_Gen, a link to purchase and install the
  [STEPper addon](https://ambient.gumroad.com/l/stepper) will appear in the
  Utilities tab if it is not already installed.
* Do not share the STEPper addon files. Each user must purchase their own
  license. Installation follows the same process as the SDF_Gen addon.

#### Step 3: Clean your mesh

This action prepares the model's geometry for the addon by applying transforms
and separating object data.

* Select all the objects you want to clean (shortcut: "a" for all).
* In the Utilities tab, press the **Clean Mesh** button. You will notice the
  object hierarchy in the outliner is removed.
* Pro-tip: If you need the original hierarchy to help organize your links,
  select only the objects for a single link and press **Clean Mesh**. Repeat
  this process for each group of objects that will form a link.

#### Step 4: Optimize your geometry

For better performance, check and reduce your model's polygon count.

* **Examine polygon count:** In the Overlay menu at the top of the viewport,
  turn on Statistics. Focus on keeping the *Triangles* count as low as possible.
  If it's too high, either simplify the model in your CAD software and
  re-export, or use the decimation tools.

  ![Polygon count statistics in the Blender viewport](../../img/guides/create_new_assets/image1.png)

* **Decimate small parts**: Select an object to optimize. In the SDF_Gen addon
  panel, find the Visual Properties section and select Decimate Mesh. Use the
  slider to lower the polygon count. If the mesh breaks or looks distorted, try
  using the Merge Vertices or Smooth Mesh buttons to fix visual artifacts.

### Part 2 - Building the SDF model

This section covers the core workflow of defining the links, colliders, and
joints for your model.

#### Step 1: Create links

Links are the individual rigid bodies of your model.

*   Navigate to the Links tab in the SDF_Gen addon.

    ![SDF_Gen Links tab](../../img/guides/create_new_assets/image9.png)

*   In the **Models:** window, rename *Scene* to your desired model name. This
    will also be the name of the export folder.

    ![SDF_Gen Models window](../../img/guides/create_new_assets/image4.png)

*   Press the **Create Link** button and enter a name. Note that *link* will be
    automatically added to the end.
*   In the Outliner, drag the mesh objects that belong to this link into the
    link's *visual* collection.

    ![Dragging meshes into a link's visual collection](../../img/guides/create_new_assets/image5.gif)

*   When a link is selected, you can check the Static box in its properties to
    control whether it will be static or dynamic in the final SDF file.
*   All links will be displayed in the **Links List** section.

    ![SDF_Gen links list](../../img/guides/create_new_assets/image8.png)

#### Step 2: Create colliders

Colliders define the physical shape of a link for simulation. Primitive
colliders are highly recommended over mesh colliders for simulation and
collision-free motion planning.

*   **Create primitive colliders:**
    *   Select one or more visual objects and press the Box, Cylinder, Sphere,
        Cone, or Plane button to generate a corresponding collider.
    *   Use the **Last Operation Panel** in the lower-left of the viewport to
        adjust the fit before clicking elsewhere.
    *   See how these settings affect the creation of colliders in the
        [SDF_Gen operation panel documentation](https://github.com/cole-bsmr/SDF_Gen?tab=readme-ov-file#operation-panel).
*   **Create mesh colliders:**
    *   If absolutely necessary, use the Mesh Collider button. Be sure to use
        the available tools to reduce the mesh resolution as much as possible to
        maintain performance.
    *   See more about
        [creating mesh colliders](https://github.com/cole-bsmr/SDF_Gen?tab=readme-ov-file#mesh-collider).
        Specifically note the tools to reduce the resolution of the mesh
        collider as this will have a large effect on performance.
*   **Transform colliders:**
    *   Use the 'Scale Cage' tool combined with the 'Face Snap' tool to quickly
        and precisely adjust the size and fit of primitive colliders to the
        underlying geometry.
    *   You can also use the Global Margin setting to adjust the distance
        between all colliders and the geometry.

      ![Adjusting colliders in Blender](../../img/guides/create_new_assets/image3.gif)

#### Step 3: Create joints

Joints define how links connect and move relative to each other.

*   Switch to the **Joints** tab and press **Create Armature** to prepare the
    scene.
*   Press the **Create Joint** button, then choose your joint type and the child
    link. The child link cannot be changed later, so you will need to delete and
    recreate the joint if you make a mistake.
*   Always use the **Delete Joint** button in the SDF_Gen menu to remove joints.
*   Press the **Adjust Joints** button to move the joint to its correct location
    without affecting the model's geometry.
    *   Use either the move/rotate gizmo, or the transform panel under the
        **Adjust Joints** button.

    ![Creating and adjusting joints](../../img/guides/create_new_assets/image11.gif)

*   In the **Joint Properties** panel, set the parent link and any applicable
    joint limits. For a revolute joint without limits, check the **Continuous**
    box.

#### Step 4: Test your joints

Verify that your joints and links move as expected before exporting.

*   At the top of the viewport, set the transform space to Local.

    ![Setting the transform space to Local](../../img/guides/create_new_assets/image10.png)

*   Use the standard move and rotate tools to test the joint's movement and
    limits.
*   When you are finished testing, press the **Reset Joints** button to return
    all links to their original positions.

### Part 3 - Exporting

Follow these final steps to export your model from Blender.

Configure the export settings to generate the final files:

*   Ensure the file format is set to GLB.
*   The default export path is a folder named `//sdf_exports/` within the same
    directory as your saved `.blend` file. Remember to save your Blender file
    first.
*   Expand the SDF Options menu and check the box for Relative link poses.
*   If you need a config file, check the `Export Config` File box and fill out
    the required fields.
*   Select any object in your scene and press the Export SDF button.
*   After exporting you should have a folder that looks like this:

    ![Contents of the SDF export folder](../../img/guides/create_new_assets/image2.png)

*   If you have any mesh colliders those will be exported as STL files.

---

## Manual SDF file creation

If you prefer to skip Blender and are comfortable hand-authoring the SDF for the
scene or kinematics, follow the steps below.

You will create the `.sdf` file itself using the assets you just exported from
your CAD tool. An SDF file is a simple text file written in XML format. See
[What is an SDF file?](#what-is-an-sdf-file) for more details.

### Step 1: Create the SDF file

1. Open a plain text editor like **VS Code (recommended)**, Sublime Text, or
   Notepad.
2. Create a new file and immediately save it in the **same folder** as your
   exported mesh file (e.g., `model.sdf`).

### Step 2: Write the SDF code

Copy the template below and paste it into your new file. Then, carefully replace
the placeholder values with the values you copied from your CAD software.

```xml
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='my_cad_model'>
    <link name='base_link'>

      <pose>0.0 0.0 0.05 0 0 0</pose>

      <inertial>
        <mass>1.2</mass>
        <inertia>
          <ixx>0.005</ixx>
          <iyy>0.005</iyy>
          <izz>0.001</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>

      <visual name='visual'>
        <geometry>
          <mesh>
            <uri>model.dae</uri>
          </mesh>
        </geometry>
      </visual>

      <collision name='collision'>
        <geometry>
          <mesh>
            <uri>model.dae</uri>
          </mesh>
        </geometry>
      </collision>

    </link>
  </model>
</sdf>
```

> [!IMPORTANT]
> Specify the `<mass>` of every link. Simulation and motion planning behave
> implausibly when masses and inertias are left at their defaults.

Once you have filled in all the placeholders and saved the file, you have a
complete, simulation-ready SDF model. Your project folder should now contain
both `model.dae` (the mesh) and `model.sdf` (the description).

### Step 3 (optional): Add frames to the SDF file

The SDF import pipeline supports
[custom attributes](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal)
to include information beyond the standard SDFormat specification. By default,
the pipeline disregards
[implicit frames](http://sdformat.org/spec?ver=1.12&elem=model#model_frame)
defined within the SDF, so frames that you want to use must be declared
explicitly with one of the following attributes.

Use `intrinsic:create_entity` to create a
[frame](../../learn/platform_introduction/world_concepts.md#frames):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--
  Note: xmlns:intrinsic="https://intrinsic.ai/" is required for the custom
  attribute used in the <frame> tag below.
-->
<sdf version="1.11" xmlns:intrinsic="https://intrinsic.ai/">

  <model name="some_model_name">

  <!-- Provide model description like links here ... -->

  <!-- Create an explicit frame within a model -->
  <frame name="tool_frame"
         attached_to="base_link"
         intrinsic:create_entity="true">
      <pose>0 0 0 0 3.141592654 0</pose>
  </frame>

  </model>
</sdf>
```

Use `intrinsic:create_attachment_entity` to create an
[attachment frame](../../learn/platform_introduction/world_concepts.md#attachment-frames):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--
  Note: xmlns:intrinsic="https://intrinsic.ai/" is required for the custom
  attribute used in the <frame> tag below.
-->
<sdf version="1.11" xmlns:intrinsic="https://intrinsic.ai/">

  <model name="some_model_name">

  <!-- Provide model description like links here ... -->

  <!-- Create an explicit attachment frame within a model -->
  <frame name="flange"
         attached_to="A6_link"
         intrinsic:create_attachment_entity="true">
  </frame>

  </model>
</sdf>
```

---

## Turn the SDF into a scene object asset

In Intrinsic Core, an SDF file is converted into a scene object asset at build
time. Place the `.sdf` file and its mesh files in a package of your workspace
and add the following to that package's `BUILD` file:

```python
load("@intrinsic-core//intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")

filegroup(
    name = "my_model_meshes",
    srcs = glob(["meshes/**"]),
)

# Converts the SDF (and the meshes it references) into the geometric
# description of the asset.
sdf_scene_object(
    name = "my_model_scene_object",
    src = "my_model.sdf",
    sdf_assets = [":my_model_meshes"],
)

# Packages the geometry and the metadata into an installable asset bundle.
intrinsic_scene_object(
    name = "my_model",
    manifest = "my_model_manifest.textproto",
    scene_object = ":my_model_scene_object",
)
```

The manifest holds the asset metadata:

```textproto
# proto-file: intrinsic/assets/scene_objects/proto/scene_object_manifest.proto
# proto-message: intrinsic_proto.scene_objects.SceneObjectManifest

metadata {
  id {
    package: "com.my_company"
    name: "my_model"
  }
  vendor {
    display_name: "My Company"
  }
  display_name: "My model"
  documentation {
    description: "A short description of the object."
  }
}
```

Build the asset, install it on a running solution, create an instance of it,
and reset the world so that the new object appears in it:

```bash
bazel build //path/to:my_model
inctl asset install bazel-bin/path/to/my_model.bundle.tar --address localhost:17080
inctl service add com.my_company.my_model --name=my_model --address localhost:17080
inctl world reset --address localhost:17080
```

Alternatively, reference the target from the `assets` attribute of your
`intrinsic_solution` target, and an `intrinsic_asset_instance` of it from the
`instances` attribute, so that the object is installed and instantiated
whenever the solution is deployed.

All geometry is interpreted in meters, so make sure the model is exported in
meters, or scale it in the SDF.
