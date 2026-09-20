<?xml version="1.0"?>
<!-- A world template must specify the SDFormat spec version used in the template. -->
<sdf version="1.6">
  <!-- A world template must contain exactly one <world> element. -->
  <world name="template">
    <!-- Gazebo 1p plugins -->
    <!-- The `enforce_fixed_constraint` option is enabled to improve gripper
          reliability. It is only used if `bullet-featherstone` physics engine
          is used. The physics engine used is specified as a cmd-line flag to
          the Gazebo simulation server and not included here. -->
    <plugin filename="static://gz::sim::systems::Physics"
      name="gz::sim::systems::Physics">
      <enforce_fixed_constraint>1</enforce_fixed_constraint>
    </plugin>

    <plugin filename="static://gz::sim::systems::SceneBroadcaster"
      name="gz::sim::systems::SceneBroadcaster" />

    <plugin filename="static://gz::sim::systems::Sensors" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
      <background_color>0.5 0.5 0.5</background_color>
      <ambient_light>0.5 0.5 0.5</ambient_light>
    </plugin>

    <plugin filename="static://gz::sim::systems::ForceTorque"
      name="gz::sim::systems::ForceTorque" />

    <plugin filename="static://gz::sim::systems::UserCommands"
      name="gz::sim::systems::UserCommands" />

    <!-- Flowstate 1p plugins -->
    <plugin filename="static://intrinsic::simulation::ServiceStateAggregator"
      name="intrinsic::simulation::ServiceStateAggregator" />

    <plugin filename="static://intrinsic::simulation::SimInputsSystem"
      name="intrinsic::simulation::SimInputsSystem" />

    <!-- Policies -->
    <!-- Make PostUpdate() run serially for performance improvement -->
    <gz:policies>
      <parallel_postupdates>false</parallel_postupdates>
    </gz:policies>

    <!-- Lights -->
    <!-- Default lighting consists of two directional lights with slightly
          different diffuse color.
          TODO(b/235991654): Enable cast_shadows and remove custom intensity. -->
    <light type="directional" name="light1">
      <cast_shadows>false</cast_shadows>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <intensity>2</intensity>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <light type="directional" name="light2">
      <cast_shadows>false</cast_shadows>
      <diffuse>0.4 0.4 0.4 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <intensity>1</intensity>
      <direction>0.1 -0.5 -0.9</direction>
    </light>

    <!-- Physics settings -->
    <!-- The physics `type` field is set to `ignored` below as it is a
          required field in the sdf spec, but it is not used in Gazebo.
          Instead, the physics engine is specified as a cmd-line flag to the
          gazebo simulation server. -->
    <physics type="ignored">
      <!-- Default sim step rate is 500Hz. -->
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1</real_time_factor>
      <!-- Use `bullet` collision detector with DART physics engine.
            This field will be ignored if DART is not used. -->
      <dart>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>
  </world>
</sdf>
