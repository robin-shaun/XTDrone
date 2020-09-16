# About

Gazebo Ros plugin to steer an actor with twist messages via ros.
Based on actor_plugin from https://bitbucket.org/osrf/gazebo/raw/default/plugins/ActorPlugin.cc

# Usage

Example actor tag in a .world file:
```
<actor name="actor1">
      <pose>0 1 1.25 0 0 0</pose>
      <skin>
        <filename>moonwalk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>walk.dae</filename>
        <scale>1.000000</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <plugin name="actor1_plugin" filename="libros_actor_cmd_pose_plugin.so">
        <animation_factor>5.1</animation_factor>
        <init_pose>2 3 1.1 1.57 0 0 <init_pose>
      </plugin>
    </actor>
```

The actor will listen for point messages on the ros topic `/actor1/cmd_pose`.

