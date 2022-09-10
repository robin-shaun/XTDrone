# Actor Collisions Plugin

This plugin example demonstates how to enable collisions for an
animated actor and scale them.

Key points:

* The plugin must call `Link::Init` on each of the actor's links so collisions
  are enabled on the physics engine

* When using ODE as the physics engine, it is necessary to set
  `<allow_auto_disable>` to false on models which should collide with the actor.
  This does not mean that setting it to true will prevent collisions.

* The collision names used on the SDF file take the following format:

    <node_name>_<node_name>_collision

  Where the nodes are described on the skeleton's COLLADA file.

## Build plugin

From this directory

    mkdir build
    cd build
    cmake ..
    make

## Run example world

An example world is provided where an actor is walking in circles and there are
objects on the way for it to collide with. Run it as follows:

    cd build
    gazebo ../actor_collisions.world

## Generate new worlds

The example world was generated from a template that randomizes various shapes
parameters. You can generate new ones from this directory as follows:

    erb actor_collisions.world.erb > actor_collisions.world