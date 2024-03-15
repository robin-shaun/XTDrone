# How to Contribute

We welcome contributions to this repo and encourage you to fork the project, thanks!

## Implementations of simulations

It is highly recommended to place the code that performs simulation inside
[upstream gazebo code](https://bitbucket.org/osrf/gazebo/). The Gazebo project
provides documentation about [how to create and code
plugins](http://gazebosim.org/tutorials?cat=write_plugin) Ideally
gazebo_ros_pkgs should implement the ROS wrapper over an existing gazebo
plugin.

## Issues and Pull Requests

There are several maintainers of different packages in this repository. If you
are submitting an issue or a pull request, please prefix the title of the issue
or pull resquest with the package name. This way the appropriate maintainers
can more easily recognize contributions which require their attention.

## Style

We follow the [C++ ROS style guidelines](http://ros.org/wiki/CppStyleGuide) and
conventions as closely as possible. However, because the plugins inherit from Gazebo
classes and Gazebo follows a very different formatting standard, there are a few
exceptions where Gazebo's function names do not comply to the ROS guidelines.

## Tests

We encourage developers to include tests in their pull requests when possible,
both for bug fixes and new features.
Examples of tests are in the `gazebo_plugins/test*` folders.
Currently the tests must not be run in parallel (see issue
[issue 409](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/409))
so it is recommented to use the `-j1` argument to `catkin run_tests`.
