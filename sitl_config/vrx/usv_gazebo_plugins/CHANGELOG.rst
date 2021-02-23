^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package usv_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2019-12-26)
------------------
* Make code_check happy.
* Mod to make use of maxCmd and update to .hgignore
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero

1.2.6 (2019-10-04)
------------------

1.2.5 (2019-09-19)
------------------
* merging default into branch
* Changes for code checker
* Contributors: Brian Bingham <briansbingham@gmail.com>

1.2.4 (2019-09-12)
------------------

1.2.3 (2019-09-12)
------------------
* Minor maintenance updates.
* Fix style error.
* Adding a default value for the length_n plugin parameter
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>

1.2.2 (2019-09-06)
------------------

1.2.1 (2019-09-05)
------------------
* updated comments
* minor cleanup + env flag + disable z
* Namespace tweaks.
* Style changes.
* gazebo 7 bug fix
* removed from wamv
* gazebo 7 compatibility
* force vectors are correct; scaling added
* Contributors: Carlos Aguero, Rumman Waqar <rumman.waqar05@gmail.com>

1.2.0 (2019-08-19)
------------------
* Deterministic wind.
* Add v3d plugin - this publishes a vecotr based on the world frame velocity in Gazebo
  Update gps configuration to add gazebo gps and v3d plugins to standard configuration
* Add plugin for ROS interface to gazebo GPS sensor.
* added cylinder placeholder
* incremental
* added plate and sphere models
* functional for cubes
* added force visual plugin
* Contributors: Carlos Aguero, Jonathan Wheare <jonathan.wheare@flinders.edu.au>, MarshallRawson, Rumman Waqar <rumman.waqar05@gmail.com>

1.1.2 (2019-07-10)
------------------
* usv_gazebo_wind_plugin.hh changes
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Agüero <cen.aguero@gmail.com>, Rumman Waqar <rumman.waqar05@gmail.com>

1.1.1 (2019-07-03)
------------------
* Reinterpret the wind 'gain' parameter.  Set defaults to zero
* updated style for buoyancy plugin
* Contributors: Brian Bingham <briansbingham@gmail.com>, Rumman Waqar <rumman.waqar05@gmail.com>

1.1.0 (2019-07-01)
------------------
* Refactor SpinPropeller to be more clear and match style of RotateEngine
* Publish joint state for chassis engine joint to fix tf tree issue
* addressed Brian's comments
* Connecting wave model to buoyancy plugin
* buoyancy now uses wave height
* capitalized brief sentences
* gazebo 7 vector3[] operator lhv error fixed try 2
* gazebo 7 vector3[] operator lhv error fixed
* gazebo <= 8 fixes for Mass and AngularVel
* complex objects + xacro cleanup
* force applied in correct place
* centroid + volume complete
* moved shape volume into its own file and renamed classes
* tiny refactor
* polyhedron class finished and tests passed
* implemented polyhedron based cube + cylinder submerged volume and cov
* rotation bug fix
* basic volume for box, sphere and cylinder
* removed old volume data structure + created skeleton for volume calc
* removed old volume data structure + created skeleton for volume calc
* added move constructor for buoyancy object and remove copy constructor
* added links and buoyancy object to plugin
* minor cleanup
* added pose for buoyancy element
* parsed buoyancy shape now saved as unique_ptr
* cleaned up parsing
* updated buoyancy xacro + parsing
* Setup buoyancy test build system + test world
* merging with default - need to check wind
* added wind capabilities
* Removing gazebo::msg::Param references and cleaning up for gazebo version < 8 compatibility.
* Removed gazebo messaging, introduces redundancy in model.sdf for ocean. USV and buoyancy plugins only get wave parameters once instead of every update.
* Move link error message.
* Another Nitpick fix (== nullptr) => !
* Nitpick fix (== nullptr) => !
* Clarify required and optional parameters, and remove unused confusing default declarations
* Put required parameters together and make it obvious which are required
* Retune PID for engineJoint with lower P gain, for more realistic behavior
* Add <enableAngle> bool parameter that controls if angle is adjustable or not
* C++ Code style fixes
* Add documentation about maxAngle and angleTopic
* updated documentation
* Attempt to fix build issue with .GetAngle().Radian()
* Attempt to fix build issue with Position() -> GetAngle() for old gazebo version
* Implement PID controller for engine joint to set joint angle
* changed sdf sytax for passing models to be effected by wind and addressed styling
* Attempt to fix build issues SetAngle->SetPosition
* Attempt to fix build issue with different setAngle setPosition implementation based on Gazebo version
* Implement turnable thruster joint
* Basic implementation of angle adjustable thrusters, still need to test, add joints, and change visuals
* merging with default
* fix build issue for gz <8
* merged. expanded xacro capabilities
* Rewrite implementation of setting windDirection
* documenting wind direction
* changing the interface from timePeriod to frequency
* cleaning up the includes order and white spaces
* cleanup
* adding ROS API to probe for wind speed
* enabling the user to input only the angle for wind direction
* increment
* documented
* incremental(basic testing passed)
* changed wind plugin(untested
* Initial testing of random seed with print statements
* Added wavegauge plugin to visualize physical wave height.  Setup example with buoy world.  Implemented simplified wave height calculation in WavefieldSampler for regularly spaced grid (steepness=1=0).
* verifying with examples
* toward buoy examples
* merging default into named branch
* removed currentVarVel from member variable list and fixed indentation for directives
* made gzmsg more efficient
* Implemented changed after PR is reviewed - V1
  Remove Ros dependency (regarding time)
  fixed typoes
  fixed wrong comments
  Exposed seed value to user
  Updated purpose of SDF params in the header file
  lines are now shorted than 80 chars
  added comments around explaining the calculations done
* made wind speed randomized
* merging default to update the feature branch
* Remove more trailing whitespace
  Redundant codepath in usv_gazwebo_dynamics_plugin removed.  Euler values now derived identically between gazebo 7 and 9.
* Fix trailing whitespace
* Fix line breaks
* Alter patch to use .Ign method to convert between gazebo::math and Ignition::math types
* Add support for Kinetic/Gazebo-7
  The ignition types are mostly kept, with code transforming from the methods deprecated in gazebo-8
* Changing license text
* Adding two packages from asv_wave_sim as a part of VRC
* Issue #23: Coordinate the physics and visualization of the wave field
  1. Use the asv_wave_sim_gazebo_plugins package for wave field visualisation and depth calculation.
  2. Update the buoyancy and dynamics plugins for buoyancy calculations.
  3. Update sdf and xacro for models that require buoyancy.
  4. Replace the ocean model with ocean_waves in the sandisland world.
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, Jonathan Wheare <jonathan.wheare@flinders.edu.au>, MarshallRawson, Rhys Mainwaring <rhys.mainwaring@me.com>, Rumman Waqar <rumman.waqar05@gmail.com>, Tyler Lum <tylergwlum@gmail.com>, Youssef Khaky <youssefkhaky@hotmail.com>, YoussefKhaky <youssefkhaky@hotmail.com.com>

1.0.1 (2019-03-01)
------------------

1.0.0 (2019-02-28)
------------------
* Porting to Gazebo 9
* Rename vmrc to vrx.
* More progress.
* Changed from buoyancy calculation method
* Fixing error where buoyancy force could be applied in the negative direction (downward)
* Add dependency on usv_msgs by usv_gazebo_pinger_plugin.  This forces the message to be built before the plugin is compiled.
* Set the sensor WAM-V as the default model
* Fix the doxygen generation
* Update variable names and comments to be compliant with the Gazebo style guide.
* Add the pinger plugin to the wamv_gazebo package.
  The wamv_gazebo_sensors.urdf file has been modified to add support for the pinger plugin.
* removing static tags so vessel is freee to move
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>, Jonathan Wheare <jonathan.wheare@flinders.edu.au>

0.3.2 (2018-10-08)
------------------
* Include jrivero as maintainer of the ROS packages
* Include headers in the installation of usv_gazebo_plugins
* Contributors: Jose Luis Rivero <jrivero@osrfoundation.org>

0.3.1 (2018-10-05)
------------------
* Decleare eigen as dependency for usv_gazebo_plugins
* modifying grid spacing
* Contributors: Brian Bingham <briansbingham@gmail.com>, Jose Luis Rivero <jrivero@osrfoundation.org>

0.3.0 (2018-09-28)
------------------
* vrx metapackage and spring cleaning.
* adding publication of forces/moments
* trying to get wamv to be static using a fixed joint
* Adding publication from dynamics plugin for wave height at USV CG for Josh's thesis work
* Tweak
* Changelog and minor tweaks.
* Remove extra dependency.
* Merged in generalize-thruster-desc (pull request #34)
  Generalize thruster desc
  Approved-by: Brian Bingham <briansbingham@gmail.com>
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* merging changes from PR branch into development branch
* resolving merge conflict
* Adding bits to repond to PR comments
* adding examples for T and X thruster configurations - accessible as args to sandisland.launch. Prototype - too much redundancy in the various urdf.xacro file hierarchy, but functional.
* Tweaks.
* Tabs -> spaces
* Initial style pass
* props now spinning, removed old method of thrust implementation, removed custome UsvDrive message
* working prototype - next remove old method
* prior to splitting thruster into its own header
* increment - builds, but need to go home
* catching up with default
* increment, pushing to work from home
* first steps towards new structure
* Drop log level to DEBUG for imformation unimportant to user
* Minor style changes in the gazebo_ros_color plugin.
* Tweak
* Move log message to DEBUG.
* adding a bit more doxygen, including link to Theory of Operation document
* Tweaks.
* adding doxygen comments
* Doxygen and cleaning up
* Rename buoyLinks to buoyancyLinks and remove debug output.
* More style.
* More tweaks.
* Initial style changes.
* Merge from default.
* Apply Gazebo style.
* Move some ROS_INFO messages to ROS_DEBUG and remove ros::init().
* More tweaks.
* Tweaks
* Tweaks
* Initial work
* Publish joint_states from thrust plugin
* Tweak
* Refactor wind plugin.
* Split the wamv xacro file.
* Generate messages before building the Thrust plugin.
* More modular model with spinning propellers.
* Merge from default
* Add message_generation.
* Backed out changeset 8023d94fc0e1
* Add light buoy challenge
* Remove unsused buoyancy plugin (already in gazebo)
* Boostrap usv_gazebo_plugins
* Move gazebo plugins to usv_gazebo_plugins
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <caguero@osrfoundation.org>, Kevin Allen <kallen@osrfoundation.org>
