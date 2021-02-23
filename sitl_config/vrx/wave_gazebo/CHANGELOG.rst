^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wave_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2019-12-26)
------------------

1.2.6 (2019-10-04)
------------------

1.2.5 (2019-09-19)
------------------

1.2.4 (2019-09-12)
------------------

1.2.3 (2019-09-12)
------------------

1.2.2 (2019-09-06)
------------------

1.2.1 (2019-09-05)
------------------
* remove gazebo version check, scene blend
* Reduce noise back down
* Change to have distortion in both refl and refr
* Add rttNoise parameter to scale distortion in refraction
* Switch ratio->opacity naming
* Add back ocean transparency
* Change to realistic reflection and refraction ratio
* Simplify by using bumpCoord
* Working refraction
* Fix build issues and document enableRtt
* Simplify flipAcrossY in shader
* Add flipAcrossY temp fix for camera sensors
* Increase reflection for easier debugging, rm redundant code, writeContentsToFile
* Fix model.xacro.erb to match without 2nd plugin and add offset
* Attempt to merge
* Namespace tweaks.
* Move ocean_visual_below slightly to stop interference and wrong colors cutting in/out
* Add back ocean_waves_below_visual, without Visual Plugin, working as expected
* Define refl/refr amounts in model.xacro, then use visual plugin to set the parameters
* Add envReflectRatio and shallowRefractRatio variables
* Add comments, clean code, pass code check
* Reduce reflection and refraction amounts to make more subtle effect
* Integrate reflection with Gerstner waves, working well
* Fixes for the new topic layout.
* Show updated render picture
* Use reflection color
* Change plane->mesh, still shows correct reflection texture but projected onto the mesh wrong
* Change mesh->plane and show pure reflection color on it, working well but need to switch back to mesh
* Add reflection/refraction texture to shaders, but do not use yet
* Merge
* merged with master
* Remove unneeded files
* Add missing part of Fresnel
* Remove fresnel and noise effects
* Refactor C++ code and change shaders to match version 130
* Copy over files from gazebo_plugin_setup
* Add changes recommended by Ian
* Move reflectMap to default params
* Test hardcoding fragment shader to red, not working
* Remove random text
* Small fix (add void)
* Remove version and add back reflectMap
* Simplify shaders and texture, still has exception about reflectMap
* Try to improve plane, no difference
* Use shaders to show texture
* Try to use shaders, but not working
* Comment out shader and most of material (near empty material), in C++ get material and give it a texture, but will not run
* Contributors: Carlos Aguero, Ian Chen <ichen@osrfoundation.org>, Rumman Waqar <rumman.waqar05@gmail.com>, Tyler Lum <tylergwlum@gmail.com>

1.2.0 (2019-08-19)
------------------
* Try to change material script to match the newly created texture, did not work
* Rewrite code to match with rendertotexture tutorial
* Hide minimap, water constant texture, try get plane to be reflection, shows reflection but wrong geometry
* Add texture material to water
* Remove enable/disable refl to fix render issue
* Change to ogre user camera pos and orient, try but fail shaders
* Scale plane and mesh to show it
* Flip plane to be flat, need to next hide the original water
* Test removing preset shader/texture to see if water will update, will not BIG CHANGES SOON
* Create new texture unit
* Change texture name, miniscreen and plane work but not water
* Try to change ocean to show the texture, did not work yet
* Change position and angle of camera
* Add reflection texture empty
* Remove textures
* Move ocean visual plane upwards
* Use gl_ModelViewProjectionMatrix to see colorful ocean
* Work off ocean model, clean out visual plugin and use new simple material scripts
* Merged default into topic_namespace_generation
* fix catkin make install issues with meshes
* Fix source paths and variable keywords
* merged with default
* removed sdf
* Copy over Fresnel materials files
* merge default
* Merged default into Add-Option-To-Hide-Gazebo-Topics
* Merged in xacro_for_oceanwaves (pull request #153)
  Modiying world definitions in wave_gazebo package to use xacro
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Style.
* Modiying world definitions in wave_gazebo package to use xacro
* Merged in ocean-wave-xacro (pull request #150)
  Ocean wave xacro
  Approved-by: Marshall Rawson <marshallrawson@osrfoundation.org>
* cleaning
* added spinning out detection
* incremental
* removed erb from CMake
* removed ocean-waves-sdf
* functional?
* incremental
* functional
* approximate 4x4 dock block as a sphere
* merge
* fixed dock inertial issues
* model.sdf.erb edited online with Bitbucket
* model.sdf.erb edited online with Bitbucket
* model.sdf.erb edited online with Bitbucket
* added <laser_retro>-1 flags to new wave visual links
* functional
* Install world_models in wave_gazebo
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>, Jonathan Wheare <jonathan.wheare@flinders.edu.au>, Jose Luis Rivero <jrivero@osrfoundation.org>, Marshall Rawson <marshallrawson@osrfoundation.org>, MarshallRawson, MarshallRawson <marshallrawson@osrfoundation.org>, Tyler Lum <tylergwlum@gmail.com>

1.1.2 (2019-07-10)
------------------
* fixed 2016 placard joint issue
* Contributors: MarshallRawson

1.1.1 (2019-07-03)
------------------
* Missing ruby in build depend for wave_gazebo
* Contributors: Jose Luis Rivero <jrivero@osrfoundation.org>

1.1.0 (2019-07-01)
------------------
* Generate changelog for new packages
* merge with default
* changing buoy buoyancy to sphere, adding feature to generator
* Tweaks
* ready
* Connecting wave model to buoyancy plugin
* working version with dock buoyancy, but need to attach placards
* first cut - dock elements work, but to build a full dock need to add joints between elements
* tweaks
* Merged in wave_visualization_refactor (pull request #114)
  Wave visual / physics refactor
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Restoring waves parameters.
* Smooth water.
* Style
* reducing wave height to something more reasonable
* removing the ocean_waves model.sdf since it is generated via erb
* Using Ruby to generate ocean wave model SDF
* testing side by side scaling
* case 0
* temporary branch for comparing with wave_visualization
* Testing scalability of new implementation - updated hgignore vmrc->vrx
* Removed gazebo messaging, introduces redundancy in model.sdf for ocean. USV and buoyancy plugins only get wave parameters once instead of every update.
* setting model back to original seastate
* style
* adding to docs and allowing for both PMS and CWR wavefield models
* addin PM spectrum
* testing wave fields
* adding exponential increase in wave field and LaTeX doc^C
* increment
* Clean up some of the diagnostic messages
* Added wavegauge plugin to visualize physical wave height.  Setup example with buoy world.  Implemented simplified wave height calculation in WavefieldSampler for regularly spaced grid (steepness=1=0).
* verifying with examples
* toward buoy examples
* Removing superfluous models and empty tests
* Changing license text
* Modifications from original source for integration in VRX
* Adding two packages from asv_wave_sim as a part of VRC
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, Jose Luis Rivero <jrivero@osrfoundation.org>, MarshallRawson

* Merged in wave_visualization_refactor (pull request #114)
  Wave visual / physics refactor
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Removed gazebo messaging, introduces redundancy in model.sdf for ocean. USV and buoyancy plugins only get wave parameters once instead of every update.
* Added wavegauge plugin to visualize physical wave height.  Setup example with buoy world.  Implemented simplified wave height calculation in WavefieldSampler for regularly spaced grid (steepness=1=0).
* Modifications from original source for integration in VRX
* Adding two packages from asv_wave_sim as a part of VRC
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, MarshallRawson

1.0.1 (2019-03-01)
------------------

1.0.0 (2019-02-28)
------------------

0.3.3 (2018-10-19)
------------------

0.3.2 (2018-10-08)
------------------

0.3.1 (2018-10-05)
------------------

0.3.0 (2018-09-28)
------------------
