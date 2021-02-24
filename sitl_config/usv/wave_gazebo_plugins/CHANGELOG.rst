^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wave_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2019-12-26)
------------------

1.2.6 (2019-10-04)
------------------

1.2.5 (2019-09-19)
------------------
* typo
* Fix aspect ration with reflections.
* Style.
* apply reflection / refraction only to camera sensors
* syntax tweak
* removing redundancy in variable initialization
* merging default into branch
* Merged in maintenance (pull request #174)
  Minor maintenance updates
  Approved-by: Brian Bingham <briansbingham@gmail.com>
* Changes for code checker
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>, Ian Chen <ichen@osrfoundation.org>

1.2.4 (2019-09-12)
------------------

1.2.3 (2019-09-12)
------------------
* Replace EnableVisualizations() with UserCameraCount().
* Minor maintenance updates.
* Contributors: Carlos Aguero

1.2.2 (2019-09-06)
------------------
* Combine boolean expressions.
* Make code compatible with older Gazebo minor versions.
* Contributors: Carlos Aguero

1.2.1 (2019-09-05)
------------------
* remove gazebo version check, scene blend
* Add rttNoise parameter to scale distortion in refraction
* Switch ratio->opacity naming
* Clean code for checking GZ version
* Test having higher required GZ major and minor version, but fix codecheck issues
* Test having higher required GZ major and minor version
* Fix build issues and document enableRtt
* Use #if to not use ConnectCameraPreRender in gz7
* Add enableRtt bool, may need to change as it likely still not build for gz7
* Refactor to have more functions for clarity and to setup for optional no rtt
* Clean code
* Simplify code by removing unused private variables
* Simplify NewCameras() by not using SensorManager
* Fix code to pass code check
* Clean up code with comments and removing redundant parts
* Merged in Issue#122-Ocean-Reflections_ian (pull request #170)
  test using render events
* Fix clipPlane position, need to use WorldPose()
* test using render events
* Fix to build on gz7
* Change clipPlane updates to build on gz7
* Update comments
* Add more clarity with comments
* Update header file for better comments and less redundancy
* Remove redundant cameras.size() == 0 check
* Set ocean texture only in preRenderTargetUpdate(), not postRenderTargetUpdate()
* Remove writing texture contents to file for debugging
* Add better comments, clean out redundant code, pass code check
* Move flipAcrossY to setupreflrefr() to be just set once
* Move flipAcrossY setup from preRender to Load for efficiency
* Add flipAcrossY temp fix for camera sensors
* Increase reflection for easier debugging, rm redundant code, writeContentsToFile
* Add listener for user camera correctly with OgreViewport
* Try to run addListener for usercams and camera sensors, usercam texture returning null
* Add Ian's recommended change, image_viewer still not show refl
* Make ocean reflections work, even if ocean pose changes (position and angle)
* Fix codecheck build errors
* Save textures to pngs for viewing
* Store reflect/refract texture unit states for easier usage
* Iterate through all rts in preRenderTargetUpdate() and postRenderTargetUpdate(), client working, but sensors not
* IMPORTANT FIX: remove unneeded pause variable that was never set or cleared, causing issues with updating
* Update OnRender to update all textures
* In client, add usercam. In server, add all sensors. Still working, but not camera sensors
* Remove redundant comments, still leave in variables to runs fine now, but not sure why they are needed
* Very strange, comment out these unused variables and it sometimes doesn't work (waves don't move)
* Fully functional, but if I comment out the Ogre objects, it stops working sometimes
* Transition to using vectors, but not fully. Fully functional
* Refactor to get ready for transition, fully functional tested multiple times, about to switch to vectors
* After fixing multiple head issues, stable working commit of user camera reflections
* Set correct visibility to see proper wave movement and add prints
* Use vector of cams, rtts, textures. Runs, shows reflections for user cam, but not for sensors.
* Reintegration to make refl/refr still work
* Add CreateReflRefrTexture function to work on rendering::Camera, need to finish integration
* Reorder SetupReflRefr function to only do one-time assignments, so that CreateReflRefrTexture function can be added
* Create NewCameras() function, working but need to add Usercams and cams as well
* Add new cameras from OnPreRender()
* Change render order of water to properly render propellers
* Add documentation to header file
* Fix build issues
* Define refl/refr amounts in model.xacro, then use visual plugin to set the parameters
* Fix build issues related to gz7 by storing visual name
* Add comments, clean code, pass code check
* Reduce reflection and refraction amounts to make more subtle effect
* Integrate reflection with Gerstner waves, working well
* Show updated render picture
* Change mesh->plane and show pure reflection color on it, working well but need to switch back to mesh
* Add in ocean reflection/refraction C++ code, but keep main function commented out so it still looks same as before
* Refactor C++ code and change shaders to match version 130
* Copy over files from gazebo_plugin_setup
* Add changes recommended by Ian
* Simplify shaders and texture, still has exception about reflectMap
* Try to use shaders, but not working
* Comment out shader and most of material (near empty material), in C++ get material and give it a texture, but will not run
* Stop unneeded plane object from being added to planeNode
* Contributors: Ian Chen <ichen@osrfoundation.org>, Tyler Lum <tylergwlum@gmail.com>

1.2.0 (2019-08-19)
------------------
* Go back to custom material, note if you change mytexture2 -> mytexture, it breaks it from resource group can't find error
* Try to change plane material to use existing reflection material and only edit the texture, but does not work
* Fix code quality to pass pipeline
* Try to change material script to match the newly created texture, did not work
* Add jpg texture mix with ocean, worked decently
* Try to add miniscreen to see the material/texture, but not working for some reason
* Disable clip plane each post render, working very well
* Turn on and off reflection and clip plane in pre/post render
* Rewrite code to match with rendertotexture tutorial
* Add reflection to plane
* Add comments and documentation and removed unneeded parts
* Hide minimap, water constant texture, try get plane to be reflection, shows reflection but wrong geometry
* Add texture material to water
* Go back to orig user camera
* Unsuccessful attempt to switch cameras
* Show difference between Ogre::Cam and gz:rend:Cam position
* Add code from book to use new camera, needs update
* Remove enable/disable refl to fix render issue
* Hide plane from texture
* Change to ogre user camera pos and orient, try but fail shaders
* BIG CLEANUP, removed old unused lines of code
* Add enableRefl and disableRelf
* Scale plane and mesh to show it
* Flip plane to be flat, need to next hide the original water
* Create new texture unit
* Change texture name, miniscreen and plane work but not water
* Try to change ocean to show the texture, did not work yet
* Put texture onto plane
* Make only one visualplugin to remove extra miniscreen
* Add rendertargetlistener to not show miniscreen (still shows because there are two)
* Try to implement it, did not work
* Add WavefieldRenderTargetListener, completely untested
* Update miniscreen continuously
* SUCCESSFULLY show small version in mini screen
* Add view to miniscreen, ugly
* Add miniscreen
* Change position and angle of camera
* Change angle to view something
* Save to image file, it is blank
* Add render texture
* Add texture
* Change to valid image
* Add plane image, looks weird
* Move user camera
* Added a light
* Add render updates
* Add RTShaderSystem
* Add static function variable to differentiate between Ogre names
* Fix scene, still not working
* Not working setup, likely need to use visualptr to get scene
* Add scene ptr
* Add viewport setup
* Add scene nodes and camera setup
* Add root, scenemgr
* Add unworking Ogre texture creation
* Work off ocean model, clean out visual plugin and use new simple material scripts
* Modiying world definitions in wave_gazebo package to use xacro
* Contributors: Brian Bingham <briansbingham@gmail.com>, Tyler Lum <tylergwlum@gmail.com>

1.1.2 (2019-07-10)
------------------
* Workaround to fix compile errors on Kinetic
  The version of ign-math2 present in Ubuntu Xenial (2.2.3) lacks
  of some features (Zero or Length) implemented starting on 2.3.x.
  This change add some preprocessors defines to workaround the
  problem. A more elegant solution would be ideal.
* Contributors: Jose Luis Rivero <jrivero@osrfoundation.org>

1.1.1 (2019-07-03)
------------------

1.1.0 (2019-07-01)
------------------
* Generate changelog for new packages
* Merged in wave_visualization_refactor (pull request #114)
  Wave visual / physics refactor
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Restoring waves parameters.
* Smooth water.
* Style
* Removing gazebo::msg::Param references and cleaning up for gazebo version < 8 compatibility.
* Removed gazebo messaging, introduces redundancy in model.sdf for ocean. USV and buoyancy plugins only get wave parameters once instead of every update.
* style
* adding to docs and allowing for both PMS and CWR wavefield models
* addin PM spectrum
* adding exponential increase in wave field and LaTeX doc^C
* increment
* increment
* Clean up some of the diagnostic messages
* Added wavegauge plugin to visualize physical wave height.  Setup example with buoy world.  Implemented simplified wave height calculation in WavefieldSampler for regularly spaced grid (steepness=1=0).
* verifying with examples
* changing wind to waves
* Added an example to illustrate using request/response to transport the wave_params and fixed a couple tiny typos
* Overtly requiring C++14 for the wave_gazebo_plugins package - required for use of autos in lambda functions.  Only necessary for supporting Kinetic build.
* Setting wave parameters by hand in source for testing
* Removing superfluous models and empty tests
* Changing license text
* Modifications from original source for integration in VRX
* Adding two packages from asv_wave_sim as a part of VRC
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>, Jose Luis Rivero <jrivero@osrfoundation.org>

* Merged in wave_visualization_refactor (pull request #114)
  Wave visual / physics refactor
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Removing gazebo::msg::Param references and cleaning up for gazebo version < 8 compatibility.
* Removed gazebo messaging, introduces redundancy in model.sdf for ocean. USV and buoyancy plugins only get wave parameters once instead of every update.
* Added wavegauge plugin to visualize physical wave height.  Setup example with buoy world.  Implemented simplified wave height calculation in WavefieldSampler for regularly spaced grid (steepness=1=0).
* Added an example to illustrate using request/response to transport the wave_params and fixed a couple tiny typos
* Overtly requiring C++14 for the wave_gazebo_plugins package - required for use of autos in lambda functions.  Only necessary for supporting Kinetic build.
* Setting wave parameters by hand in source for testing
* Modifications from original source for integration in VRX
* Adding two packages from asv_wave_sim as a part of VRC
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>

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
