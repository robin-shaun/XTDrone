^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sicktoolbox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.104 (2019-05-04)
--------------------

1.0.103 (2013-08-21)
--------------------
* No more Willow Garage email.
* Update CMakeLists.txt
* Contributors: Chad Rockey

1.0.102 (2013-03-28)
--------------------
* Update README
* Catkinized and converted to CMake.
* Initial package.xml and CmakeLists.txt
* Applied power_delay.patch.
* Applying unistd.patch
* Applied stdlib_include.patch
* Removing Matlab support.
* Commiting configured files for cmake conversion.
* Forgot an installation step for the INSTALL readme.
* Updated install files, updated install directions.
* Some minor changes
* Cleaned up and began adjusting mex interface a bit for the lms1xx
* Actually, the previous post should have been renamaing 1ms to lms2xx.  Now I am adding lms1xx
* Added directory for lms1xx in matlab examples directory
* Added include for GCC4.3
* A number of improvements, bug fixes. Also put together mex file for streaming values via the lms 1xx using the given C++ driver.  Adjusted mex build scripts to build and include lms1xx.
* deleting sicklms2xxlite
* Changed name of sickld-1.0 to sickld
* Moving mex lms dir to lms2xx
* fixed some bugs and restructured the driver to provide more low-level control
* Added range and reflectivity streaming; however, still can't figure out how to successfully transition between measurement modes without uninitializing. Lots of cleanup
* Adjusted the lms1xx app and some minor cleanup
* Added some range measurement acquisition to driver
* Polished configuration functions
* Adjusted lms1xx example file
* committing cahnges to base and sick ld & lms2xx drivers
* Fixed some bugs and setup configuration capability for the lms 1xx driver
* Started adding _getSickStatus and began writing the message parsing code for the Sick LMS 1xx - currently the LMS 1xx will compile but is unstable
* Setup connect/disconnect routines and buffer monitor for SickLMS1xx driver. Also, included an example for testing the code right now.
* Started adding driver files for SickLMS1xx unit
* Adjusted lms examples to using renamed sicklms2xx class. Also fixed corresponding makefiles and changed the prefix for each lms example to lms2xx (e.g. lms_config is now lms2xx_config)
* Moved c++/examples/lms to c++/examples/lms2xx
* Supplanted SickLMS with SickLMS2xx and sick_lms with sick_lms_2xx where appropriate.  Currently compiles, but needs a sanity test
* Created sicklms1xx driver directory and Makefile.am for each directory (currently, they are empty)
* Updating configure.ac to use the new AX macros for lms1xx and lms2xx source directories
* changes sicklms-1.0 to sicklms2xx and sicklmslite-1.0 to sicklms1xx
* Adding macros for setting lms2xx and lms1xx source directories
* Changed lms driver directory to lms2xx and created lms1xx drive directory in c++/drivers
* Fixed a bug concerning dynamically allocating strings on the stack of an active exception. Thanks to Philipp Aumayr and Simon Opelt for their original patch, which motivated this fix.
* Fixed quotation bug for newer versions of autoconf/aclocal.
  See http://www.gnu.org/software/autoconf/manual/html_node/Quoting-and-Parameters.html
* Added stdlib.h to ld_config's main.cc and lms_config's main.cc
* 
* 
* 
* 
* Added ld_lite directory
* Addedd lms_lite directory
* 
* 
* Added ld_lite folder for ld_lite driver
* Added lms_lite directory for lms_lite driver
* lmsmex.cc now supports up to four separate Sick LMS units.
* Set lmsmex.cc to also return a vector of bearings for range and/or reflectivity values.  The coordinate system corresponds to that defined in the Sick LMS 2xx manuals.  Adjusted lms_cart and lms_stream examples to use this vector.
* Removed SickConfig.hh from the driver headers and adjusted the installation to leave out SickConfig.hh and the utility headers as per Tully's suggestion.
* Changed std::cerr to std::cout for printing 'cancel buffer monitor' string
* Adjusted m-file comments for sickld and sicklms
* Adjusted m-file comments
* Fixing permissions
* Still fixing file permissions
* Adjusting file permissions
* Took out std::cerr in bad checksum exception handler.
* Took out std::cerr in bad checksum exception handler.
* Monitor now clears buffer on bad checksum
* Monitor now clears msg container on bad checksum
* Fixed print-out in build_mex script
* Took out packed attribute for structs in SickLMS.hh.
* Removed try/catch in buffer monitor base
* Adjusted the examples to exit more cleanly.
* Fixed rm -r bug in build_mex and install_mex
* Adjusted configure.ac
* Fixing README
* testing
* Adjusted NEWS
* Modified INSTALL
* Fixing permissions on matlab/install_mex.
* Fixing permissions with mex bash script.
* Changed dates in mex installation bash scripts.
* Removed doc directory from c++/drivers/ld.
* Initial project import.
* Contributors: Chad Rockey, Chris Mansley, Jason Derenick, Michael Sands, Tom Miller, chadrockey
