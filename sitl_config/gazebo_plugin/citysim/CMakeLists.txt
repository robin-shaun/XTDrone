cmake_minimum_required(VERSION 2.8.3)
project(citysim)

set (CITYSIM_MAJOR_VERSION 0)
set (CITYSIM_MINOR_VERSION 1)
set (CITYSIM_PATCH_VERSION 0)
set (CITYSIM_VERSION_FULL ${CITYSIM_MAJOR_VERSION}.${CITYSIM_MINOR_VERSION}.${CITYSIM_PATCH_VERSION})

# Packaging configuration
set (CPACK_PACKAGE_VERSION "${CITYSIM_VERSION_FULL}")
set (CPACK_PACKAGE_VERSION_MAJOR "${CITYSIM_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${CITYSIM_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${CITYSIM_PATCH_VERSION}")
list (APPEND CPACK_SOURCE_GENERATOR "TBZ2")
list (APPEND CPACK_SOURCE_IGNORE_FILES ";/.hg/;.hgignore;.swp$;/build/")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${CITYSIM_VERSION_FULL}")
include (CPack)

#####################################
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "Release" CACHE STRING
    "Choose the type of build, options are: Debug Release RelWithDebInfo" FORCE)
  message(STATUS "Build type not selected: Release selected by default")
endif (NOT CMAKE_BUILD_TYPE)

find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package(gazebo 9 REQUIRED)

include(GNUInstallDirs)

if(NOT DEFINED CMAKE_INSTALL_LIBDIR)
  set(CMAKE_INSTALL_LIBDIR lib)
endif(NOT DEFINED CMAKE_INSTALL_LIBDIR)
set(CITYSIM_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/citysim-${CITYSIM_MAJOR_VERSION}/plugins
)

# Generate the setup.sh file
configure_file(${CMAKE_SOURCE_DIR}/cmake/setup.sh.in ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/citysim-${CITYSIM_MAJOR_VERSION}/)
# Also install the setup.sh in an unversioned location
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/citysim/)


add_subdirectory(plugins)
add_subdirectory(models)
add_subdirectory(worlds)
add_subdirectory(media)
