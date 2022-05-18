set (rostests_python
  ros_network/ros_network_default.test
  ros_network/ros_network_disabled.test
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
    foreach (rostest ${rostests_python})
      # We don't set a timeout here because we trust rostest to enforce the
      # timeout specified in each .test file.
      add_rostest(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
      # Check for test result file and create one if needed.  rostest can fail to
      # generate a file if it throws an exception.
      add_test(check_${rostest} rosrun rosunit check_test_ran.py 
               --rostest ${ROS_PACKAGE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
  endforeach()
endif()

install(PROGRAMS
  ros_network/ros_api_checker
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
