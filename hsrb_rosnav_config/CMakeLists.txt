cmake_minimum_required(VERSION 2.8.3)
project(hsrb_rosnav_config)

find_package(catkin REQUIRED COMPONENTS
  amcl dwa_local_planner map_server move_base)

catkin_package()

install(DIRECTORY launch config
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
     USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(STATUS "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
  # rostest
  file(GLOB ROSTEST_FILES test/*.test)
  foreach(ROSTEST_FILE ${ROSTEST_FILES})
    message(STATUS "Testing ${ROSTEST_FILE}")
    add_rostest(${ROSTEST_FILE})
  endforeach()
endif()
