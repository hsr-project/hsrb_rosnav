cmake_minimum_required(VERSION 2.8.3)
project(hsrb_mapping)

find_package(catkin REQUIRED COMPONENTS
  gmapping hector_mapping map_server slam_karto)

catkin_package()

install(DIRECTORY launch config map
     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
     USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  # tmc_lint
  # find_package(tmc_lint REQUIRED)
  # tmc_lint_python(test/test_hsrb_navigation.py)
  # add_dependencies(tests tmc_lint)
  find_package(roslint REQUIRED)
  roslint_python(test/test_hsrb_mapping.py)  
  add_dependencies(tests roslint)

  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    # skipping cartographer.launch due to cartographer_ros
    if(NOT ${LAUNCH_FILE} MATCHES "/cartographer.launch$" )
      message(STATUS "Testing ${LAUNCH_FILE}")
      roslaunch_add_file_check(${LAUNCH_FILE})
    endif()
  endforeach()
  # rostest
  file(GLOB ROSTEST_FILES test/*.test)
  foreach(ROSTEST_FILE ${ROSTEST_FILES})
    message(STATUS "Testing ${ROSTEST_FILE}")
    add_rostest(${ROSTEST_FILE})
  endforeach()
endif()
