cmake_minimum_required(VERSION 3.5)
project(ur_env)

find_package(ament_cmake REQUIRED)

# Add worlds directory to the installation
install(DIRECTORY 
  config 
  launch 
  worlds     # chess world
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  find_package(ament_cmake_pytest REQUIRED)
  add_launch_test(test/test_gazebo.py
    TIMEOUT
      180
  )
endif()

ament_package()