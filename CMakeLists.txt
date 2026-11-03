cmake_minimum_required(VERSION 3.8)
project(rover_arm_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

# Plugin de Gazebo que sujeta los objetos con el gripper (src/grasp_weld.cc)
add_library(rover_arm_sim_grasp_weld SHARED src/grasp_weld.cc)
set_property(TARGET rover_arm_sim_grasp_weld PROPERTY CXX_STANDARD 17)
target_link_libraries(rover_arm_sim_grasp_weld
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8)
install(TARGETS rover_arm_sim_grasp_weld LIBRARY DESTINATION lib)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/driver_emulator
  scripts/virtual_joystick
  scripts/wait_for_feedback
  scripts/build_model
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch meshes udev tools DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test TIMEOUT 300)
endif()

ament_package()
