cmake_minimum_required(VERSION 3.8)
project(dynaarm_description VERSION 0.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
