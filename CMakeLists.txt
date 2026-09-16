cmake_minimum_required(VERSION 3.8)
project(gazebo_sfm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

## Package Setup
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

set(GZ_PLUGIN_VER ${gz-plugin1_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED COMPONENTS profiler)

set(GZ_COMMON_VER ${gz-common4_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)

set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

## Library Setup
add_library(GazeboSFM SHARED src/GazeboSFM.cpp)

target_include_directories(
    GazeboSFM PRIVATE include
)

target_link_libraries(GazeboSFM PRIVATE
    ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER}
)

## Library Install
install(
    TARGETS GazeboSFM
    DESTINATION lib/${PROJECT_NAME}
    # ARCHIVE DESTINATION lib
    # LIBRARY DESTINATION lib
    # LIBRARY DESTINATION ${ignition-gazebo6_PLUGIN_PATH}
    # RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
