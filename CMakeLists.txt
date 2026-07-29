cmake_minimum_required(VERSION 2.6)
project(wam_teleop)

option(BUILD_LEADER "Build leader executable" ON)

# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

include(FetchContent)
find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   message_generation
)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG "0.8.0"
)
FetchContent_MakeAvailable(yaml-cpp)

add_message_files(
   FILES
   GravityTorque.msg
)

generate_messages( DEPENDENCIES std_msgs)

catkin_package(
   # INCLUDE_DIRS include
   # LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
  DESTINATION /etc/teleop
  FILES_MATCHING PATTERN "teleop_config.yaml"
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_WRITE GROUP_EXECUTE
    WORLD_READ WORLD_WRITE WORLD_EXECUTE
)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

## Gripper 
find_package(gripper REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

if (BUILD_LEADER) 
    find_package(haptic_wrist 2.0 REQUIRED)
    add_executable(leader src/leader.cpp src/lib/leader_udp_handler.cpp src/lib/utils.cpp)
    target_link_libraries(leader ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} haptic_wrist Boost::system Boost::thread yaml-cpp)
    target_compile_definitions(leader PRIVATE BUILD_LEADER)
    # target_compile_options(leader PRIVATE -fsanitize=address -g) 
    # target_link_options(leader PRIVATE -fsanitize=address)
    add_dependencies(leader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

add_executable(follower src/follower.cpp src/lib/follower_udp_handler.cpp src/lib/utils.cpp)
target_link_libraries(follower ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} gripper::gripper Boost::system Boost::thread yaml-cpp)
# target_compile_options(follower PRIVATE -fsanitize=address -g) 
# target_link_options(follower PRIVATE -fsanitize=address)
add_dependencies(follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_handle src/test_handle_gripper.cpp)
target_link_libraries(test_handle ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} haptic_wrist gripper::gripper Boost::system Boost::thread)
add_dependencies(leader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
