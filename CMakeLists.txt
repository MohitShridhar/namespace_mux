cmake_minimum_required(VERSION 2.8.3)
project(namespace_mux)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  std_srvs
  topic_tools
)

find_package(Boost 1.40 COMPONENTS program_options REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES namespace_mux
  CATKIN_DEPENDS roscpp rospy rviz std_srvs topic_tools
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

# set the default path for built executables to the "bin" directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_executable(namespace_mux src/namespace_mux.cpp src/dynamic_topic_relay.cpp)
target_link_libraries(namespace_mux ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_executable(fake_map_broadcaster src/fake_map_broadcaster.cpp)
target_link_libraries(fake_map_broadcaster ${catkin_LIBRARIES})
