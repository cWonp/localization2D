cmake_minimum_required(VERSION 3.0.2)
project(localization2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  
  pcl_ros
  pcl_conversions
)

find_package(PCL 1.8 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
link_directories(
        include
        ${PCL_LIBRARY_DIRS}
)

add_executable(main src/main.cpp)
add_dependencies(main ${catkin_EXPORTED_TARGETS})
target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES})

