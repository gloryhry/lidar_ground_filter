cmake_minimum_required(VERSION 2.8.3)
project(pcl_test)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
message_filters
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})


add_executable(${PROJECT_NAME}_node src/pcl_test_node.cpp src/pcl_test_core.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(multi2single_lidar src/multi2single.cpp)
target_link_libraries(multi2single_lidar 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(ground_filter_node src/ground_filter_node.cpp
                                  src/ground_filter.cpp)
target_link_libraries(ground_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
