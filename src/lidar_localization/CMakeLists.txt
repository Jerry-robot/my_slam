# cmake_minimum_required(VERSION 3.0.2)
# project(lidar_localization)

# ## Compile as C++11, supported in ROS Kinetic and newer
# # 用于构建的优化的库或可执行文件，不包含调试符号
# SET(CMAKE_BUILD_TYPE "Release")
# # 启用优化（1～3）编译成release版本,-Wall是开启所有编译警告
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# # 针对所有编译器设置编译选项
# add_compile_options(-std=c++11)
# add_definitions(-std=c++11)
# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   geometry_msgs
#   pcl_ros
#   roscpp
#   rospy
#   std_msgs
#   tf
#   eigen_conversions
# )
# catkin_package()

# SET(ALL_TARGET_LIBRARIES "")
# include_directories(include ${catkin_INCLUDE_DIRS})

# include (cmake/glog.cmake)
# include (cmake/PCL.cmake)
# include (cmake/eigen.cmake)
# include (cmake/geographic.cmake)
# include (cmake/global_defination.cmake)

# file(GLOB_RECURSE ALL_SRCS "*.cpp")
# file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
# file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
# list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
# list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

# add_library(${PROJECT_NAME}
#   src/sensor_data/gnss_data.cpp
#   src/subscriber/cloud_subscriber.cpp
#   src/subscriber/gnss_subscriber
#   src/subscriber/imu_subscriber.cpp
# )

# add_executable(test_frame_node src/test_frame_node.cpp)
# add_dependencies(test_frame_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_frame_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})






cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
)
set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(test_frame_node src/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})