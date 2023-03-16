cmake_minimum_required(VERSION 3.0.2)
project(cluster_cloud)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS    
    cv_bridge
    pcl_conversions
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

#寻找PCL包
find_package(PCL REQUIRED )
IF(PCL_FOUND)
    message("PCL lib is found:" ${PCL_INCLUDE_DIRS})
    message("PCL lib:" ${PCL_LIBRARIES}) 
ENDIF(PCL_FOUND)

#寻找OpenCV包
find_package(OpenCV REQUIRED)
IF(OpenCV_FOUND)
    message("OpenCV lib is found:" ${OpenCV_INCLUDE_DIRS})
    message("OpenCV lib:" ${OpenCV_LIBS}) 
ENDIF(OpenCV_FOUND)

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OPENCV_LIBRARY_DIRS}
)

add_executable(cluster_cloud src/main.cpp)
target_link_libraries(cluster_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
