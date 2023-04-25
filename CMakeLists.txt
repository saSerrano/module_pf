cmake_minimum_required(VERSION 2.8.3)
project(module_pf)

set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  homer_mapnav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rb1track_pkg
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs homer_mapnav_msgs
#  DEPENDS system_lib
)

#find_package( OpenCV REQUIRED )

###########
## Build ##
###########

set(T_SOURCE
  src/TrackerNode.cpp
  src/TrackedObject.cpp
  src/Tracker.cpp
  src/SSDetector.cpp
)

set(P_SOURCE
  src/PathManager.cpp
)

set(PH_SOURCE                                                                                           
    src/ControlPanHead.cpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
set(opencv_LOCAL_INCLUDE_DIRS "/home/markovito/code/opencv/install/include" "/home/markovito/code/opencv/install/include/opencv4")
set(opencv_LOCAL_LIBS_DIR "/home/markovito/code/opencv/install/lib")

#set (cv_bridge_LOCAL_INCLUDE_DIRS /home/rb1/catkin_ws/src/cv3_bridge/include)
#set (cv_bridge_LOCAL_LIBS_DIR /home/rb1/catkin_ws/devel/lib)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${opencv_LOCAL_INCLUDE_DIRS}
#  ${cv_bridge_LOCAL_INCLUDE_DIRS}
)

link_directories(
  ${opencv_LOCAL_LIBS_DIR}
#  ${cv_bridge_LOCAL_LIBS_DIR}
)

set(opencv_LOCAL_LIBS 
  -lopencv_core 
  -lopencv_dnn 
  -lopencv_imgproc 
  -lopencv_highgui 
  -lopencv_video 
  -lopencv_videoio 
  -lopencv_imgcodecs 
  -lopencv_features2d 
  -lopencv_flann
)
#set (cv_bridge_LOCAL_LIBS -lcv3_bridge)

add_executable(${PROJECT_NAME}_tracker_node src/tracker_node.cpp ${T_SOURCE})
target_link_libraries(${PROJECT_NAME}_tracker_node ${catkin_LIBRARIES} ${opencv_LOCAL_LIBS})
#target_link_libraries(${PROJECT_NAME}_tracker_node ${cv_bridge_LOCAL_LIBS} ${catkin_LIBRARIES} ${opencv_LOCAL_LIBS})

add_executable(${PROJECT_NAME}_path_manager_node src/path_manager_node.cpp ${P_SOURCE})
target_link_libraries(${PROJECT_NAME}_path_manager_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_panhead_node src/panhead_node.cpp ${PH_SOURCE})
target_link_libraries(${PROJECT_NAME}_panhead_node ${catkin_LIBRARIES})

