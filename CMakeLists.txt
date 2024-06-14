cmake_minimum_required(VERSION 3.0.2)
project(hikrobot_mvs_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  hik_include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS}
  lib
)

# add_executable(grabImg src/grab.cpp)
# add_executable(grabImgWithTrigger src/grab_trigger.cpp)
# add_executable(Trigger_Image src/Trigger_Image.cpp)

# target_link_libraries(
#   grabImg
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   -lMvCameraControl
#   -lpthread
#   )

# target_link_libraries(
#   grabImgWithTrigger
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   -lMvCameraControl
#   -lpthread
#   -lrt
#   )

# target_link_libraries(
#   Trigger_Image
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   -lMvCameraControl
#   -lpthread
#   )

add_executable(${PROJECT_NAME} src/grab_image.cpp src/camera.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  -lMvCameraControl
  -lpthread
  -lMvUsb3vTL
  -lMVGigEVisionSDK
  -lrt
)