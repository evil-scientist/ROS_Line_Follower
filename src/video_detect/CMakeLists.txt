cmake_minimum_required(VERSION 2.8.3)
project(video_detect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  geometry_msgs 
  genmsg
  message_generation
)

find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  geometry_msgs 
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
include  ${catkin_INCLUDE_DIRS}
)


add_executable(get_image src/get_image.cpp)

target_link_libraries(get_image ${catkin_LIBRARIES}
								${OpenCV_LIBS})

add_dependencies(get_image video_detect_generate_messages_cpp)
