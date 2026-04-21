cmake_minimum_required(VERSION 3.14)
project(bag2vid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# System dependencies
find_package(OpenCV REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Multimedia MultimediaWidgets Concurrent)

# Library
add_library(${PROJECT_NAME}_lib SHARED
  src/backend/Extractor.cpp
  src/frontend/Visualiser.cpp
  src/frontend/Timeline.cpp
  src/frontend/VideoPlayer.cpp
  src/frontend/PlaybackClock.cpp
  include/bag2vid/frontend/Visualiser.hpp
  include/bag2vid/frontend/Timeline.hpp
  include/bag2vid/frontend/VideoPlayer.hpp
  include/bag2vid/frontend/PlaybackClock.hpp
  resources/resources.qrc
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC include)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rosbag2_cpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  Qt6::Core
  Qt6::Gui
  Qt6::Widgets
  Qt6::Multimedia
  Qt6::MultimediaWidgets
  Qt6::Concurrent
)

# GUI executable
add_executable(bag2vid_gui nodes/bag2vid_gui.cpp)
target_link_libraries(bag2vid_gui ${PROJECT_NAME}_lib)

# CLI executable
add_executable(bag2vid_node nodes/bag2vid_node.cpp)
target_link_libraries(bag2vid_node ${PROJECT_NAME}_lib)

# Install
install(TARGETS ${PROJECT_NAME}_lib bag2vid_gui bag2vid_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
