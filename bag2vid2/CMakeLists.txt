cmake_minimum_required(VERSION 3.5)
project(bag2vid2)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt6Widgets REQUIRED)
find_package(Qt6PrintSupport REQUIRED)
find_package(Qt6Multimedia REQUIRED)
find_package(Qt6MultimediaWidgets REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Qt6Widgets_INCLUDE_DIRS}
  ${Qt6Multimedia_INCLUDE_DIRS}
  ${Qt6MultimediaWidgets_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/backend/Extractor.cpp
  src/frontend/Visualiser.cpp
  src/frontend/Timeline.cpp
  src/frontend/VideoPlayer.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBS}
  Qt6::Core
  Qt6::Gui
  Qt6::Widgets
  Qt6::Multimedia
  Qt6::MultimediaWidgets
)

# Declare executables
add_executable(bag2vid_node src/nodes/bag2vid_node.cpp)
add_executable(bag2vid_gui src/nodes/bag2vid_gui.cpp src/frontend/Visualiser.cpp src/frontend/VideoPlayer.cpp)

# Link executables to libraries
target_link_libraries(bag2vid_node
  ${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(bag2vid_gui
  ${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBS}
  Qt6::Core
  Qt6::Gui
  Qt6::Widgets
  Qt6::Multimedia
  Qt6::MultimediaWidgets
)

# Install targets
install(TARGETS ${PROJECT_NAME} bag2vid_node bag2vid_gui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

# Ament package
ament_package()