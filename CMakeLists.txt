cmake_minimum_required(VERSION 3.16)
project(weldline_reflectivity_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)

add_executable(yolo_weldline_3d_node src/yolo_weldline_3d_node.cpp)
target_include_directories(yolo_weldline_3d_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_compile_options(yolo_weldline_3d_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(yolo_weldline_3d_node ${OpenCV_LIBS})
ament_target_dependencies(yolo_weldline_3d_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs
  message_filters cv_bridge)

install(TARGETS yolo_weldline_3d_node DESTINATION lib/${PROJECT_NAME})
install(FILES launch/yolo_weldline_3d.launch.xml DESTINATION share/${PROJECT_NAME}/launch)
install(FILES config/yolo_weldline_3d.yaml DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY weights DESTINATION share/${PROJECT_NAME} OPTIONAL FILES_MATCHING PATTERN "*.onnx")
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
