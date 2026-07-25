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
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(WELDLINE_DEFAULT_ONNX_MODEL "${CMAKE_CURRENT_SOURCE_DIR}/weights/best.onnx")
if(DEFINED ENV{WELDLINE_ONNX_MODEL})
  set(WELDLINE_DEFAULT_ONNX_MODEL "$ENV{WELDLINE_ONNX_MODEL}")
endif()
set(WELDLINE_ONNX_MODEL "${WELDLINE_DEFAULT_ONNX_MODEL}" CACHE FILEPATH
  "ONNX model validated against ONNX Runtime during build")
option(WELDLINE_VALIDATE_ONNX "Fail the build when the configured ONNX model cannot be loaded by ONNX Runtime" ON)

set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime root containing include/ and lib/")
if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include" /usr/local/include /usr/include)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}/lib" /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime C++ was not found. Run ./build.sh so scripts/ensure_onnxruntime.sh can prepare it.")
endif()
get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
file(GLOB ONNXRUNTIME_RUNTIME_LIBS "${ONNXRUNTIME_LIBRARY_DIR}/libonnxruntime.so*")
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")

add_executable(yolo_weldline_3d_node src/yolo_weldline_3d_node.cpp)
target_include_directories(yolo_weldline_3d_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_include_directories(yolo_weldline_3d_node SYSTEM PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_compile_options(yolo_weldline_3d_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(yolo_weldline_3d_node ${OpenCV_LIBS} ${ONNXRUNTIME_LIBRARY})
set_target_properties(yolo_weldline_3d_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
  INSTALL_RPATH "$ORIGIN")
ament_target_dependencies(yolo_weldline_3d_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs
  message_filters cv_bridge)

if(WELDLINE_VALIDATE_ONNX)
  if(NOT EXISTS "${WELDLINE_ONNX_MODEL}")
    message(FATAL_ERROR "Configured ONNX model does not exist: ${WELDLINE_ONNX_MODEL}")
  endif()
  add_executable(validate_onnx_onnxruntime tools/validate_onnx_onnxruntime.cpp)
  target_include_directories(validate_onnx_onnxruntime SYSTEM PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_compile_options(validate_onnx_onnxruntime PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(validate_onnx_onnxruntime ${ONNXRUNTIME_LIBRARY})
  set_target_properties(validate_onnx_onnxruntime PROPERTIES BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")
  add_custom_target(validate_weldline_onnx ALL
    COMMAND validate_onnx_onnxruntime "${WELDLINE_ONNX_MODEL}"
    DEPENDS validate_onnx_onnxruntime "${WELDLINE_ONNX_MODEL}"
    COMMENT "Validating ONNX model against ONNX Runtime"
    VERBATIM)
  add_dependencies(yolo_weldline_3d_node validate_weldline_onnx)
endif()

install(TARGETS yolo_weldline_3d_node DESTINATION lib/${PROJECT_NAME})
install(FILES ${ONNXRUNTIME_RUNTIME_LIBS} DESTINATION lib/${PROJECT_NAME})
install(FILES launch/yolo_weldline_3d.launch.xml DESTINATION share/${PROJECT_NAME}/launch)
install(FILES config/yolo_weldline_3d.yaml DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY weights DESTINATION share/${PROJECT_NAME} OPTIONAL FILES_MATCHING PATTERN "*.onnx")
install(PROGRAMS launch.sh scripts/ros_environment.sh scripts/realsense_visualize.py scripts/annotated_image_viewer.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
