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

set(WELDLINE_ONNX_MODEL "${CMAKE_CURRENT_SOURCE_DIR}/weights/best.onnx" CACHE FILEPATH
  "ONNX model validated against the selected target inference backend during build")
set(WELDLINE_INFERENCE_BACKEND "auto" CACHE STRING "Inference backend: auto, opencv, tensorrt")
set_property(CACHE WELDLINE_INFERENCE_BACKEND PROPERTY STRINGS auto opencv tensorrt)
option(WELDLINE_VALIDATE_ONNX "Fail the build when the configured ONNX model cannot be loaded by the selected target backend" ON)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/local/cuda/include)
find_library(TENSORRT_NVINFER_LIBRARY nvinfer
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/cuda/lib64)
find_library(TENSORRT_ONNXPARSER_LIBRARY nvonnxparser
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/cuda/lib64)
find_library(CUDA_RUNTIME_LIBRARY cudart
  PATHS /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

set(WELDLINE_RESOLVED_BACKEND "${WELDLINE_INFERENCE_BACKEND}")
if(WELDLINE_RESOLVED_BACKEND STREQUAL "auto")
  if(TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER_LIBRARY AND TENSORRT_ONNXPARSER_LIBRARY AND CUDA_RUNTIME_LIBRARY)
    set(WELDLINE_RESOLVED_BACKEND "tensorrt")
  else()
    set(WELDLINE_RESOLVED_BACKEND "opencv")
  endif()
endif()
if(NOT WELDLINE_RESOLVED_BACKEND MATCHES "^(opencv|tensorrt)$")
  message(FATAL_ERROR "WELDLINE_INFERENCE_BACKEND must be one of: auto, opencv, tensorrt")
endif()
if(WELDLINE_RESOLVED_BACKEND STREQUAL "tensorrt")
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_NVINFER_LIBRARY OR NOT TENSORRT_ONNXPARSER_LIBRARY OR NOT CUDA_RUNTIME_LIBRARY)
    message(FATAL_ERROR "TensorRT backend requested, but TensorRT/CUDA libraries were not found on this target")
  endif()
endif()
message(STATUS "Weldline inference backend: ${WELDLINE_RESOLVED_BACKEND}")

add_executable(yolo_weldline_3d_node src/yolo_weldline_3d_node.cpp)
target_include_directories(yolo_weldline_3d_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_compile_options(yolo_weldline_3d_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(yolo_weldline_3d_node ${OpenCV_LIBS})
ament_target_dependencies(yolo_weldline_3d_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs
  message_filters cv_bridge)
if(WELDLINE_RESOLVED_BACKEND STREQUAL "tensorrt")
  target_compile_definitions(yolo_weldline_3d_node PRIVATE WELDLINE_HAS_TENSORRT)
  target_include_directories(yolo_weldline_3d_node PRIVATE ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(yolo_weldline_3d_node
    ${TENSORRT_NVINFER_LIBRARY} ${TENSORRT_ONNXPARSER_LIBRARY} ${CUDA_RUNTIME_LIBRARY})
endif()

if(WELDLINE_VALIDATE_ONNX)
  if(NOT EXISTS "${WELDLINE_ONNX_MODEL}")
    message(FATAL_ERROR "Configured ONNX model does not exist: ${WELDLINE_ONNX_MODEL}")
  endif()
  if(WELDLINE_RESOLVED_BACKEND STREQUAL "tensorrt")
    add_executable(validate_onnx_tensorrt tools/validate_onnx_tensorrt.cpp)
    target_include_directories(validate_onnx_tensorrt PRIVATE ${TENSORRT_INCLUDE_DIR})
    target_compile_options(validate_onnx_tensorrt PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(validate_onnx_tensorrt
      ${TENSORRT_NVINFER_LIBRARY} ${TENSORRT_ONNXPARSER_LIBRARY} ${CUDA_RUNTIME_LIBRARY})
    add_custom_target(validate_weldline_onnx ALL
      COMMAND validate_onnx_tensorrt "${WELDLINE_ONNX_MODEL}"
      DEPENDS validate_onnx_tensorrt "${WELDLINE_ONNX_MODEL}"
      COMMENT "Validating ONNX model against target TensorRT runtime"
      VERBATIM)
  else()
    add_executable(validate_onnx_opencv tools/validate_onnx_opencv.cpp)
    target_include_directories(validate_onnx_opencv PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_compile_options(validate_onnx_opencv PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(validate_onnx_opencv ${OpenCV_LIBS})
    add_custom_target(validate_weldline_onnx ALL
      COMMAND validate_onnx_opencv "${WELDLINE_ONNX_MODEL}"
      DEPENDS validate_onnx_opencv "${WELDLINE_ONNX_MODEL}"
      COMMENT "Validating ONNX model against target OpenCV DNN runtime"
      VERBATIM)
  endif()
  add_dependencies(yolo_weldline_3d_node validate_weldline_onnx)
endif()

install(TARGETS yolo_weldline_3d_node DESTINATION lib/${PROJECT_NAME})
install(FILES launch/yolo_weldline_3d.launch.xml DESTINATION share/${PROJECT_NAME}/launch)
install(FILES config/yolo_weldline_3d.yaml DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY weights DESTINATION share/${PROJECT_NAME} OPTIONAL FILES_MATCHING PATTERN "*.onnx")
install(PROGRAMS launch.sh scripts/ros_environment.sh scripts/realsense_visualize.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
