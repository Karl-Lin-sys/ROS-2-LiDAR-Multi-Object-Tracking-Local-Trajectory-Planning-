cmake_minimum_required(VERSION 3.20)
project(ros2_mot_ltp VERSION 1.0.0 LANGUAGES CXX)

# Enforce Modern C++20 and strict compiler security flags (NIST CSF PR.PS / PR.DS)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Werror=format-security
    -fno-omit-frame-pointer -fstack-protector-strong -D_FORTIFY_SOURCE=2
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(PCL 1.12 REQUIRED COMPONENTS common filters segmentation kdtree)
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc calib3d)
find_package(Ceres 2.1 REQUIRED)

# Optional ONNX Runtime / TensorRT C++ API Discovery
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS /usr/include/onnxruntime /usr/local/include/onnxruntime ENV ONNXRUNTIME_ROOT)
find_library(ONNXRUNTIME_LIB onnxruntime
  HINTS /usr/lib /usr/local/lib ENV ONNXRUNTIME_ROOT)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

add_library(mot_ltp_component SHARED
  src/mot_ltp_node.cpp
)

target_link_libraries(mot_ltp_component
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Ceres::ceres
)

if(ONNXRUNTIME_LIB)
  target_link_libraries(mot_ltp_component ${ONNXRUNTIME_LIB})
  target_compile_definitions(mot_ltp_component PRIVATE USE_ONNXRUNTIME_API=1)
endif()

ament_target_dependencies(mot_ltp_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
)

rclcpp_components_register_node(mot_ltp_component
  PLUGIN "ros2_mot_ltp::MotLtpPipelineNode"
  EXECUTABLE mot_ltp_pipeline_exec
)

install(TARGETS mot_ltp_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES security/governance_policy.yaml
  DESTINATION share/${PROJECT_NAME}/security
)

ament_package()
