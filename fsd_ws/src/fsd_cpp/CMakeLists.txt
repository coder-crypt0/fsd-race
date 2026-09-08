cmake_minimum_required(VERSION 3.16)
project(fsd_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fsd_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# ---------------------------------------------------------------- CUDA
# Compiles the GPU segmentation kernel when a CUDA toolchain exists
# (JetPack on the Orin ships one). Without CUDA the stereo node builds
# with its complete CPU fallback — same results, more CPU load.
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    # 87 = Jetson Orin. On a dev PC: colcon build --cmake-args -DCMAKE_CUDA_ARCHITECTURES=native
    set(CMAKE_CUDA_ARCHITECTURES 87)
  endif()
  add_library(fsd_cuda STATIC src/cuda/segmentation.cu)
  target_include_directories(fsd_cuda PUBLIC include)
  set_target_properties(fsd_cuda PROPERTIES POSITION_INDEPENDENT_CODE ON)
  message(STATUS "fsd_cpp: CUDA enabled (arch ${CMAKE_CUDA_ARCHITECTURES})")
else()
  message(STATUS "fsd_cpp: no CUDA compiler found, building CPU-only")
endif()

function(fsd_node name)
  add_executable(${name} src/${name}.cpp)
  target_include_directories(${name} PRIVATE include)
  ament_target_dependencies(${name}
    rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs fsd_msgs fs_msgs cv_bridge)
  target_link_libraries(${name} ${OpenCV_LIBS})
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endfunction()

fsd_node(stereo_cone_node)
if(CMAKE_CUDA_COMPILER)
  target_compile_definitions(stereo_cone_node PRIVATE FSD_USE_CUDA)
  target_link_libraries(stereo_cone_node fsd_cuda CUDA::cudart)
endif()

fsd_node(state_estimation_node)
fsd_node(cone_mapping_node)
fsd_node(path_planning_node)
fsd_node(motion_control_node)
fsd_node(safety_supervisor_node)
fsd_node(fsds_adapter_node)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
