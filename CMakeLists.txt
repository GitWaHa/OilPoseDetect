cmake_minimum_required(VERSION 2.8.3)
project(oil_pose_detector)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++14")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS cmake_modules 
tf 
tf_conversions 
eigen_conversions 
message_generation 
roscpp 
rospy 
sensor_msgs
geometry_msgs
std_msgs 
message_filters 
cv_bridge 
image_transport 
compressed_image_transport 
compressed_depth_image_transport
moveit_core
moveit_ros_planning
moveit_ros_planning_interface)

# Opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBRARIES}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# PCL 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(CUDA REQUIRED)

if(${CUDA_FOUND})
# add_definitions(-DGPU_CUDA)
# SET(CUDA_NVCC_FLAGS -Xcompiler -std=c++11 -Xcompiler -fPIC)
  message("CUDA_FOUND")
endif(${CUDA_FOUND})


add_service_files(
  FILES
  OilPoseDetector.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
) 

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs
DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# add_executable (oil_filler_seg src/oil_filler_seg.cpp)
# target_link_libraries (oil_filler_seg ${PCL_LIBRARIES} ${catkin_LIBRARIES})

# add_executable (center_pose src/center_pose.cpp)
# target_link_libraries (center_pose ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

# add_executable (oil_filler_pose src/oil_filler_pose.cpp)
# target_link_libraries (oil_filler_pose
#         ${PCL_LIBRARIES}
#         ${OpenCV_LIBRARIES}
#         ${catkin_LIBRARIES})

add_executable (detect_oil_pose src/detect_oil_pose.cpp
  src/oil_detect/oil_detect.cpp
)
target_link_libraries (detect_oil_pose
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
${catkin_LIBRARIES}
${image_transport_LIBRARIES}
${cv_bridge_LIBRARIES})

add_executable (split_plane src/split_plane.cpp)
target_link_libraries (split_plane
${PCL_LIBRARIES}
${catkin_LIBRARIES})

# 
add_library(tsdf_fusion src/tsdf_fusion/tsdf_fusion.cpp)

cuda_add_library(tsdf_cuda STATIC
  src/tsdf_fusion/tsdf_cuda.cu
)

add_executable (detect_oil_with_reconstruct src/detect_oil_with_reconstruct.cpp
  src/oil_detect/oil_detect_tsdf.cpp
  src/oil_detect/oil_accurate_detect.cpp
  src/oil_detect/oil_rough_detect.cpp
  src/tsdf_fusion/topics_capture.cpp
)
# add_dependencies(detect_oil_with_reconstruct tsdf_fusion)
target_link_libraries (detect_oil_with_reconstruct
  tsdf_fusion
  tsdf_cuda
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

add_executable (test_tsdf src/test_tsdf.cpp
)
target_link_libraries (test_tsdf
  tsdf_fusion
  tsdf_cuda
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

add_executable (test_tsdf_data_capture src/test_tsdf_data_capture.cpp
  src/tsdf_fusion/topics_capture.cpp
)
target_link_libraries (test_tsdf_data_capture
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

