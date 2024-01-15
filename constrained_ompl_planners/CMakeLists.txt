cmake_minimum_required(VERSION 2.8.3)
project(constrained_ompl_planners)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

#set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(Torch_DIR /root/libtorch/share/cmake/Torch)

list( APPEND CMAKE_INSTALL_RPATH /root/libtorch/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)  

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ompl REQUIRED)
find_package(Torch REQUIRED)
find_package(moveit_planners_ompl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
  LIBRARIES 
    ${PROJECT_NAME} 
  INCLUDE_DIRS  #which is neede to indicate the include path
    include
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
  DEPENDS
    EIGEN3
)

include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OMPL_INCLUDE_DIRS}
  SYSTEM ${TORCH_INCLUDE_DIRS}
)

#link_directories(BEFORE ${OMPL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/cbirrt.cpp
  src/DLbirrt.cpp
  # src/DLcbirrt.cpp
  src/CLazyPRM.cpp
  src/MPNetrrt.cpp
  src/CMPNetrrt.cpp
  src/CVQMPTrrt.cpp
  src/CDistributionrrt.cpp
  src/JiamingAtlasStateSpace.cpp
  src/JiamingAtlasChart.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${OMPL_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${TORCH_LIBRARIES}
)
#ompl should be OMPL
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
