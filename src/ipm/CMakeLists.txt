cmake_minimum_required(VERSION 3.0.2)
project(ipm)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/camera_extrinsic_param.cfg
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
#  INCLUDE_DIRS include
#  LIBRARIES ipm
   CATKIN_DEPENDS dynamic_reconfigure roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include  
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ipm_node
  src/ipm_node.cpp
  src/ipm.cpp
)

add_dependencies(ipm_node ${PROJECT_NAME}_gencfg)

target_link_libraries(ipm_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
