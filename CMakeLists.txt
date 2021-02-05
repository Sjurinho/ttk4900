# Defining the cmake vesrion and project name
cmake_minimum_required(VERSION 3.8.0)
project(tunnel_slam)

# compiling the code with c++11
add_compile_options(-std=c++11)

# Finding the packages required (dependencies for this package)
find_package(PCL 1.8 REQUIRED)
find_package(GTSAM 4.0.0 REQUIRED COMPONENTS
  geometry
  nonlinear
  slam 
)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

# Declaring the dependencies for packages that depend on this package.
catkin_package(
  INCLUDE_DIRS include/tunnel_slam
  CATKIN_DEPENDS roscpp rospy std_msgs
)
#message(STATUS "${PCL_INCLUDE_DIRS}=${${PCL_INCLUDE_DIRS}}")
# Add the given directories to those the compiler uses to search for include files. 
# So nothing but giving the location of the include files
include_directories(
  include/tunnel_slam
  ${catkin_INCLUDE_DIRS}
  ${PCL_INLCUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${GTSAM_LIBRARY_DIR})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${GTSAM_DEFINITIONS})

# Adding the executable files for the build
add_executable(${PROJECT_NAME}_node src/tunnel_slam.cpp src/tunnel_slam_node.cpp)
add_executable(feature_association_node src/feature_association.cpp src/feature_association_node.cpp)
add_executable(graph_node src/graph.cpp src/graph_node.cpp)

# linking the libraries for successful binary genertion
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(feature_association_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(graph_node
  gtsam
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)