# Defining the cmake vesrion and project name
cmake_minimum_required(VERSION 2.8.3)
project(tunnel_slam)

# compiling the code with c++11
add_compile_options(-std=c++11)

# Finding the packages required (dependencies for this package)
find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# Adding the executable files for the build
add_executable(${PROJECT_NAME}_node src/tunnel_slam.cpp src/tunnel_slam_node.cpp)
add_executable(feature_association_node src/feature_association.cpp src/feature_association_node.cpp)

# linking the libraries for successful binary genertion
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(feature_association_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)