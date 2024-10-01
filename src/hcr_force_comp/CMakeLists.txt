cmake_minimum_required(VERSION 2.8.3)

project(lwr_ft_comp)

#SET(CMAKE_CXX_COMPILER /usr/bin/g++-4.8)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  roslib
  std_msgs
)

#set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

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
#  LIBRARIES lwr_template
  CATKIN_DEPENDS roscpp roslib geometry_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lwr_template.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/lwr_template_node.cpp)

set(INCLUDE_SHARE ~/workspace/share/include)
set(INCLUDE_FRI ~/workspace/share/FRILibrary/include)

# Changed lib from x86 to x64
#set(LIB_FRI ~/workspace/share/FRILibrary/Linux/x64/release/lib)
set(LIB_FRI ~/workspace/share/newFRILibrary/FRILibrary/Linux/x64/release/lib)
set(SOURCES_FRI ~/workspace/share/FRILibrary/src)
set(KMS_DRIVER ~/catkin_ws/src/hri_kms_40_driver/include)
set(WS_LIBS ~/catkin_ws/devel/lib)

set( sources_irml
     # Reflexxes Motion Library sources
     ${SOURCES_FRI}/TypeIRML/TypeIRML.cpp
     ${SOURCES_FRI}/TypeIRML/TypeIRMLDecision.cpp
     ${SOURCES_FRI}/TypeIRML/TypeIRMLMath.cpp
     ${SOURCES_FRI}/TypeIRML/TypeIRMLPolynomial.cpp
     ${SOURCES_FRI}/TypeIRML/TypeIRMLProfiles.cpp
)

set( sources_project
     src/mainFRITest.cpp 
     src/RunTrajectorySimple.cpp
)

include_directories(
    include
    src/hcr_force_comp
    ${catkin_INCLUDE_DIRS} 
    ${INCLUDE_SHARE} 
    ${INCLUDE_FRI} 
    ${KMS_DRIVER} 
    #${ROS}
    ${EIGEN_INCLUDE_DIRS}
)

link_directories(
    ${LIB_FRI} 
    ${WS_LIBS}
)

add_definitions( -D__LINUX__ )

add_executable(main_ft_comp ${sources_irml} ${sources_project} src/hcr_force_comp/ft_comp.cpp src/util.hpp)
#add_dependencies(mainFRITest ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_ft_comp
    #${catkin_MANUAL} 
    ${catkin_LIBRARIES}
    #-lroscpp 
    #-lroslib 
    Threads::Threads 
    -lFastResearchInterfaceLibrary 
    -lrt 
    -lhri_kms_40_driver
    Eigen3::Eigen
    )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lwr_template.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
