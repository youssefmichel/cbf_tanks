cmake_minimum_required(VERSION 2.8.3)

project(cbf_tanks_pkg)

#SET(CMAKE_CXX_COMPILER /usr/bin/g++-4.8)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Only worked with absolute directories ..why ??


list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic" )
#list(APPEND CMAKE_PREFIX_PATH "/home/hwadong/catkin_ws/src" )

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  std_msgs

)


find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(hierarchical_varimpedance_control)




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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib geometry_msgs std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)

#FRI Dependecies, adjust to your PATH
set(INCLUDE_FRI /home/hwadong/workspace/share/FRILibrary/include)
set(LIB_FRI /home/hwadong/workspace/share/FRILibrary/Linux/x64/lib)
set(INCLUDE_SHARE /home/hwadong/workspace/share/include)

#H.D , adjust to your PATH
set(LIB_HD /home/hwadong/Youssef/sdk-3.7.3/lib/release/lin-x86_64-gcc )
set(INCLUDE_HD /home/hwadong/Youssef/sdk-3.7.3/include )



#QP Optimization QPOASES Library , adjust to your PATH
set(INCLUDE_QP  /home/hwadong/Youssef/OptimizationLibraryC++/qpOASES/include)
set(LIB_QP /home/hwadong/Youssef/OptimizationLibraryC++/qpOASES/bin)


set( sources_kuka

     src/run_trajectory_simple.cpp
     src/utility_fri.cpp
     src/utility.cpp
     src/quatUtils.cpp
     src/cbf_tank.cpp
     src/first_order_tank.cpp
     src/min_jerk.cpp
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${INCLUDE_FRI} 
    ${INCLUDE_SHARE}
    ${INCLUDE_HD}
    ${INCLUDE_QP}
)


link_directories(
    ${LIB_FRI} 
    ${LIB_HD}
    ${LIB_QP}
)

add_library(${PROJECT_NAME} ${sources_kuka})

add_definitions( -D__LINUX__ )



#add_executable(cbftank_main  ${sources_kuka})
add_executable(var_stiff_node src/tasks/var_stiff_node.cpp)
add_executable(force_control_node src/tasks/force_control_node.cpp)
add_executable(KE_limitation_node src/tasks/KE_limitation_node.cpp)



#add_executable(robot_main src/robot_main.cpp)

#target_link_libraries(robot_main
#    ${catkin_LIBRARIES}
#    ${PROJECT_NAME}
#    )

target_link_libraries(var_stiff_node
    ${catkin_LIBRARIES}
     ${PROJECT_NAME}
    Threads::Threads
    -lFastResearchInterfaceLibrary
    -lTypeIRML
    -lrt
    -ldrd
    -ldhd
    -lusb-1.0
    -lqpOASES
    )

target_link_libraries(KE_limitation_node
    ${catkin_LIBRARIES}
     ${PROJECT_NAME}
    Threads::Threads
    -lFastResearchInterfaceLibrary
    -lTypeIRML
    -lrt
    -ldrd
    -ldhd
    -lusb-1.0
     -lqpOASES
    )

target_link_libraries(force_control_node
    ${catkin_LIBRARIES}
     ${PROJECT_NAME}
    Threads::Threads
    -lFastResearchInterfaceLibrary
    -lTypeIRML
    -lrt
    -ldrd
    -ldhd
    -lusb-1.0
     -lqpOASES
    )

