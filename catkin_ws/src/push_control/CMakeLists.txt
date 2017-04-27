cmake_minimum_required(VERSION 2.8.3)
project(push_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  tf
)
#`pkg-config --cflags --libs protobuf`
## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)
#find_package(dl)
#find_package(pthread)
#find_package(gurobi_c++)
#find_package(gurobi65)
# hack
set(GUROBI_HOME "$ENV{HOME}/software/gurobi651/linux64")
set(GUROBI_HOME2 "$ENV{HOME}/gurobi651/linux64")
set(GUROBI_INCLUDE_DIR "${GUROBI_HOME}/include;${GUROBI_HOME2}/include")
set(EIGEN_INCLUDE_DIR "/usr/include/eigen3")
find_library( GUROBI_LIBRARY 
              NAMES gurobi65
              PATHS "${GUROBI_HOME}/lib" 
                    "${GUROBI_HOME2}/lib"
              )

find_library( GUROBI_CXX_LIBRARY 
              NAMES gurobi_c++
              PATHS "${GUROBI_HOME}/lib" 
                    "${GUROBI_HOME2}/lib"
              )
              # 

#add_library(libjsoncpp.a)
find_package(Protobuf REQUIRED)

set(GUROBI_LIBRARIES "${GUROBI_CXX_LIBRARY};${GUROBI_LIBRARY}" )

MESSAGE( STATUS "GUROBI_LIBRARIES: " ${GUROBI_LIBRARIES} )

#/home/mcube10/gurobi651/linux64/lib
#/home/mcube10/gurobi651/linux64/include/gurobi_c++.h


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files(
  #FILES
  #GetObjectPointCloud.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_capture
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
  src/egm
  src/PracticalSocket
  ${PROTOBUF_INCLUDE_DIRS}
)

#protoc --cpp_out=. egm.proto  # better generate at compile time
#PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS src/egm/egm.proto)

## Declare a cpp library
# add_library(image_capture
#   src/${PROJECT_NAME}/image_capture.cpp
# )

## Declare a cpp executable
# add_executable(push_control src/ABBRobot.cpp src/OptProgram.cpp  src/helper.cpp src/pushing.cpp src/main.cpp src/PracticalSocket/PracticalSocket.cpp src/egm/egm.pb.cc)
#~ add_executable(push_control src_QS/main.cpp src_QS/ABBRobot.cpp src_QS/OptProgram.cpp  src_QS/helper.cpp src_QS/Push.cpp src_QS/PracticalSocket/PracticalSocket.cpp src_QS/egm/egm.pb.cc)
add_executable(push_control src_ISRR/main.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(image_capture_node image_capture_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(push_control
  ${catkin_LIBRARIES}
  ${GUROBI_LIBRARIES}
  dl
  pthread
  ${PROTOBUF_LIBRARIES}
  libjsoncpp.a
)

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
# install(TARGETS image_capture image_capture_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_image_capture.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
