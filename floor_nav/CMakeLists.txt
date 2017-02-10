cmake_minimum_required(VERSION 2.8.3)
project(floor_nav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf dynamic_reconfigure face_detect_base)

## System dependencies are found with CMake's conventions
find_package(task_manager_msgs REQUIRED)
find_package(task_manager_lib REQUIRED)
# find_package(task_manager_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
generate_dynamic_reconfigure_options(
  cfg/TaskGoTo.cfg
  cfg/TaskGoToPose.cfg
  cfg/TaskPlanTo.cfg
  cfg/TaskSetHeading.cfg
  cfg/TaskSetManual.cfg
  cfg/TaskStareAtFace.cfg
  cfg/TaskWaitForAuto.cfg
  cfg/TaskWaitForROI.cfg
  cfg/TaskWaitForFace.cfg
  cfg/TaskWander.cfg
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs   sensor_msgs   std_msgs   task_manager_msgs
# )

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
#  LIBRARIES floor_nav
  CATKIN_DEPENDS geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs task_manager_lib task_manager_msgs tf 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include ${task_manager_msgs_INCLUDE_DIRS} ${task_manager_lib_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(floor_nav
#   src/${PROJECT_NAME}/floor_nav.cpp
# )

## Declare a cpp executable
# add_executable(floor_nav_node src/floor_nav_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(floor_nav_node floor_nav_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(floor_nav_node
#   ${catkin_LIBRARIES}
# )

ADD_EXECUTABLE( task_server src/task_server.cpp src/SimTasksEnv.cpp) 
TARGET_LINK_LIBRARIES(task_server ${task_manager_lib_LIBRARIES} ${catkin_LIBRARIES} dl boost_system)
add_dependencies(task_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tasks)
# Now compile the tasks independently

ADD_LIBRARY( TaskSetHeading SHARED tasks/TaskSetHeading.cpp)
TARGET_LINK_LIBRARIES(TaskSetHeading ${catkin_LIBRARIES} dl)
add_dependencies(TaskSetHeading ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( TaskGoTo SHARED tasks/TaskGoTo.cpp)
TARGET_LINK_LIBRARIES(TaskGoTo ${catkin_LIBRARIES} dl)
add_dependencies(TaskGoTo ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( TaskGoToPose SHARED tasks/TaskGoToPose.cpp)
TARGET_LINK_LIBRARIES(TaskGoToPose ${catkin_LIBRARIES} dl)
add_dependencies(TaskGoToPose ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( TaskPlanTo SHARED tasks/TaskPlanTo.cpp)
TARGET_LINK_LIBRARIES(TaskPlanTo ${catkin_LIBRARIES} dl)
add_dependencies(TaskPlanTo ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( TaskWaitForAuto SHARED tasks/TaskWaitForAuto.cpp)
TARGET_LINK_LIBRARIES(TaskWaitForAuto ${catkin_LIBRARIES} dl)
add_dependencies(TaskWaitForAuto ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( TaskSetManual SHARED tasks/TaskSetManual.cpp)
TARGET_LINK_LIBRARIES(TaskSetManual ${catkin_LIBRARIES} dl)
add_dependencies(TaskSetManual ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( TaskStareAtFace SHARED tasks/TaskStareAtFace.cpp)
TARGET_LINK_LIBRARIES(TaskStareAtFace ${catkin_LIBRARIES} dl)
add_dependencies(TaskStareAtFace ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( TaskWaitForROI SHARED tasks/TaskWaitForROI.cpp)
TARGET_LINK_LIBRARIES(TaskWaitForROI ${catkin_LIBRARIES} dl)
add_dependencies(TaskWaitForROI ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( TaskWaitForFace SHARED tasks/TaskWaitForFace.cpp)
TARGET_LINK_LIBRARIES(TaskWaitForFace ${catkin_LIBRARIES} dl)
add_dependencies(TaskWaitForFace ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY( TaskWander SHARED tasks/TaskWander.cpp)
TARGET_LINK_LIBRARIES(TaskWander ${catkin_LIBRARIES} dl)
add_dependencies(TaskWander ${${PROJECT_NAME}_EXPORTED_TARGETS})


#add_dependencies(face_detect_base ${catkin_EXPORTED_TARGETS})
#add_message_files(DIRECTORY msg FILES roiVect.msg)
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
# install(TARGETS floor_nav floor_nav_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_floor_nav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
