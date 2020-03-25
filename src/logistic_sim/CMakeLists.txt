cmake_minimum_required(VERSION 2.8.3)
project(logistic_sim)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib
  move_base_msgs
  nav_msgs
  roscpp
#  rospy
  roslib
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
add_message_files(
  FILES
  Task.msg
  TaskRequest.msg
  Token.msg
  Mission.msg
  Path.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   RobotReady.srv
   AddMissions.srv
)

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
   INCLUDE_DIRS include
   LIBRARIES Agent
   CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs
#  CATKIN_DEPENDS actionlib move_base_msgs nav_msgs roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  # src
  ${catkin_INCLUDE_DIRS}
  ../../lib
)

############
## Agents ##
############

# Declare a cpp library
add_library(Agent
  src/Agent.cpp
  # src/algorithms.cpp
  src/config.cpp
)

add_dependencies(Agent ${PROJECT_NAME}_generate_messages_cpp  ${catkin_EXPORTED_TARGETS})

# Declare a cpp library
add_library(DistrAgentLib
  src/DistrAgent.cpp
)

add_library(algorithmsLib
  src/algorithms.cpp
)

add_library(partitionLib
  src/partition.cpp
)

add_dependencies(DistrAgentLib ${PROJECT_NAME}_generate_messages_cpp  ${catkin_EXPORTED_TARGETS})
########################################################################################################################

# add_dependencies(Agent SSIPatrolAgent LogicAgent CycleAgent CycleOptAgent CFAgent  ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable 
add_executable(OnlineAgent src/OnlineAgent.cpp)
add_dependencies(OnlineAgent ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(OnlineAgent Agent DistrAgentLib algorithmsLib ${catkin_LIBRARIES})

## Declare a cpp executable 
add_executable(OnlineCentralizedAgent src/OnlineCentralizedAgent.cpp)
add_dependencies(OnlineCentralizedAgent ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(OnlineCentralizedAgent Agent DistrAgentLib algorithmsLib ${catkin_LIBRARIES})

##################
## TaskPlanners ##
##################

# Declare a cpp library
add_library(libTaskPlanner
  src/TaskPlanner.cpp
)

add_dependencies(libTaskPlanner ${PROJECT_NAME}_generate_messages_cpp  ${catkin_EXPORTED_TARGETS})

########################################################################################################################

add_executable(OnlineTaskPlanner src/OnlineTaskPlanner.cpp)
add_dependencies(OnlineTaskPlanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 ## Specify libraries to link a library or executable target against
target_link_libraries(OnlineTaskPlanner libTaskPlanner algorithmsLib partitionLib ${catkin_LIBRARIES})

add_executable(OnlineGlobalTaskPlanner src/OnlineGlobalTaskPlanner.cpp src/OnlineCentralizedTaskPlanner.cpp)
add_dependencies(OnlineGlobalTaskPlanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 ## Specify libraries to link a library or executable target against
target_link_libraries(OnlineGlobalTaskPlanner libTaskPlanner algorithmsLib partitionLib ${catkin_LIBRARIES})

add_executable(OnlineGreedyTaskPlanner src/OnlineGreedyTaskPlanner.cpp src/OnlineCentralizedTaskPlanner.cpp)
add_dependencies(OnlineGreedyTaskPlanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 ## Specify libraries to link a library or executable target against
target_link_libraries(OnlineGreedyTaskPlanner libTaskPlanner algorithmsLib partitionLib ${catkin_LIBRARIES})

########################################################################################################################


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(patrolling_sim_node patrolling_sim_generate_messages_cpp)

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
# install(TARGETS patrolling_sim patrolling_sim_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_patrolling_sim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#add_executable(Conscientious_Reactive src/Conscientious_Reactive.cpp)
#add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive.cpp)
#add_executable(Conscientious_Cognitive src/Conscientious_Cognitive.cpp)
#add_executable(Cyclic src/Cyclic.cpp)
#add_executable(MSP src/MSP.cpp)
#add_executable(GBS src/GBS.cpp)
#add_executable(SEBS src/SEBS.cpp)

