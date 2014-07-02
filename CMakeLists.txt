cmake_minimum_required(VERSION 2.8.3)
project(cob_forcetorque)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  cob_srvs
  visualization_msgs
  libntcan
)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS common/include
 CATKIN_DEPENDS roscpp libntcan
 DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ros/src/forcetorque.cpp common/src/ForceTorqueCtrl.cpp common/src/CanESD.cpp common/src/TimeStamp.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)