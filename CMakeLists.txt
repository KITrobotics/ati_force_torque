cmake_minimum_required(VERSION 2.8.3)
project(ati_mini_45)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
  cob_generic_can
  cmake_modules
)

find_package(Eigen REQUIRED)

add_service_files(
    DiagnosticVoltage.srv
    )

generate_messages(
    DEPENDENCIES
    std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS common/include
 CATKIN_DEPENDS roscpp message_runtime std_msgs std_srvs cob_generic_can geometry_msgs tf2 tf2_ros tf2_geometry_msgs
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

add_library(${PROJECT_NAME} common/src/ForceTorqueCtrl.cpp)

add_executable(${PROJECT_NAME}_node ros/src/fts_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_executable(${PROJECT_NAME}_config ros/src/fts_config.cpp)
target_link_libraries(${PROJECT_NAME}_config ${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_config
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ros/launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
