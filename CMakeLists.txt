cmake_minimum_required(VERSION 2.8.3)
project(ati_force_torque)

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
  iirob_filters
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

add_service_files(
    FILES
    DiagnosticVoltages.srv
    )

generate_messages(
    DEPENDENCIES
    std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS common/include ros/include
 CATKIN_DEPENDS roscpp message_runtime std_msgs std_srvs cob_generic_can geometry_msgs tf2 tf2_ros tf2_geometry_msgs iirob_filters
 DEPENDS Eigen
 LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} common/src/ForceTorqueCtrl.cpp ros/src/force_torque_sensor.cpp ros/src/force_torque_sensor_handle.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} gravity_compensation low_pass_filter moving_mean_filter threshold_filter)

add_executable(${PROJECT_NAME}_node ros/src/force_torque_sensor_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} gravity_compensation low_pass_filter moving_mean_filter threshold_filter)

add_executable(${PROJECT_NAME}_config ros/src/force_torque_sensor_config.cpp)
target_link_libraries(${PROJECT_NAME}_config ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(${PROJECT_NAME}_config ${PROJECT_NAME}_generate_messages_cpp)

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

install(DIRECTORY ros/include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ros/launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
