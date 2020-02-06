cmake_minimum_required(VERSION 2.8.3)
project(mavros_rgbled)

find_package(catkin REQUIRED COMPONENTS
  mavros
  message_generation
  pluginlib
)

add_message_files(
  FILES
    BlinkSequence.msg
    LedControl.msg
    LedControlRaw.msg
    LedState.msg
)

generate_messages()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    rgb_led
    utils
  CATKIN_DEPENDS
    mavros
    message_runtime
    pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rgb_led src/rgb_led.cpp)
add_dependencies(rgb_led ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rgb_led ${catkin_LIBRARIES})

add_library(utils src/utils.cpp)
add_dependencies(utils ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(utils ${catkin_LIBRARIES})

install(TARGETS rgb_led utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)