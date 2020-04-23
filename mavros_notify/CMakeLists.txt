cmake_minimum_required(VERSION 2.8.3)
project(mavros_notify)

add_compile_options(-std=c++11)

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
    ${PROJECT_NAME}
    ${PROJECT_NAME}_utils
  CATKIN_DEPENDS
    mavros
    message_runtime
    pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/rgb_led.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_utils src/utils.cpp)
add_dependencies(${PROJECT_NAME}_utils ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_utils ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)