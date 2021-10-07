cmake_minimum_required(VERSION 2.8.3)
project(mavros_notify_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    mavros_notify
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(blinking src/blinking.cpp)
add_dependencies(blinking ${catkin_EXPORTED_TARGETS})
target_link_libraries(blinking ${catkin_LIBRARIES})

install(DIRECTORY models worlds param launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS start_sitl.bash prerequests.bash
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)