cmake_minimum_required(VERSION 2.8.3)
project(mavros_notify_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    mavros_notify
)

include(ExternalProject)

ExternalProject_Add(ardupilot
    GIT_REPOSITORY https://github.com/ArduPilot/ardupilot
    GIT_TAG ArduCopter-stable
    GIT_SUBMODULES modules/mavlink modules/waf modules/uavcan
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    PREFIX ardupilot
    SOURCE_DIR ardupilot/src/ardupilot
    CONFIGURE_COMMAND SKIP_AP_EXT_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_COMPLETION_ENV=1 SKIP_AP_GIT_CHECK=1 Tools/environment_install/install-prereqs-ubuntu.sh -y
    COMMAND ./waf configure --board=sitl --no-submodule-update --notests --disable-tests --disable-scripting --enable-sfml --sitl-rgbled
    BUILD_COMMAND ./waf copter
    BUILD_IN_SOURCE TRUE
    INSTALL_COMMAND ""
    TEST_COMMAND ""
)

ExternalProject_Get_Property(ardupilot SOURCE_DIR)

file(WRITE start_sitl.bash "#!/bin/bash\n")
file(APPEND start_sitl.bash "${SOURCE_DIR}/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --no-rebuild -I0 --no-extra-ports --rgbled -m --mav10 --out=udp:127.0.0.1:14560\n")

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