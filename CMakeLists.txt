cmake_minimum_required(VERSION 2.8.3)
project(videostream)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  OpenCV
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES videostream
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} /usr/include/flycapture 
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/videostream.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ros_videostream src/ros_videostream.cpp)
add_executable(displayimg src/displayimg.cpp)
add_executable(capturevideo src/capturevideo.cpp)
add_executable(canny src/canny.cpp)
add_executable(HSV_reconfigure src/HSV_reconfigure.cpp)
add_executable(object_detect src/object_detect.cpp)
add_executable(flycapture src/flycapture.cpp)
add_executable(ros_flycapture src/ros_flycapture.cpp)
add_executable(flycapture_HSV_reconfugure src/flycapture_HSV_reconfugure.cpp)
add_executable(eyes_HSV_reconfigure src/eyes_HSV_reconfigure.cpp)
add_executable(flycapture_findball src/flycapture_findball.cpp)
add_executable(fps src/fps.cpp)
add_executable(takePhoto src/takePhoto.cpp)
add_executable(takePhoto2 src/takePhoto2.cpp)
add_executable(findcontour2020 src/findcontour2020.cpp)
add_executable(frameRate src/frameRate.cpp)
add_executable(frameRate_right src/frameRate_right.cpp)
add_executable(flycapture_image_processing src/flycapture_image_processing.cpp)
add_executable(thread_frameRate src/thread_frameRate.cpp)
add_executable(frameRate_left src/frameRate_left.cpp)
add_executable(frameRate_oop src/frameRate_oop.cpp)
add_executable(frameRate_eyes src/frameRate_eyes.cpp)
add_executable(frameRate_eyes_timer src/frameRate_eyes_timer.cpp)
add_executable(eye_image_process src/eye_image_process.cpp)
add_executable(image_process_left src/image_process_left.cpp)
add_executable(image_process_right src/image_process_right.cpp)
add_executable(positioning src/positioning.cpp)
add_executable(sampling_timer src/sampling_timer.cpp)
add_executable(image_process_test src/image_process_test.cpp)
add_executable(thread_oop_test src/thread_oop_test.cpp)
add_executable(thread_left src/thread_left.cpp)
add_executable(thread_right src/thread_right.cpp)
add_executable(stereo src/stereo.cpp)
add_executable(stereo_oop src/stereo_oop.cpp)
add_executable(thread_left20191125 src/thread_left20191125.cpp)
add_executable(thread_right20191125 src/thread_right20191125.cpp)
add_executable(thread_oop20191126 src/thread_oop20191126.cpp)
add_executable(thread_left20191127 src/thread_left20191127.cpp)
add_executable(thread_right20191127 src/thread_right20191127.cpp)
add_executable(thread_left20191129 src/thread_left20191129.cpp)
add_executable(thread_right20191129 src/thread_right20191129.cpp)
add_executable(thread_left20191203 src/thread_left20191203.cpp)
add_executable(thread_right20191203 src/thread_right20191203.cpp)
add_executable(thread_left20191209 src/thread_left20191209.cpp)
add_executable(thread_right20191209 src/thread_right20191209.cpp)
#add_executable(record_video src/record_video.cpp)
add_executable(thread_left20200102 src/thread_left20200102.cpp)
add_executable(thread_right20200102 src/thread_right20200102.cpp)
add_executable(thread_left20200211 src/thread_left20200211.cpp)
add_executable(thread_right20200211 src/thread_right20200211.cpp)
add_executable(thread_left20200218 src/thread_left20200218.cpp)
add_executable(thread_right20200218 src/thread_right20200218.cpp)
add_executable(thread_right20200225 src/thread_right20200225.cpp)
add_executable(thread_left20200302 src/thread_left20200302.cpp)
add_executable(thread_right20200302 src/thread_right20200302.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(displayimg
    ${OpenCV_LIBS}
)
target_link_libraries(capturevideo
    ${OpenCV_LIBS}
)
target_link_libraries(ros_videostream
    ${OpenCV_LIBS} 
    ${catkin_LIBRARIES}
)
target_link_libraries(canny
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
target_link_libraries(HSV_reconfigure
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
target_link_libraries(object_detect
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
target_link_libraries(stereo
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
target_link_libraries(stereo_oop
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)

target_link_libraries(flycapture ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(ros_flycapture ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(flycapture_HSV_reconfugure ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(eyes_HSV_reconfigure ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(flycapture_findball ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(fps ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(takePhoto ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(takePhoto2 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(findcontour2020 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(frameRate ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(frameRate_right ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(flycapture_image_processing ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_frameRate ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(frameRate_left ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(frameRate_oop ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(frameRate_eyes ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(frameRate_eyes_timer ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(eye_image_process ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(image_process_left ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(image_process_right ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(positioning ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(image_process_test ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(sampling_timer ${OpenCV_LIBS} ${catkin_LIBRARIES})

target_link_libraries(thread_oop_test ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left20191125 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20191125 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_oop20191126 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left20191127 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20191127 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left20191129 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20191129 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left20191203 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20191203 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left20191209 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20191209 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left20200102 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20200102 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left20200211 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20200211 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left20200218 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20200218 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20200225 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_left20200302 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

target_link_libraries(thread_right20200302 ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

#target_link_libraries(record_video ${OpenCV_LIBS} ${catkin_LIBRARIES} -L../../lib -lflycapture${D} ${FC2_DEPS})

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_videostream.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
