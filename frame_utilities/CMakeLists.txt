cmake_minimum_required(VERSION 2.8.3)
project(frame_utilities)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf geometry_utilities)

# catkin_python_setup()

set(SOURCES_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDES_DIR ${PROJECT_SOURCE_DIR}/include)
set(TEST_DIR ${PROJECT_SOURCE_DIR}/test)

set(SOURCES ${SOURCES_DIR}/ReferenceFrame.cpp ${SOURCES_DIR}/FramePoint.cpp ${SOURCES_DIR}/FrameTuple.cpp)
set(INCLUDES ${INCLUDES_DIR}/ReferenceFrame.hpp ${INCLUDES_DIR}/FramePoint.hpp ${INCLUDES_DIR}/FrameTuple.hpp ${INCLUDES_DIR}/ReferenceFrameHolder.hpp)
set(TEST_SOURCES ${TEST_DIR}/test_main.cpp ${TEST_DIR}/ReferenceFrameTest.cpp ${TEST_DIR}/ReferenceFrameTestHelper.hpp)

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
 INCLUDE_DIRS ${INCLUDES_DIR}
#  LIBRARIES frame_utilities
 CATKIN_DEPENDS geometry_msgs tf geometry_utilities
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
message(${geometry_utilities_INCLUDE_DIRS})
include_directories(${INCLUDES_DIR} ${geometry_utilities_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${SOURCES} 
  ${INCLUDES}
)

## Declare a cpp executable
# add_executable(frame_utilities_node src/frame_utilities_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(frame_utilities_node frame_utilities_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

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
# install(TARGETS frame_utilities frame_utilities_node
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
catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SOURCES})
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
