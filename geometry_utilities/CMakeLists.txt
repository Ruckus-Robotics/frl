cmake_minimum_required(VERSION 2.8.3)
project(geometry_utilities)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS tf2)
find_package(Eigen3 REQUIRED)

set(SOURCES_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDES_DIR ${PROJECT_SOURCE_DIR}/include)
set(TEST_DIR ${PROJECT_SOURCE_DIR}/test)

set(SOURCES ${SOURCES_DIR}/Tuple3d.cpp ${SOURCES_DIR}/RigidBodyTransform.cpp)
set(INCLUDES ${INCLUDES_DIR}/Tuple3d.hpp ${INCLUDES_DIR}/RigidBodyTransform.hpp)
set(TEST_SOURCES ${TEST_DIR}/test_main.cpp ${TEST_DIR}/Tuple3dTest.cpp ${TEST_DIR}/GeometryUtilitiesTestHelper.hpp)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR})

## Declare a cpp library
 add_library(${PROJECT_NAME}
   ${SOURCES}
   ${INCLUDES}
 )

 catkin_package(
 INCLUDE_DIRS ${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS tf2
 DEPENDS
)

## Declare a cpp executable
# add_executable(math_utilities_node src/math_utilities_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(math_utilities_node math_utilities_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
# install(TARGETS math_utilities math_utilities_node
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


# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SOURCES})
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
