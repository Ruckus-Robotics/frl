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

set(SOURCES ${SOURCES_DIR}/Tuple3d.cpp ${SOURCES_DIR}/RigidBodyTransform.cpp ${SOURCES_DIR}/AxisAngle.cpp ${SOURCES_DIR}/Point3d.cpp)
set(INCLUDES ${INCLUDES_DIR}/Tuple3d.hpp ${INCLUDES_DIR}/RigidBodyTransform.hpp ${INCLUDES_DIR}/AxisAngle.hpp ${INCLUDES_DIR}/Point3d.hpp)
set(TEST_SOURCES ${TEST_DIR}/test_main.cpp ${TEST_DIR}/Tuple3dTest.cpp ${TEST_DIR}/AxisAngleTest.cpp ${TEST_DIR}/GeometryUtilitiesTestHelper.hpp)

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

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SOURCES})
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
