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

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf)

# catkin_python_setup()

set(SOURCES_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDES_DIR ${PROJECT_SOURCE_DIR}/include)
set(TEST_DIR ${PROJECT_SOURCE_DIR}/test)

set(SOURCES ${SOURCES_DIR}/ReferenceFrame.cpp ${SOURCES_DIR}/FramePoint.cpp ${SOURCES_DIR}/FrameTuple.cpp)
set(INCLUDES ${INCLUDES_DIR}/ReferenceFrame.hpp ${INCLUDES_DIR}/FramePoint.hpp ${INCLUDES_DIR}/FrameTuple.hpp ${INCLUDES_DIR}/ReferenceFrameHolder.hpp)
set(TEST_SOURCES ${TEST_DIR}/test_main.cpp ${TEST_DIR}/ReferenceFrameTest.cpp ${TEST_DIR}/FrameTupleTest.cpp ${TEST_DIR}/ReferenceFrameTestHelper.hpp ${TEST_DIR}/FrameUtilitiesTestHelper.hpp)

include_directories(${INCLUDES_DIR} ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${SOURCES} 
  ${INCLUDES}
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS ${INCLUDES_DIR}
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS geometry_msgs tf
#  DEPENDS system_lib
)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SOURCES})
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
endif()
