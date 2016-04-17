cmake_minimum_required(VERSION 2.8.3)
project(frl)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

set(SOURCES_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDES_DIR ${PROJECT_SOURCE_DIR}/include)
set(TEST_DIR ${PROJECT_SOURCE_DIR}/test)

set(SOURCES ${SOURCES_DIR}/RigidBodyTransform.cpp
			${SOURCES_DIR}/Point3d.cpp 
			${SOURCES_DIR}/Quaternion.cpp
			${SOURCES_DIR}/ReferenceFrame.cpp 
			${SOURCES_DIR}/FramePoint.cpp
			${SOURCES_DIR}/FrameVector.cpp)

set(INCLUDES ${INCLUDES_DIR}/${PROJECT_NAME}/geometry_utilities/RigidBodyTransform.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/geometry_utilities/Point3d.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/geometry_utilities/Quaternion.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/frame_utilities/ReferenceFrame.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/frame_utilities/FramePoint.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/frame_utilities/ReferenceFrameHolder.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/frame_utilities/FrameVector.hpp)

set(TEST_SOURCES ${TEST_DIR}/test_main.cpp
				 ${TEST_DIR}/RigidBodyTransformTest.cpp
				 ${TEST_DIR}/GeometryUtilitiesTestHelper.hpp
				 ${TEST_DIR}/ReferenceFrameTest.cpp
				 ${TEST_DIR}/ReferenceFrameTestHelper.hpp 
				 ${TEST_DIR}/FrameUtilitiesTestHelper.hpp
				 ${TEST_DIR}/Point3dTest.cpp
				 ${TEST_DIR}/QuaternionTest.cpp
				 ${TEST_DIR}/FramePointTest.cpp
				 ${TEST_DIR}/FrameVectorTest.cpp)


include_directories(${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR})

## Declare a cpp library
 add_library(${PROJECT_NAME}
   ${SOURCES}
   ${INCLUDES}
 )

 catkin_package(
 INCLUDE_DIRS ${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME}
# CATKIN_DEPENDS
 DEPENDS Eigen3
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SOURCES})
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
endif()
