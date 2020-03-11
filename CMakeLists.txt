cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cassie_description)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy roslib eigen_utilities cassie_common_toolbox
)

# Find RBDL
find_package(PkgConfig)
pkg_check_modules(RBDL REQUIRED rbdl)
IF (RBDL_FOUND)
  SET(RBDL_URDFReader_LIBRARIES rbdl_urdfreader)
ENDIF (RBDL_FOUND)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include gen/include gen/include/frost_expr gen/src ${RBDL_INCLUDE_DIRS}
   LIBRARIES ${PROJECT_NAME} ${RBDL_LIBRARIES}
   CATKIN_DEPENDS roscpp rospy roslib eigen_utilities cassie_common_toolbox
   DEPENDS RBDL)

###########
## Build ##
###########
include(common_flags)
option(USE_VECTORIZATION "Use vectorization" OFF)
if(USE_VECTORIZATION)
  common_add_vectorization_flags()
endif()

add_definitions(${RBDL_CFLAGS_OTHER})
include_directories(include gen/include gen/include/frost_expr gen/src ${RBDL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include(gen/sources.cmake)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SRCS} src/cassie_model.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${RBDL_LIBRARIES} ${RBDL_URDFReader_LIBRARIES})
