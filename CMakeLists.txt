cmake_minimum_required (VERSION 2.8.12)
project (volleybot)
SET(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include -I/usr/local/include")
SET(CMAKE_BUILD_TYPE "Release")

# set common dependencies
# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 QUIET)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# # - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# - SAI2-SIMULATION
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-MODEL
FIND_PACKAGE(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# - SAI2-GRAPHICS
FIND_PACKAGE(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - SAI2-URDF
find_package(SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})
add_definitions(${SAI2-URDF_DEFINITIONS})

# - glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)


set(CS225A_COMMON_LIBRARIES
	${CHAI3D_LIBARIES}
	${SAI2-COMMON_LIBRARIES}
	${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}
	)

set (VOLLEY_BINARY_DIR                  ${PROJECT_SOURCE_DIR}/bin)


# - SAI2-URDF
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${VOLLEY_BINARY_DIR})
#ADD_EXECUTABLE (controller controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz simviz_volley.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_legs simviz_volley_legs.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_balance controller_balance.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_full controller_full.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
#TARGET_LINK_LIBRARIES (controller ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_legs ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_balance ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_full ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY world_legged.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY world_simple_legged.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY mmp_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY legged_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY simple_legged_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY tennisBall.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY toro.urdf DESTINATION ${APP_RESOURCE_DIR})
