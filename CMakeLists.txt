cmake_minimum_required(VERSION 2.8.3)
project(hybris_monitor)

include(FindPkgConfig)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  fawkes_msgs
  roscpp
	continual_planning_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include(CheckCXXCompilerFlag)
find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-DRESDIR="${PROJECT_SOURCE_DIR}/res")
add_definitions(-DUSE_ROS -DHAVE_GLADEMM -DHAVE_GCONFMM)
pkg_check_modules(GUI REQUIRED gtkmm-3.0 glibmm-2.4 gconfmm-2.6)
#pkg_check_modules(GRAPHVIZ REQUIRED libgvc)

CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
	message(STATUS "Use C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
	message(STATUS "Use C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	message(STATUS "Use C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


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
#  INCLUDE_DIRS include
#  LIBRARIES rcll_refbox_peer
#  CATKIN_DEPENDS rcll_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${GUI_INCLUDE_DIRS}
	${GRAPHVIZ_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GUI_LIBRARIES} ${GRAPHVIZ_LIBRARIES})

## Declare a C++ executable
add_executable(hybris_monitor src/main.cpp src/hybris_monitor.cpp)
