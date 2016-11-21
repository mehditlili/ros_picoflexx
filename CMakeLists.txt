cmake_minimum_required(VERSION 2.8.3)
project(ros_picoflexx)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-DBT_USE_DOUBLE_PRECISION)

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
add_compile_options(-std=c++11)
ELSEIF(COMPILER_SUPPORTS_CXX0X)
add_compile_options(-std=c++0x)
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  image_transport
)
## System dependencies are found with CMake's conventions
find_package(royale REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

link_directories(${royale_LIB_DIR})

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   PicoFlexxConfig.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_picoflexx
  CATKIN_DEPENDS roscpp sensor_msgs pcl_msgs pcl_ros
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${royale_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

link_directories(${PCL_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH ${royale_LIB_DIR})

add_definitions(${PCL_DEFINITIONS})

add_executable(ros_picoflexx_node src/ros_picoflexx_node.cpp)
get_target_property(ros_picoflexx_node_location ros_picoflexx_node LOCATION)
get_filename_component(ros_picoflexx_node_DIR ${ros_picoflexx_node_location} PATH)

#add_custom_command(
#    TARGET ros_picoflexx_node
#    POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libroyale.so"  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})


## Specify libraries to link a library or executable target against
 target_link_libraries(ros_picoflexx_node
   ${catkin_LIBRARIES}
   ${royale_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
 )


#############
## Install ##
#############

install(TARGETS ros_picoflexx_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
