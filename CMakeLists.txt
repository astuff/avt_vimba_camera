cmake_minimum_required(VERSION 3.5)
project(avt_vimba_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)

find_package(camera_info_manager REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# find_package(catkin REQUIRED COMPONENTS
#   camera_info_manager
#   diagnostic_updater
#   dynamic_reconfigure
#   image_transport
#   message_filters
#   roscpp
#   sensor_msgs
#   std_msgs
#   nodelet
# )

#Get architecture
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(TargetArchitecture)
target_architecture(ARCH)

#add dynamic reconfigure api
# generate_dynamic_reconfigure_options(
#   cfg/AvtVimbaCamera.cfg
# )

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS camera_info_manager diagnostic_updater dynamic_reconfigure image_transport roscpp sensor_msgs std_msgs
# )

###########
## Build ##
###########

include_directories(
  # ${catkin_INCLUDE_DIRS}
  include
)

# C++11 support
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
# elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# else()
#   message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
# endif()

function(add_dependencies_and_linkings arg)
  # add_dependencies(${arg}
  #   ${PROJECT_NAME}_gencfg
  # )

  if("${ARCH}" STREQUAL x86_64)
    target_link_libraries(${arg}
      ${catkin_LIBRARIES}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
    )
  elseif("${ARCH}" STREQUAL armv7)
    target_link_libraries(${arg}
      ${catkin_LIBRARIES}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
    )
  elseif("${ARCH}" STREQUAL armv8)
    target_link_libraries(${arg}
      ${catkin_LIBRARIES}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaCPP.so
    )
  else()
    message(FATAL_ERROR "[libvimba]: Architecture ${ARCH} not suported. Exiting...")
  endif()
endfunction(add_dependencies_and_linkings)


# --- Nodes

# Mono Camera
add_library(mono_camera_node SHARED
  src/mono_camera.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
)
ament_target_dependencies(mono_camera_node
  "rclcpp"
  "rclcpp_components"
  "image_transport"
  "camera_info_manager"
  "diagnostic_updater"
  "diagnostic_msgs"
  "std_msgs"
  "sensor_msgs"
)
rclcpp_components_register_nodes(
  mono_camera_node
  "avt_vimba_camera::MonoCameraNode"
)
add_dependencies_and_linkings(mono_camera_node)

# Trigger
add_library(trigger_node SHARED
  src/trigger.cpp
)
ament_target_dependencies(trigger_node
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)
rclcpp_components_register_nodes(
  trigger_node
  "avt_vimba_camera::TriggerNode"
)
add_dependencies_and_linkings(trigger_node)





# add_library(
#   ${PROJECT_NAME}_node SHARED
#   src/pacmod3_ros_msg_handler.cpp
#   src/pacmod3_core.cpp
#   src/pacmod3_node.cpp
# )
# target_compile_definitions(
#   ${PROJECT_NAME}_node
#   PRIVATE "COMPOSITION_BUILDING_DLL"
# )
# ament_target_dependencies(
#   ${PROJECT_NAME}_node
#   "rclcpp"
#   "rclcpp_components"
#   "rclcpp_lifecycle"
#   "can_msgs"
#   "pacmod3_msgs"
#   "std_msgs"
# )
# rclcpp_components_register_nodes(
#   ${PROJECT_NAME}_node
#   "pacmod3::PACMod3Node"
# )




# --- Executables

# ament_auto_add_executable(mono_camera_exec
#   src/nodes/mono_camera_node.cpp
# )

# Mono Camera
add_executable(
  mono_camera_exec
  src/nodes/mono_camera_node.cpp
)
ament_target_dependencies(
  mono_camera_exec
  "rclcpp"
)
target_link_libraries(
  mono_camera_exec
  mono_camera_node
)
add_dependencies_and_linkings(mono_camera_exec)

# Trigger
add_executable(
  trigger_exec
  src/nodes/trigger_node.cpp
)
ament_target_dependencies(
  mono_camera_exec
  "rclcpp"
)
target_link_libraries(
  trigger_exec
  trigger_node
)
add_dependencies_and_linkings(trigger_exec)



# Nodes
# add_executable(mono_camera_node
#   src/nodes/mono_camera_node.cpp
#   src/mono_camera.cpp
#   src/avt_vimba_camera.cpp
#   src/frame_observer.cpp
# )
# add_dependencies_and_linkings(mono_camera_node)

# add_executable(sync_node
#   src/nodes/sync_node.cpp
#   src/sync.cpp
# )
# add_dependencies_and_linkings(sync_node)

# add_executable(trigger_node
#   src/nodes/trigger_node.cpp
#   src/trigger.cpp
# )
# add_dependencies_and_linkings(trigger_node)

# Nodelets
# add_library(avt_camera_nodelets
#   src/nodes/mono_camera_nodelet.cpp
#   src/mono_camera.cpp
#   src/avt_vimba_camera.cpp
#   src/frame_observer.cpp
# )
# add_dependencies_and_linkings(avt_camera_nodelets)

#############
## Install ##
#############

if("${ARCH}" STREQUAL x86_64)
  # install(FILES
  #   ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
  #   ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
  #   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # )
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
    DESTINATION lib
  )
elseif("${ARCH}" STREQUAL armv7)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
elseif("${ARCH}" STREQUAL armv8)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaCPP.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endif()

# Install
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  config
  calibrations
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  mono_camera_node
  trigger_node
  EXPORT export_${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  mono_camera_exec
  trigger_exec
  DESTINATION lib/${PROJECT_NAME}/
)




# ament_auto_package(INSTALL_TO_SHARE
#   launch
#   # config
# )

ament_package()







## Mark executables and/or libraries for installation
# install(TARGETS
#   mono_camera_node
#   sync_node
#   trigger_node
#   avt_camera_nodelets
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   README.md
#   plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY
#   launch
#   calibrations
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# if("${ARCH}" STREQUAL x86_64)
#   install(FILES
#     ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
#     ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
#     DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   )
# elseif("${ARCH}" STREQUAL armv7)
#   install(FILES
#     ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
#     ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
#     DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   )
# elseif("${ARCH}" STREQUAL armv8)
#   install(FILES
#     ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaC.so
#     ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaCPP.so
#     DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   )
# endif()
