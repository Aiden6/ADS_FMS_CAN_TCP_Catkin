set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

aux_source_directory(. DIR_LIB_SRCS)

message(building libADS_FMS_OTA_LIB.so)

add_library(ADS_FMS_OTA_LIB SHARED ${DIR_LIB_SRCS} )


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  genmsg
  message_generation
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  
)

