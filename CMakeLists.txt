cmake_minimum_required(VERSION 2.8.3)
project(six_point_two_eight)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  nodelet
  nodelet_topic_tools
  pcl_ros
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

# find_package(Boost REQUIRED COMPONENTS)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_action_files(FILES
  GetPointCloud2.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
)

add_compile_options(
  "-std=c++14"
  
  # Ubuntu16.04のboost1.58のrange::adaptors::filteredはForwardConceptを要求して、filesystemsはSinglePassConceptです（1.59では、filteredがSinglePassConceptを要求する形に修正されている……）。
  # とりあえず、コンセプト・チェックを外して対応とします。boostのバージョン上げたいなぁ……。
  "-DBOOST_RANGE_ENABLE_CONCEPT_ASSERT=0"
)

# PCL1.7.2は、C++14に対応していません。コンパイル・オプションを指定して対応します。
set_source_files_properties(src/point_cloud_utilities.cpp
  PROPERTIES COMPILE_FLAGS "-std=c++03 -Wno-deprecated"
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(six_point_two_eight
  src/six_point_two_eight.cpp
  src/point_cloud_utilities.cpp
  src/utilities.cpp
)

add_dependencies(six_point_two_eight
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(six_point_two_eight
  ${catkin_LIBRARIES}
)
