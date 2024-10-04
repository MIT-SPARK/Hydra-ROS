cmake_minimum_required(VERSION 3.16)
project(hydra_visualizer)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(config_utilities REQUIRED)
find_package(gflags REQUIRED)
find_package(spark_dsg REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             hydra_msgs
             kimera_pgmo_msgs
             roscpp
             std_msgs
             tf2_eigen
             tf2_ros
             visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/DynamicLayerVisualizer.cfg cfg/LayerVisualizer.cfg cfg/Visualizer.cfg
)

catkin_package(
  CATKIN_DEPENDS
  dynamic_reconfigure
  hydra_msgs
  kimera_pgmo_msgs
  roscpp
  std_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
  DEPENDS config_utilities spark_dsg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/dsg_visualizer.cpp
  src/scene_graph_renderer.cpp
  src/color/colormap_utilities.cpp
  src/color/graph_color_adaptors.cpp
  src/color/mesh_color_adaptor.cpp
  src/io/graph_file_wrapper.cpp
  src/io/graph_ros_wrapper.cpp
  src/io/graph_zmq_wrapper.cpp
  src/groundtruth/region_publisher.cpp
  src/groundtruth/bounding_box_publisher.cpp
  src/plugins/basis_point_plugin.cpp
  src/plugins/khronos_object_plugin.cpp
  src/plugins/mesh_plugin.cpp
  src/plugins/footprint_plugin.cpp
  src/utils/config_manager.cpp
  src/utils/ear_clipping.cpp
  src/utils/label_adaptors.cpp
  src/utils/marker_group_pub.cpp
  src/utils/marker_tracker.cpp
  src/utils/node_filter.cpp
  src/utils/polygon_utilities.cpp
  src/utils/visualizer_utilities.cpp
  src/utils/visualizer_types.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} config_utilities::config_utilities
                         spark_dsg::spark_dsg
)
add_dependencies(
  ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(hydra_visualizer_node app/hydra_visualizer_node.cpp)
target_link_libraries(hydra_visualizer_node ${PROJECT_NAME} gflags)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME} tests/main.cpp tests/test_ear_clipping.cpp)
  target_link_libraries(
    test_${PROJECT_NAME}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    gflags
  )
endif()

install(
  TARGETS ${PROJECT_NAME} hydra_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
