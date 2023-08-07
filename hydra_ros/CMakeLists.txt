cmake_minimum_required(VERSION 3.1)
project(hydra_ros)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
# TODO(nathan) find hydra independently
# TODO(nathan) think about private deps
find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             geometry_msgs
             hydra
             hydra_msgs
             image_transport
             kimera_pgmo
             pcl_ros
             pose_graph_tools
             roscpp
             std_msgs
             tf2_eigen
             tf2_ros
             visualization_msgs
             voxblox_ros
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Colormap.cfg
  cfg/DynamicLayerVisualizer.cfg
  cfg/GvdVisualizer.cfg
  cfg/LayerVisualizer.cfg
  cfg/Visualizer.cfg
)

catkin_package(
  CATKIN_DEPENDS
  dynamic_reconfigure
  geometry_msgs
  hydra
  hydra_msgs
  image_transport
  kimera_pgmo
  pcl_ros
  pose_graph_tools
  roscpp
  std_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
  voxblox_ros
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/pipeline/hydra_ros_pipeline.cpp
  src/pipeline/ros_backend.cpp
  src/pipeline/ros_backend_publisher.cpp
  src/pipeline/ros_frontend.cpp
  src/pipeline/ros_frontend_publisher.cpp
  src/pipeline/ros_lcd_registration.cpp
  src/pipeline/ros_reconstruction.cpp
  src/pipeline/ros_reconstruction_config.cpp
  src/utils/dsg_streaming_interface.cpp
  src/utils/image_color_adaptor.cpp
  src/utils/image_depth_adaptor.cpp
  src/utils/image_label_adaptor.cpp
  src/utils/image_receiver.cpp
  src/utils/image_to_pointcloud.cpp
  src/utils/mask_nodelet.cpp
  src/utils/node_utilities.cpp
  src/utils/pointcloud_adaptor.cpp
  src/visualizer/colormap_utilities.cpp
  src/visualizer/mesh_plugin.cpp
  src/visualizer/dynamic_scene_graph_visualizer.cpp
  src/visualizer/gvd_visualization_utilities.cpp
  src/visualizer/reconstruction_visualizer.cpp
  src/visualizer/visualizer_plugins.cpp
  src/visualizer/visualizer_utilities.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC include ${catkin_INCLUDE_DIRS}
  PRIVATE ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES}
  PRIVATE ${OpenCV_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(dsg_optimizer_node src/nodes/dsg_optimizer_node.cpp)
target_link_libraries(dsg_optimizer_node ${PROJECT_NAME})

add_executable(gvd_validator_node src/nodes/gvd_validator_node.cpp)
target_link_libraries(gvd_validator_node ${PROJECT_NAME})

add_executable(hydra_ros_node src/nodes/hydra_node.cpp)
target_link_libraries(hydra_ros_node ${PROJECT_NAME})

add_executable(hydra_topology_node src/nodes/hydra_topology_node.cpp)
target_link_libraries(hydra_topology_node ${PROJECT_NAME})

add_executable(hydra_visualizer_node src/nodes/hydra_visualizer_node.cpp)
target_link_libraries(hydra_visualizer_node ${PROJECT_NAME})

add_executable(rotate_tf_node src/nodes/rotate_tf_node.cpp)
target_include_directories(rotate_tf_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(rotate_tf_node PUBLIC ${catkin_LIBRARIES})

add_executable(scene_graph_logger_node src/nodes/scene_graph_logger_node.cpp)
target_link_libraries(scene_graph_logger_node ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

install(
  TARGETS ${PROJECT_NAME}
          dsg_optimizer_node
          gvd_validator_node
          hydra_ros_node
          hydra_topology_node
          hydra_visualizer_node
          rotate_tf_node
          scene_graph_logger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(
  PROGRAMS
  bin/dsg_republisher
  bin/csv_to_tf
  bin/odom_to_tf
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
