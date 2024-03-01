cmake_minimum_required(VERSION 3.16)
project(hydra_ros)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(hydra REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             dynamic_reconfigure
             geometry_msgs
             hydra_msgs
             image_transport
             kimera_pgmo
             roscpp
             std_msgs
             tf2_eigen
             tf2_ros
             visualization_msgs
             pose_graph_tools_ros
             pose_graph_tools_msgs
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
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  hydra_msgs
  image_transport
  kimera_pgmo
  roscpp
  std_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
  pose_graph_tools_ros
  pose_graph_tools_msgs
  DEPENDS hydra PCL OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/backend/ros_backend.cpp
  src/backend/ros_backend_publisher.cpp
  src/common/hydra_ros_pipeline.cpp
  src/frontend/ros_frontend_publisher.cpp
  src/loop_closure/ros_lcd_registration.cpp
  src/reconstruction/data_receiver.cpp
  src/reconstruction/image_receiver.cpp
  src/reconstruction/pointcloud_adaptor.cpp
  src/reconstruction/pointcloud_receiver.cpp
  src/reconstruction/ros_reconstruction.cpp
  src/reconstruction/ros_reconstruction_config.cpp
  src/reconstruction/ros_sensors.cpp
  src/utils/dsg_streaming_interface.cpp
  src/utils/lookup_tf.cpp
  src/utils/node_utilities.cpp
  src/utils/ear_clipping.cpp
  src/visualizer/basis_point_plugin.cpp
  src/visualizer/colormap_utilities.cpp
  src/visualizer/config_manager.cpp
  src/visualizer/dynamic_scene_graph_visualizer.cpp
  src/visualizer/gt_region_plugin.cpp
  src/visualizer/gvd_visualization_utilities.cpp
  src/visualizer/footprint_plugin.cpp
  src/visualizer/mesh_plugin.cpp
  src/visualizer/object_visualizer.cpp
  src/visualizer/places_visualizer.cpp
  src/visualizer/hydra_visualizer.cpp
  src/visualizer/polygon_utilities.cpp
  src/visualizer/region_plugin.cpp
  src/visualizer/reconstruction_visualizer.cpp
  src/visualizer/visualizer_plugins.cpp
  src/visualizer/visualizer_utilities.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC include ${catkin_INCLUDE_DIRS}
  PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES} hydra::hydra
  PRIVATE ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(dsg_optimizer_node src/nodes/dsg_optimizer_node.cpp)
target_link_libraries(dsg_optimizer_node ${PROJECT_NAME})

add_executable(hydra_reconstruction_node src/nodes/hydra_reconstruction_node.cpp)
target_link_libraries(hydra_reconstruction_node ${PROJECT_NAME})

add_executable(hydra_ros_node src/nodes/hydra_node.cpp)
target_link_libraries(hydra_ros_node ${PROJECT_NAME})

add_executable(hydra_visualizer_node src/nodes/hydra_visualizer_node.cpp)
target_link_libraries(hydra_visualizer_node ${PROJECT_NAME})

add_executable(mesh_publisher_node src/nodes/mesh_publisher_node.cpp)
target_link_libraries(mesh_publisher_node ${PROJECT_NAME})

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
          hydra_reconstruction_node
          hydra_ros_node
          hydra_visualizer_node
          mesh_publisher_node
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
