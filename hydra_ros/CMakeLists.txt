cmake_minimum_required(VERSION 3.16)
project(hydra_ros)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(hydra REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(gflags REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             dynamic_reconfigure
             geometry_msgs
             hydra_msgs
             hydra_visualizer
             image_transport
             kimera_pgmo_ros
             kimera_pgmo_msgs
             rosbag
             roscpp
             std_msgs
             tf2_eigen
             tf2_ros
             visualization_msgs
             pose_graph_tools_ros
             pose_graph_tools_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/GvdVisualizer.cfg)

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  hydra_msgs
  image_transport
  kimera_pgmo_ros
  kimera_pgmo_msgs
  rosbag
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
  src/hydra_ros_pipeline.cpp
  src/active_window/reconstruction_visualizer.cpp
  src/active_window/tsdf_occupancy_publisher.cpp
  src/backend/ros_backend_publisher.cpp
  src/frontend/gvd_occupancy_publisher.cpp
  src/frontend/gvd_visualization_utilities.cpp
  src/frontend/object_visualizer.cpp
  src/frontend/places_visualizer.cpp
  src/frontend/ros_frontend_publisher.cpp
  src/input/feature_receiver.cpp
  src/input/image_receiver.cpp
  src/input/pointcloud_adaptor.cpp
  src/input/pointcloud_receiver.cpp
  src/input/ros_data_receiver.cpp
  src/input/ros_input_module.cpp
  src/input/ros_sensors.cpp
  src/loop_closure/ros_lcd_registration.cpp
  src/odometry/ros_pose_graph_tracker.cpp
  src/openset/ros_embedding_group.cpp
  src/utils/bag_reader.cpp
  src/utils/bow_subscriber.cpp
  src/utils/dsg_streaming_interface.cpp
  src/utils/external_loop_closure_subscriber.cpp
  src/utils/node_handle_factory.cpp
  src/utils/input_data_to_messages.cpp
  src/utils/node_utilities.cpp
  src/utils/occupancy_publisher.cpp
  src/utils/pose_cache.cpp
  src/utils/scene_graph_logger.cpp
  src/utils/tf_lookup.cpp
  src/visualizer/feature_color_adaptors.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC include ${catkin_INCLUDE_DIRS}
  PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES} hydra::hydra
  PRIVATE ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

find_package(semantic_inference_msgs QUIET)

if(${semantic_inference_msgs_FOUND})
  set(HYDRA_USE_SEMANTIC_INFERENCE_VALUE 1)
  target_include_directories(
    ${PROJECT_NAME} PRIVATE ${semantic_inference_msgs_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME} PRIVATE ${semantic_inference_msgs_LIBRARIES})
else()
  set(HYDRA_USE_SEMANTIC_INFERENCE_VALUE 0)
endif()
configure_file(cmake/hydra_ros_build_config.h.in hydra_ros_build_config.h)

add_executable(hydra_ros_node app/hydra_node.cpp)
target_link_libraries(hydra_ros_node ${PROJECT_NAME} ${gflags_LIBRARIES})

add_executable(scene_graph_logger_node app/scene_graph_logger_node.cpp)
target_link_libraries(scene_graph_logger_node ${PROJECT_NAME})

add_executable(reconstruct_mesh app/reconstruct_mesh.cpp)
target_link_libraries(reconstruct_mesh ${PROJECT_NAME} ${gflags_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

install(
  TARGETS ${PROJECT_NAME}
          hydra_ros_node
          scene_graph_logger_node
          reconstruct_mesh
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
  app/dsg_republisher
  app/csv_to_tf
  app/odom_to_tf
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
