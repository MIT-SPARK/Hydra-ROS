/**
 * @brief Standalone node to load and visualize a serialized deformation graph (.dgrf)
 *
 * Loads a .dgrf file, then publishes the deformation graph as rviz markers on a timer.
 * Topics published:
 *   deformation_graph_mesh_mesh (MarkerArray) — mesh-mesh edges with variance coloring
 *   deformation_graph_pose_mesh (Marker) — pose-mesh valence edges
 */
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo_ros/visualization_functions.h>

class DeformationGraphVisualizer : public rclcpp::Node {
 public:
  DeformationGraphVisualizer() : Node("deformation_graph_visualizer") {
    declare_parameter<std::string>("dgrf_path", "");
    declare_parameter<std::string>("frame_id", "world");
    declare_parameter<double>("publish_rate", 1.0);
    declare_parameter<bool>("include_temp", true);

    const auto dgrf_path = get_parameter("dgrf_path").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    const auto rate = get_parameter("publish_rate").as_double();
    const auto include_temp = get_parameter("include_temp").as_bool();

    if (dgrf_path.empty()) {
      RCLCPP_FATAL(get_logger(), "dgrf_path parameter is required");
      throw std::runtime_error("dgrf_path parameter is required");
    }

    if (!std::filesystem::exists(dgrf_path)) {
      RCLCPP_FATAL(get_logger(), "File not found: %s", dgrf_path.c_str());
      throw std::runtime_error("dgrf file not found: " + dgrf_path);
    }

    RCLCPP_INFO(get_logger(), "Loading deformation graph from: %s", dgrf_path.c_str());
    graph_ = std::make_shared<kimera_pgmo::DeformationGraph>();
    graph_->load(dgrf_path, include_temp);

    if (graph_->getValues()->empty()) {
      RCLCPP_FATAL(get_logger(), "Loaded graph has no values — file may be corrupt");
      throw std::runtime_error("Loaded deformation graph is empty");
    }

    RCLCPP_INFO(get_logger(), "Loaded: %zu factors, %zu values, %zu mesh vertices",
                graph_->getFactors()->size(),
                graph_->getValues()->size(),
                graph_->getNumVertices());

    mm_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "deformation_graph_mesh_mesh", 10);
    pm_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "deformation_graph_pose_mesh", 10);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&DeformationGraphVisualizer::publish, this));

    RCLCPP_INFO(get_logger(), "Publishing at %.1f Hz in frame '%s'",
                rate, frame_id_.c_str());
  }

 private:
  void publish() {
    const rclcpp::Time stamp = now();
    visualization_msgs::msg::MarkerArray mm_msg;
    visualization_msgs::msg::Marker pm_msg;

    kimera_pgmo::fillDeformationGraphMarkers(*graph_, stamp, mm_msg, pm_msg, frame_id_);

    if (!mm_msg.markers.empty()) {
      mm_pub_->publish(mm_msg);
    }
    if (!pm_msg.points.empty()) {
      pm_pub_->publish(pm_msg);
    }
  }

  std::shared_ptr<kimera_pgmo::DeformationGraph> graph_;
  std::string frame_id_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mm_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeformationGraphVisualizer>());
  rclcpp::shutdown();
  return 0;
}
