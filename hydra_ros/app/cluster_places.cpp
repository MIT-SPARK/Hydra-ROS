/**
 * Offline Script to load a scene graph, cluster the places based on their similarity
 * labels, and visualize the result.
 */

#include <config_utilities/config.h>
#include <config_utilities/dynamic_config.h>
#include <config_utilities/external_registry.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/settings.h>
#include <config_utilities/types/enum.h>
#include <config_utilities_ros/ros_dynamic_config_server.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra/backend/dsg_updater.h>
#include <hydra/openset/embedding_distances.h>
#include <hydra/rooms/room_finder.h>
#include <hydra/utils/cognition_labels.h>
#include <ianvs/node_init.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/scene_graph_layer.h>

#include <rclcpp/rclcpp.hpp>

#include "hydra_ros/utils/dsg_streaming_interface.h"

using hydra::CognitionLabels;
using hydra::FeatureMap;
using hydra::FeatureVector;
using hydra::LazyCognitionLabels;
using spark_dsg::Color;
using spark_dsg::DsgLayers;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraphLayer;
using spark_dsg::TraversabilityNodeAttributes;

struct NodeConfig {
  std::vector<std::string> external_library_paths;
  bool interactive = true;
};

struct Config {
  std::string src_path;
  std::string dst_path;
  CognitionLabels::Config cognition_labels;
  std::string places_layer = DsgLayers::TRAVERSABILITY;
  hydra::RoomFinderConfig room_finder;
  hydra::DsgUpdater::Config backend;
};

void declare_config(NodeConfig& config) {
  using namespace config;
  name("ClusterPlaces::NodeConfig");
  field(config.external_library_paths, "external_library_paths");
  field(config.interactive, "interactive");
}

void declare_config(Config& config) {
  using namespace config;
  name("ClusterPlaces::Config");
  field(config.src_path, "src_path");
  field(config.dst_path, "dst_path");
  field(config.cognition_labels, "cognition_labels");
  field(config.places_layer, "places_layer");
  field(config.room_finder, "room_finder");
  field(config.backend, "backend");
}

template <typename ConfigT>
bool isEqual(const ConfigT& a, const ConfigT& b) {
  // Identical configs should produce identical renderings.
  return config::toString(a) == config::toString(b);
}

class Clusterer {
 public:
  Clusterer(ianvs::NodeHandle nh)
      : sender_(nh, "world", "publish_dsg", true, 0.0, true) {}

  void process(const Config& config) {
    LOG(INFO) << "Processing:\n" << config::toString(config);
    // Load.
    graph_updated_ = false;
    if (!load(config)) {
      return;
    }

    // If requested, merge nodes in the graph.
    mergeGraph(config);

    // If needed, compute distances for place nodes.
    computeFeatures(config);

    // Cluster places.
    cluster(config);

    // Publish.
    sender_.sendGraph(*merged_graph_, rclcpp::Time(0));
    LOG(INFO) << "Published clustered graph.";

    // Save.
    save(config);
    prev_config_ = config;
  }

 private:
  const hydra::DsgSender sender_;
  DynamicSceneGraph::Ptr graph_;
  bool graph_updated_;
  DynamicSceneGraph::Ptr merged_graph_;
  Config prev_config_;
  std::unique_ptr<LazyCognitionLabels> labels_;

  void mergeGraph(const Config& config) {
    if (!graph_updated_ && merged_graph_) {
      return;
    }
    // Always set the merged graph for downstream processing.
    if (!config.backend.enable_node_merging ||
        isEqual(config.backend, prev_config_.backend)) {
      merged_graph_ = graph_;
      return;
    }

    // Apply merging by wrapping around the hydra interfaces.
    merged_graph_ = graph_->clone();
    auto merged_graph_info =
        std::make_shared<hydra::SharedDsgInfo>(hydra::SharedDsgInfo::Config{});
    merged_graph_info->graph = merged_graph_;
    hydra::DsgUpdater updater(config.backend, graph_, merged_graph_info);
    hydra::UpdateInfo::ConstPtr info(new hydra::UpdateInfo{0});
    updater.callUpdateFunctions(0, info);

    // Log some stats.
    std::stringstream ss;
    for (const auto& [layer_id, layer] : merged_graph_->layers()) {
      const size_t num_prev = graph_->getLayer(layer_id).nodes().size();
      const size_t num_merged = layer->nodes().size();
      ss << "\n- Layer " << layer_id << ": Merged " << (num_prev - num_merged)
         << " nodes (" << num_prev << "->" << num_merged << ")";
    }
    LOG(INFO) << "Merged graph:" << ss.str();
  }

  void computeFeatures(const Config& config) {
    // Check for feature changes.
    if (!labels_ || config.cognition_labels.feature_type !=
                        prev_config_.cognition_labels.feature_type) {
      labels_ = std::make_unique<LazyCognitionLabels>(*merged_graph_);
    } else if (config.cognition_labels.distance_metric ==
               prev_config_.cognition_labels.distance_metric) {
      // No feature or metric changes.
      return;
    }

    CognitionLabels::setup(config.cognition_labels);
    const auto& layer = merged_graph_->getLayer(config.places_layer);

    size_t num_with_id = 0;
    size_t num_with_feature = 0;
    // Parse all nodes.
    for (const auto& [_, node] : layer.nodes()) {
      auto& attrs = node->attributes<TraversabilityNodeAttributes>();
      attrs.distance = 0.0;
      attrs.is_active = false;  // Hijack the active flag for valid nodes.

      // Check if we have an ID.
      if (attrs.cognition_labels.empty()) {
        attrs.semantic_label = 0;  // No Label attached.
        attrs.color = Color::gray();
        continue;
      }

      // Check if we have a feature.
      num_with_id++;
      attrs.semantic_label = hydra::getMaxCognitionLabel(attrs.cognition_labels).first;
      const auto& feature = labels_->get(attrs.semantic_label);
      if (feature.size() == 0) {
        attrs.color = Color::red();
        continue;
      }
      num_with_feature++;
      // NOTE(lschmid): Can add PCA or other feature visualization here.
      attrs.color = Color::green();
      attrs.is_active = true;
    }
    LOG(INFO) << "Parsed " << layer.nodes().size() << " nodes, of which " << num_with_id
              << " had a assigned ID and " << num_with_feature
              << " had valid features.";

    // Compute the edge similarities.
    size_t num_invalid = 0;
    std::vector<float> distances;
    distances.reserve(layer.edges().size());
    for (auto& [_, edge] : layer.edges()) {
      edge.info->weighted = true;
      auto& source_attrs =
          layer.getNode(edge.source).attributes<TraversabilityNodeAttributes>();
      if (!source_attrs.is_active) {
        edge.info->weight = 0.0f;
        num_invalid++;
        continue;
      }

      auto& target_attrs =
          layer.getNode(edge.target).attributes<TraversabilityNodeAttributes>();
      if (!target_attrs.is_active) {
        edge.info->weight = 0.0f;
        num_invalid++;
        continue;
      }

      // Compute the similarities.
      const double score =
          CognitionLabels::getScore(labels_->get(source_attrs.semantic_label),
                                    labels_->get(target_attrs.semantic_label));
      edge.info->weight = score;
      source_attrs.distance = std::max(source_attrs.distance, score);
      target_attrs.distance = std::max(target_attrs.distance, score);
      distances.push_back(score);
    }

    // Compute statistics
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();
    double sum = 0.0;
    double sum_sq = 0.0;
    for (const auto& d : distances) {
      if (d < min_score) min_score = d;
      if (d > max_score) max_score = d;
      sum += d;
      sum_sq += d * d;
    }
    double mean = distances.empty() ? 0.0 : sum / distances.size();
    double stddev = 0.0;
    if (!distances.empty()) {
      stddev = std::sqrt(sum_sq / distances.size() - mean * mean);
    }

    LOG(INFO) << "Computed scores for " << layer.edges().size() << " edges, of which "
              << num_invalid << " invalid. min: " << min_score << ", max: " << max_score
              << ", mean: " << mean << ", stddev: " << stddev;
  }

  void cluster(const Config& config) {
    hydra::RoomFinder room_finder(config.room_finder);
    auto rooms =
        room_finder.findRooms(*merged_graph_->getLayer(config.places_layer).clone());
    // auto rooms = room_finder.findRooms(graph_->getLayer(config.places_layer));
    rewriteRooms(rooms.get(), *merged_graph_);
    room_finder.addRoomPlaceEdges(*merged_graph_, config.places_layer);
  }

  void rewriteRooms(const SceneGraphLayer* new_rooms, DynamicSceneGraph& graph) const {
    std::vector<NodeId> to_remove;
    const auto& prev_rooms = graph.getLayer(DsgLayers::ROOMS);
    for (const auto& id_node_pair : prev_rooms.nodes()) {
      to_remove.push_back(id_node_pair.first);
    }

    for (const auto node_id : to_remove) {
      graph.removeNode(node_id);
    }

    if (!new_rooms) {
      return;
    }

    for (auto&& [id, node] : new_rooms->nodes()) {
      graph.emplaceNode(DsgLayers::ROOMS, id, node->attributes().clone());
    }

    for (const auto& id_edge_pair : new_rooms->edges()) {
      const auto& edge = id_edge_pair.second;
      graph.insertEdge(edge.source, edge.target, edge.info->clone());
    }
  }

  bool load(const Config& config) {
    if (config.src_path == prev_config_.src_path && graph_) {
      return true;
    }
    graph_ = spark_dsg::DynamicSceneGraph::load(config.src_path);
    if (!graph_) {
      LOG(ERROR) << "Failed to load graph from '" << config.src_path << "'.";
      return false;
    }
    LOG(INFO) << "Loaded graph from '" << config.src_path << "'.";

    // Mark nodes inactive by default.
    for (auto& [_, layer] : graph_->layers()) {
      for (auto& [_, node] : layer->nodes()) {
        node->attributes().is_active = false;
      }
    }
    graph_updated_ = true;

    if (!graph_->hasLayer(config.places_layer)) {
      LOG(ERROR) << "Graph does not have a layer named '" << config.places_layer
                 << "', cannot cluster places.";
      return false;
    }
    return true;
  }

  void save(const Config& config) {
    if (config.dst_path == prev_config_.dst_path) {
      return;
    }
    merged_graph_->save(config.dst_path);
    LOG(INFO) << "Saved clustered graph to '" << config.dst_path << "'.";
  }
};

int main(int argc, char** argv) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();

  // Node
  [[maybe_unused]] const auto guard =
      ianvs::init_node(argc, argv, "cluster_places_node");
  auto nh = ianvs::NodeHandle::this_node("~");

  // Glog.
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  config::Settings().setLogger("glog");

  // Setup config utilities.
  const auto node_settings = config::fromContext<NodeConfig>();
  [[maybe_unused]] const auto plugins_guard =
      config::loadExternalFactories(node_settings.external_library_paths);
  LOG(INFO) << "Loaded node settings:\n" << config::toString(node_settings);

  // ================= Start processing =================
  // Load config.
  Clusterer clusterer(nh);
  const Config initial_config = config::fromContext<Config>();

  if (!node_settings.interactive) {
    clusterer.process(initial_config);
    LOG(INFO) << "Done. Exiting.";
    rclcpp::shutdown();
    return 0;
  }

  // Run interactively.
  config::DynamicConfig<Config> config("cluster_places", initial_config);
  config.setCallback([&clusterer, &config]() { clusterer.process(config.get()); });
  clusterer.process(config.get());

  // Spin.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh.as<rclcpp::node_interfaces::NodeBaseInterface>());
  const config::RosDynamicConfigServer server(nh.node());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
