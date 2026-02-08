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
#include <hydra/utils/daaam_labels.h>
#include <ianvs/node_init.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/scene_graph_layer.h>

#include <rclcpp/rclcpp.hpp>

#include "hydra_ros/utils/dsg_streaming_interface.h"

using hydra::DaaamLabels;
using hydra::FeatureMap;
using hydra::FeatureVector;
using hydra::LazyDaaamLabels;
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

enum class MergeLabelStrategy {
  SINGLE_MAX = 0,        // Current: use max-weight label only
  WEIGHTED_AVERAGE = 1,  // Compute weighted average of label features
  MULTI_LABEL_MAX = 2    // Compare all label pairs, use max similarity
};

enum class NodeDistanceAggregation {
  MAX = 0,             // Current: max similarity to any neighbor
  MIN = 1,             // Min similarity (all neighbors must be similar)
  AVERAGE = 2,         // Average similarity to neighbors
  MEDIAN = 3,          // Median similarity (robust to outliers)
  SELF_CONSISTENCY = 4 // Semantic consistency of node's own labels
};

struct Config {
  std::string src_path;
  std::string dst_path;
  DaaamLabels::Config daaam_labels;
  std::string places_layer = DsgLayers::TRAVERSABILITY;
  hydra::RoomFinderConfig room_finder;
  hydra::DsgUpdater::Config backend;
  MergeLabelStrategy merge_label_strategy = MergeLabelStrategy::SINGLE_MAX;
  NodeDistanceAggregation node_distance_aggregation = NodeDistanceAggregation::MAX;
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
  field(config.daaam_labels, "daaam_labels");
  field(config.places_layer, "places_layer");
  field(config.room_finder, "room_finder");
  field(config.backend, "backend");
  enum_field(config.merge_label_strategy,
             "merge_label_strategy",
             {{MergeLabelStrategy::SINGLE_MAX, "SINGLE_MAX"},
              {MergeLabelStrategy::WEIGHTED_AVERAGE, "WEIGHTED_AVERAGE"},
              {MergeLabelStrategy::MULTI_LABEL_MAX, "MULTI_LABEL_MAX"}});
  enum_field(config.node_distance_aggregation,
             "node_distance_aggregation",
             {{NodeDistanceAggregation::MAX, "MAX"},
              {NodeDistanceAggregation::MIN, "MIN"},
              {NodeDistanceAggregation::AVERAGE, "AVERAGE"},
              {NodeDistanceAggregation::MEDIAN, "MEDIAN"},
              {NodeDistanceAggregation::SELF_CONSISTENCY, "SELF_CONSISTENCY"}});
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
    merged_graph_regenerated_ = false;
    if (!load(config)) {
      return;
    }

    // If requested, merge nodes in the graph first.
    mergeGraph(config);

    // Compute distances on the merged graph (critical for aggregation strategies).
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
  bool merged_graph_regenerated_;
  Config prev_config_;
  std::unique_ptr<LazyDaaamLabels> labels_;
  mutable std::unordered_map<NodeId, FeatureVector> weighted_feature_cache_;

  // Compute weighted average feature for multi-label nodes
  FeatureVector getWeightedFeature(NodeId node_id,
                                   const std::map<int, float>& daaam_labels) const {
    // Check cache first
    auto cache_it = weighted_feature_cache_.find(node_id);
    if (cache_it != weighted_feature_cache_.end()) {
      return cache_it->second;
    }

    if (daaam_labels.empty()) {
      return FeatureVector();
    }

    FeatureVector result;
    float total_weight = 0.0f;
    Eigen::Index feature_dim = 0;

    for (const auto& [label, weight] : daaam_labels) {
      const auto& feature = labels_->get(label);
      if (feature.size() == 0) continue;

      if (feature_dim == 0) {
        feature_dim = feature.size();
        result = FeatureVector::Zero(feature_dim);
      }

      if (feature.size() != feature_dim) {
        LOG(WARNING) << "Feature dimension mismatch for label " << label;
        continue;
      }

      result += weight * feature;
      total_weight += weight;
    }

    if (total_weight > 0.0f) {
      result /= total_weight;
    }

    // Cache the result
    weighted_feature_cache_[node_id] = result;
    return result;
  }

  // Compute max similarity across all label pairs
  double getMultiLabelMaxScore(const std::map<int, float>& source_labels,
                               const std::map<int, float>& target_labels) const {
    double max_score = 0.0;

    for (const auto& [source_label, source_weight] : source_labels) {
      const auto& source_feature = labels_->get(source_label);
      if (source_feature.size() == 0) continue;

      for (const auto& [target_label, target_weight] : target_labels) {
        const auto& target_feature = labels_->get(target_label);
        if (target_feature.size() == 0) continue;

        const double score = DaaamLabels::getScore(source_feature, target_feature);
        max_score = std::max(max_score, score);
      }
    }

    return max_score;
  }

  // Compute semantic consistency of a node's daaam labels
  double computeSelfConsistency(const std::map<int, float>& daaam_labels) const {
    if (daaam_labels.empty()) return 0.0;
    if (daaam_labels.size() == 1) return 1.0;  // Pure node = max consistency

    // Compare all label pairs, weighted by their label weights
    double total_similarity = 0.0;
    double total_weight = 0.0;

    for (auto it1 = daaam_labels.begin(); it1 != daaam_labels.end(); ++it1) {
      for (auto it2 = std::next(it1); it2 != daaam_labels.end(); ++it2) {
        const auto& f1 = labels_->get(it1->first);
        const auto& f2 = labels_->get(it2->first);
        if (f1.size() == 0 || f2.size() == 0) continue;

        double score = DaaamLabels::getScore(f1, f2);
        double weight = it1->second * it2->second;  // Weight by label certainties
        total_similarity += score * weight;
        total_weight += weight;
      }
    }

    return total_weight > 0.0 ? total_similarity / total_weight : 0.0;
  }

  // Aggregate neighbor scores into node distance based on strategy
  double aggregateNodeDistance(const std::vector<double>& neighbor_scores,
                               NodeDistanceAggregation strategy) const {
    if (neighbor_scores.empty()) return 0.0;

    switch (strategy) {
      case NodeDistanceAggregation::MAX:
        return *std::max_element(neighbor_scores.begin(), neighbor_scores.end());

      case NodeDistanceAggregation::MIN:
        return *std::min_element(neighbor_scores.begin(), neighbor_scores.end());

      case NodeDistanceAggregation::AVERAGE: {
        double sum = std::accumulate(neighbor_scores.begin(), neighbor_scores.end(), 0.0);
        return sum / neighbor_scores.size();
      }

      case NodeDistanceAggregation::MEDIAN: {
        std::vector<double> sorted = neighbor_scores;
        std::sort(sorted.begin(), sorted.end());
        size_t mid = sorted.size() / 2;
        if (sorted.size() % 2 == 0) {
          return (sorted[mid - 1] + sorted[mid]) / 2.0;
        } else {
          return sorted[mid];
        }
      }

      case NodeDistanceAggregation::SELF_CONSISTENCY:
        // For self-consistency, we ignore neighbor scores and use node's own labels
        // This will be handled separately
        return 0.0;

      default:
        return 0.0;
    }
  }

  void mergeGraph(const Config& config) {
    // Check if backend or room_finder config changed
    // (room_finder affects traversability merging via UpdateTraversabilityFunctor)
    const bool backend_changed = !isEqual(config.backend, prev_config_.backend);
    const bool room_finder_changed = !isEqual(config.room_finder, prev_config_.room_finder);
    const bool config_changed = backend_changed || room_finder_changed;

    if (!graph_updated_ && merged_graph_ && !config_changed) {
      LOG(INFO) << "Skipping merging, no changes detected.";
      merged_graph_regenerated_ = false;
      return;
    }

    // Always set the merged graph for downstream processing.
    if (!config.backend.enable_node_merging) {
      merged_graph_ = graph_;
      merged_graph_regenerated_ = true;  // Graph reference changed
      LOG(INFO) << "Skipping merging, no node merging is disabled.";
      return;
    }

    // Apply merging by wrapping around the hydra interfaces.
    merged_graph_ = graph_->clone();
    auto merged_graph_info =
        std::make_shared<hydra::SharedDsgInfo>(hydra::SharedDsgInfo::Config{});
    merged_graph_info->graph = merged_graph_;
    hydra::DsgUpdater updater(config.backend, graph_, merged_graph_info);
    hydra::UpdateInfo::ConstPtr info(new hydra::UpdateInfo{0, nullptr, nullptr, true});
    updater.callUpdateFunctions(0, info);
    merged_graph_regenerated_ = true;

    // Log some stats.
    std::stringstream ss;
    for (const auto& [layer_name, _] : graph_->layer_names()) {
      const size_t num_prev = graph_->getLayer(layer_name).nodes().size();
      const size_t num_merged = merged_graph_->getLayer(layer_name).nodes().size();
      ss << "\n- Layer " << layer_name << ": Merged " << (num_prev - num_merged)
         << " nodes (" << num_prev << "->" << num_merged << ")";
    }
    LOG(INFO) << "Merged graph:" << ss.str();
  }

  void computeFeatures(const Config& config) {
    // Check for feature changes.
    if (!labels_ || config.daaam_labels.feature_type !=
                        prev_config_.daaam_labels.feature_type) {
      labels_ = std::make_unique<LazyDaaamLabels>(*graph_);
      weighted_feature_cache_.clear();
    } else if (!merged_graph_regenerated_ &&
               config.daaam_labels.distance_metric ==
                   prev_config_.daaam_labels.distance_metric &&
               config.merge_label_strategy == prev_config_.merge_label_strategy &&
               config.node_distance_aggregation == prev_config_.node_distance_aggregation) {
      // No merged graph changes, feature, metric, or strategy changes.
      LOG(INFO)
          << "Skipping feature computation, no merged graph, feature, metric, or strategy changes detected.";
      return;
    }

    // Clear weighted feature cache when strategy, metric, or merged graph changes
    if (merged_graph_regenerated_ ||
        config.merge_label_strategy != prev_config_.merge_label_strategy ||
        config.daaam_labels.distance_metric !=
            prev_config_.daaam_labels.distance_metric) {
      weighted_feature_cache_.clear();
      if (merged_graph_regenerated_) {
        LOG(INFO) << "Merged graph regenerated, clearing feature cache.";
      }
    }

    const std::string strategy_name =
        config.merge_label_strategy == MergeLabelStrategy::SINGLE_MAX
            ? "SINGLE_MAX"
            : config.merge_label_strategy == MergeLabelStrategy::WEIGHTED_AVERAGE
                  ? "WEIGHTED_AVERAGE"
                  : "MULTI_LABEL_MAX";
    const std::string aggregation_name =
        config.node_distance_aggregation == NodeDistanceAggregation::MAX
            ? "MAX"
            : config.node_distance_aggregation == NodeDistanceAggregation::MIN
                  ? "MIN"
                  : config.node_distance_aggregation == NodeDistanceAggregation::AVERAGE
                        ? "AVERAGE"
                        : config.node_distance_aggregation == NodeDistanceAggregation::MEDIAN
                              ? "MEDIAN"
                              : "SELF_CONSISTENCY";
    LOG(INFO) << "Updating place features and edge weights using merge strategy: "
              << strategy_name << ", aggregation: " << aggregation_name;

    DaaamLabels::setup(config.daaam_labels);
    // CRITICAL FIX: Operate on merged graph, not original graph
    const auto& layer = merged_graph_->getLayer(config.places_layer);
    graph_updated_ = true;

    size_t num_with_id = 0;
    size_t num_with_feature = 0;
    // Parse all nodes.
    for (const auto& [_, node] : layer.nodes()) {
      auto& attrs = node->attributes<TraversabilityNodeAttributes>();
      attrs.distance = 0.0;
      attrs.is_predicted = false;  // Hijack the is_predicted flag for valid nodes.

      // Check if we have an ID.
      if (attrs.daaam_labels.empty()) {
        attrs.semantic_label = 0;  // No Label attached.
        attrs.color = Color::gray();
        continue;
      }

      // Check if we have a feature.
      num_with_id++;
      attrs.semantic_label = hydra::getMaxDaaamLabel(attrs.daaam_labels).first;
      const auto& feature = labels_->get(attrs.semantic_label);
      if (feature.size() == 0) {
        attrs.color = Color::red();
        continue;
      }
      num_with_feature++;
      // NOTE(lschmid): Can add PCA or other feature visualization here.
      attrs.color = Color::green();
      attrs.is_predicted = true;
    }
    LOG(INFO) << "Parsed " << layer.nodes().size() << " place nodes, of which "
              << num_with_id << " had an assigned ID and " << num_with_feature
              << " had valid features.";

    // Compute the edge similarities.
    size_t num_invalid = 0;
    std::vector<float> distances;
    distances.reserve(layer.edges().size());
    // Map to collect neighbor scores for each node
    std::unordered_map<NodeId, std::vector<double>> node_neighbor_scores;
    for (auto& [_, edge] : layer.edges()) {
      edge.info->weighted = true;
      auto& source_attrs =
          layer.getNode(edge.source).attributes<TraversabilityNodeAttributes>();
      if (!source_attrs.is_predicted) {
        edge.info->weight = 0.0f;
        num_invalid++;
        continue;
      }

      auto& target_attrs =
          layer.getNode(edge.target).attributes<TraversabilityNodeAttributes>();
      if (!target_attrs.is_predicted) {
        edge.info->weight = 0.0f;
        num_invalid++;
        continue;
      }

      // Compute similarity based on strategy
      double score = 0.0;
      switch (config.merge_label_strategy) {
        case MergeLabelStrategy::SINGLE_MAX: {
          // Original behavior: compare max-weight labels only
          score = DaaamLabels::getScore(labels_->get(source_attrs.semantic_label),
                                           labels_->get(target_attrs.semantic_label));
          break;
        }

        case MergeLabelStrategy::WEIGHTED_AVERAGE: {
          // Compare weighted average features
          const auto source_feature =
              getWeightedFeature(edge.source, source_attrs.daaam_labels);
          const auto target_feature =
              getWeightedFeature(edge.target, target_attrs.daaam_labels);
          if (source_feature.size() > 0 && target_feature.size() > 0) {
            score = DaaamLabels::getScore(source_feature, target_feature);
          }
          break;
        }

        case MergeLabelStrategy::MULTI_LABEL_MAX: {
          // Compare all label pairs, use maximum
          score = getMultiLabelMaxScore(source_attrs.daaam_labels,
                                       target_attrs.daaam_labels);
          break;
        }
      }

      edge.info->weight = score;
      // Collect neighbor scores for later aggregation
      node_neighbor_scores[edge.source].push_back(score);
      node_neighbor_scores[edge.target].push_back(score);
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

    LOG(INFO) << "Computed scores for " << layer.edges().size()
              << " place edges, of which " << num_invalid
              << " invalid. min: " << min_score << ", max: " << max_score
              << ", mean: " << mean << ", stddev: " << stddev;

    // Aggregate neighbor scores into node distances based on strategy
    for (const auto& [node_id, node] : layer.nodes()) {
      auto& attrs = node->attributes<TraversabilityNodeAttributes>();
      if (!attrs.is_predicted) {
        attrs.distance = 0.0;
        continue;  // Skip nodes without valid features
      }

      if (config.node_distance_aggregation == NodeDistanceAggregation::SELF_CONSISTENCY) {
        // Use semantic consistency of node's own labels
        attrs.distance = computeSelfConsistency(attrs.daaam_labels);
      } else {
        // Aggregate neighbor scores
        auto it = node_neighbor_scores.find(node_id);
        if (it != node_neighbor_scores.end() && !it->second.empty()) {
          attrs.distance = aggregateNodeDistance(it->second, config.node_distance_aggregation);
        } else {
          attrs.distance = 0.0;  // No neighbors
        }
      }
    }
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
    LOG(INFO) << "Loading graph from '" << config.src_path << "'.";
    graph_ = spark_dsg::DynamicSceneGraph::load(config.src_path);
    labels_.reset();  // Invalidate labels.
    if (!graph_) {
      LOG(ERROR) << "Failed to load graph";
      return false;
    }

    // Mark nodes inactive by default.
    for (const auto& [layer_name, _] : graph_->layer_names()) {
      for (const auto& [_, node] : graph_->getLayer(layer_name).nodes()) {
        node->attributes().is_active = false;
      }
    }
    graph_updated_ = true;

    if (!graph_->hasLayer(config.places_layer)) {
      LOG(ERROR) << "Graph does not have a layer named '" << config.places_layer
                 << "', cannot cluster places.";
      return false;
    }
    LOG(INFO) << "Loaded and parsed new graph.";
    return true;
  }

  void save(const Config& config) {
    if (config.dst_path == prev_config_.dst_path) {
      LOG(INFO) << "Overwriting previous output at " << config.dst_path << ".";
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
