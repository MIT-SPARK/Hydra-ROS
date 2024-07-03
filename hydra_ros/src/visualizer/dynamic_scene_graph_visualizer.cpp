/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/visualizer/dynamic_scene_graph_visualizer.h"

#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/visualizer_utilities.h"

namespace hydra {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using Node = SceneGraphNode;

enum class NodeColorMode : int {
  DEFAULT = hydra_ros::LayerVisualizer_DEFAULT,
  DISTANCE = hydra_ros::LayerVisualizer_DISTANCE,
  PARENT = hydra_ros::LayerVisualizer_PARENT,
  ACTIVE = hydra_ros::LayerVisualizer_ACTIVE,
  NEED_CLEANUP = hydra_ros::LayerVisualizer_CLEANUP,
  ACTIVE_MESH = hydra_ros::LayerVisualizer_ACTIVE_MESH,
  FRONTIER = hydra_ros::LayerVisualizer_FRONTIER
};

void clearPrevMarkers(const std_msgs::Header& header,
                      const std::set<NodeId>& curr_nodes,
                      const std::string& ns,
                      std::set<NodeId>& prev_nodes,
                      MarkerArray& msg) {
  for (const auto& node : prev_nodes) {
    if (curr_nodes.count(node)) {
      continue;
    }

    Marker marker = makeDeleteMarker(header, node, ns);
    msg.markers.push_back(marker);
  }

  prev_nodes = curr_nodes;
}

DynamicSceneGraphVisualizer::DynamicSceneGraphVisualizer(const ros::NodeHandle& nh)
    : nh_(nh), need_redraw_(false), periodic_redraw_(false), visualizer_frame_("map") {
  nh_.param("visualizer_frame", visualizer_frame_, visualizer_frame_);

  std::string config_ns = "~";
  nh_.param("config_ns", config_ns, config_ns);
  config_manager_ = std::make_shared<ConfigManager>(ros::NodeHandle(config_ns));

  dsg_pub_ = nh_.advertise<MarkerArray>("dsg_markers", 1, true);
  dynamic_layers_viz_pub_ = nh_.advertise<MarkerArray>("dynamic_layers_viz", 1, true);
}

void DynamicSceneGraphVisualizer::start(bool periodic_redraw) {
  periodic_redraw_ = periodic_redraw;
  double visualizer_loop_period = 1.0e-1;
  nh_.param("visualizer_loop_period", visualizer_loop_period, visualizer_loop_period);
  visualizer_loop_timer_ =
      nh_.createWallTimer(ros::WallDuration(visualizer_loop_period),
                          &DynamicSceneGraphVisualizer::displayLoop,
                          this);
}

void DynamicSceneGraphVisualizer::reset() {
  if (scene_graph_) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = visualizer_frame_;

    MarkerArray msg;
    resetImpl(header, msg);

    if (!msg.markers.empty()) {
      dsg_pub_.publish(msg);
    }
  }

  scene_graph_.reset();
}

bool DynamicSceneGraphVisualizer::redraw() {
  if (!scene_graph_) {
    return false;
  }

  need_redraw_ |= config_manager_->hasChange();
  for (const auto& plugin : plugins_) {
    need_redraw_ |= plugin->hasChange();
  }

  if (!need_redraw_) {
    return false;
  }

  need_redraw_ = false;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = visualizer_frame_;

  MarkerArray msg;
  redrawImpl(header, msg);

  if (!msg.markers.empty()) {
    dsg_pub_.publish(msg);
  }

  config_manager_->clearChangeFlags();
  for (auto& plugin : plugins_) {
    plugin->clearChangeFlag();
  }

  return true;
}

void DynamicSceneGraphVisualizer::setGraph(const DynamicSceneGraph::Ptr& scene_graph,
                                           bool need_reset) {
  if (scene_graph == nullptr) {
    ROS_ERROR("Request to visualize invalid scene graph! Ignoring");
    return;
  }

  if (need_reset || !scene_graph_) {
    config_manager_->reset(*scene_graph);
  }

  if (need_reset) {
    reset();

    for (const auto& id : scene_graph->layer_ids) {
      prev_labels_[id] = {};
      curr_labels_[id] = {};
    }
  }

  scene_graph_ = scene_graph;
  need_redraw_ = true;
}

void DynamicSceneGraphVisualizer::setLayerColorFunction(LayerId layer,
                                                        const ColorFunction& func) {
  layer_colors_[layer] = func;
}

inline double getDynamicHue(const DynamicLayerConfig& config, char prefix) {
  // distance is measured from first relatively readable character prefix
  int color_num = (std::abs((prefix - '0')) + config.color_offset) % config.num_colors;
  return static_cast<double>(color_num) / static_cast<double>(config.num_colors);
}

Color getNodeColor(const DynamicLayerConfig& config, char prefix) {
  const double hue = getDynamicHue(config, prefix);
  return Color::fromHLS(hue, config.luminance, config.saturation);
}

Color getEdgeColor(const DynamicLayerConfig& config, char prefix) {
  const double hue = getDynamicHue(config, prefix);
  const double saturation = config.saturation * config.edge_sl_ratio;
  const double luminance = config.luminance * config.edge_sl_ratio;
  return Color::fromHLS(hue, saturation, luminance);
}

void DynamicSceneGraphVisualizer::drawDynamicLayer(const std_msgs::Header& header,
                                                   const DynamicSceneGraphLayer& layer,
                                                   const DynamicLayerConfig& config,
                                                   const VisualizerConfig& viz_config,
                                                   size_t viz_idx,
                                                   MarkerArray& msg) {
  const std::string node_ns = getDynamicNodeNamespace(layer.prefix);
  Marker nodes = makeDynamicCentroidMarkers(header,
                                            config,
                                            layer,
                                            viz_config,
                                            getNodeColor(config, layer.prefix),
                                            node_ns,
                                            viz_idx);
  addMultiMarkerIfValid(nodes, msg);

  const std::string edge_ns = getDynamicEdgeNamespace(layer.prefix);
  Marker edges = makeDynamicEdgeMarkers(header,
                                        config,
                                        layer,
                                        viz_config,
                                        getEdgeColor(config, layer.prefix),
                                        edge_ns,
                                        viz_idx);
  addMultiMarkerIfValid(edges, msg);

  if (layer.numNodes() == 0) {
    deleteLabel(header, layer.prefix, msg);
    return;
  }

  const std::string label_ns = getDynamicLabelNamespace(layer.prefix);
  Marker label =
      makeDynamicLabelMarker(header, config, layer, viz_config, label_ns, viz_idx);
  msg.markers.push_back(label);
  published_dynamic_labels_.insert(label_ns);
}

void DynamicSceneGraphVisualizer::deleteLabel(const std_msgs::Header& header,
                                              char prefix,
                                              MarkerArray& msg) {
  const std::string label_ns = getDynamicLabelNamespace(prefix);

  if (published_dynamic_labels_.count(label_ns)) {
    Marker marker = makeDeleteMarker(header, 0, label_ns);
    msg.markers.push_back(marker);
  }
  published_dynamic_labels_.erase(label_ns);
}

void DynamicSceneGraphVisualizer::deleteDynamicLayer(const std_msgs::Header& header,
                                                     char prefix,
                                                     MarkerArray& msg) {
  const std::string node_ns = getDynamicNodeNamespace(prefix);
  deleteMultiMarker(header, node_ns, msg);

  const std::string edge_ns = getDynamicEdgeNamespace(prefix);
  deleteMultiMarker(header, edge_ns, msg);

  deleteLabel(header, prefix, msg);
}

void DynamicSceneGraphVisualizer::drawDynamicLayers(const std_msgs::Header& header,
                                                    MarkerArray& msg) {
  const VisualizerConfig& viz_config = config_manager_->getVisualizerConfig();
  for (auto&& [layer_id, sublayers] : scene_graph_->dynamicLayers()) {
    const DynamicLayerConfig& config = config_manager_->getDynamicLayerConfig(layer_id);

    size_t viz_layer_idx = 0;
    for (auto&& [prefix, layer] : sublayers) {
      if (!config.visualize) {
        deleteDynamicLayer(header, prefix, msg);
        continue;
      }

      drawDynamicLayer(header, *layer, config, viz_config, viz_layer_idx, msg);
      viz_layer_idx++;
    }
  }
}

void DynamicSceneGraphVisualizer::resetImpl(const std_msgs::Header& header,
                                            MarkerArray& msg) {
  auto to_delete = published_multimarkers_;
  for (const auto& ns : to_delete) {
    deleteMultiMarker(header, ns, msg);
  }

  for (const auto& id_layer_pair : scene_graph_->layers()) {
    const LayerId layer_id = id_layer_pair.first;
    const std::string label_ns = getLayerLabelNamespace(layer_id);
    clearPrevMarkers(header, {}, label_ns, prev_labels_.at(layer_id), msg);
  }

  for (auto& label_set : prev_labels_) {
    label_set.second.clear();
  }
  for (auto& label_set : curr_labels_) {
    label_set.second.clear();
  }

  // vanilla scene graph also makes delete markers for dynamic layers, so we duplicate
  // them here (rviz checks for topic / namespace coherence)
  MarkerArray dynamic_msg = msg;
  auto to_delete_dynamic = published_dynamic_labels_;
  for (const auto& ns : to_delete_dynamic) {
    Marker marker = makeDeleteMarker(header, 0, ns);
    dynamic_msg.markers.push_back(marker);
  }

  if (!dynamic_msg.markers.empty()) {
    dynamic_layers_viz_pub_.publish(dynamic_msg);
  }

  published_dynamic_labels_.clear();

  for (const auto& plugin : plugins_) {
    plugin->reset(header, *scene_graph_);
  }
}

void DynamicSceneGraphVisualizer::redrawImpl(const std_msgs::Header& header,
                                             MarkerArray& msg) {
  // this is janky, figure out how to do this better
  for (const auto& callback : callbacks_) {
    callback(scene_graph_);
  }

  const auto& visualizer_config = config_manager_->getVisualizerConfig();
  for (auto&& [layer_id, layer] : scene_graph_->layers()) {
    const auto layer_config = config_manager_->getLayerConfig(layer_id);
    if (!layer_config) {
      continue;
    }

    if (!layer_config->visualize) {
      deleteLayer(header, *layer, msg);
    } else {
      drawLayer(header, *layer, *layer_config, msg);
    }
  }

  if (visualizer_config.draw_mesh_edges) {
    drawLayerMeshEdges(header, mesh_edge_source_layer_, mesh_edge_ns_, msg);
  }

  std::map<LayerId, LayerConfig> all_configs;
  for (const auto layer_id : scene_graph_->layer_ids) {
    all_configs[layer_id] = *CHECK_NOTNULL(config_manager_->getLayerConfig(layer_id));
  }

  MarkerArray interlayer_edge_markers =
      makeGraphEdgeMarkers(header,
                           *scene_graph_,
                           all_configs,
                           visualizer_config,
                           interlayer_edge_ns_prefix_);

  std::set<std::string> seen_edge_labels;
  for (const auto& marker : interlayer_edge_markers.markers) {
    addMultiMarkerIfValid(marker, msg);
    seen_edge_labels.insert(marker.ns);
  }

  for (const auto& source_pair : all_configs) {
    for (const auto& target_pair : all_configs) {
      if (source_pair.first == target_pair.first) {
        continue;
      }

      const std::string curr_ns = interlayer_edge_ns_prefix_ +
                                  std::to_string(source_pair.first) + "_" +
                                  std::to_string(target_pair.first);
      if (seen_edge_labels.count(curr_ns)) {
        continue;
      }

      deleteMultiMarker(header, curr_ns, msg);
    }
  }

  MarkerArray dynamic_markers;
  drawDynamicLayers(header, dynamic_markers);

  std::map<LayerId, DynamicLayerConfig> all_dynamic_configs;
  for (const auto& id_layer_pair : scene_graph_->dynamicLayers()) {
    const auto layer_id = id_layer_pair.first;
    all_dynamic_configs[layer_id] = config_manager_->getDynamicLayerConfig(layer_id);
  }

  const std::string dynamic_interlayer_edge_prefix = "dynamic_interlayer_edges_";
  MarkerArray dynamic_interlayer_edge_markers =
      makeDynamicGraphEdgeMarkers(header,
                                  *scene_graph_,
                                  all_configs,
                                  all_dynamic_configs,
                                  visualizer_config,
                                  dynamic_interlayer_edge_prefix);

  std::set<std::string> seen_dyn_edge_labels;
  for (const auto& marker : dynamic_interlayer_edge_markers.markers) {
    addMultiMarkerIfValid(marker, msg);
    seen_dyn_edge_labels.insert(marker.ns);
  }

  for (const auto& source_pair : all_configs) {
    for (const auto& target_pair : all_dynamic_configs) {
      std::string source_to_target_ns = dynamic_interlayer_edge_prefix +
                                        std::to_string(source_pair.first) + "_" +
                                        std::to_string(target_pair.first);
      if (!seen_dyn_edge_labels.count(source_to_target_ns)) {
        deleteMultiMarker(header, source_to_target_ns, msg);
      }

      std::string target_to_source_ns = dynamic_interlayer_edge_prefix +
                                        std::to_string(target_pair.first) + "_" +
                                        std::to_string(source_pair.first);
      if (!seen_dyn_edge_labels.count(target_to_source_ns)) {
        deleteMultiMarker(header, target_to_source_ns, msg);
      }
    }
  }

  if (!dynamic_markers.markers.empty()) {
    dynamic_layers_viz_pub_.publish(dynamic_markers);
  }

  for (const auto& plugin : plugins_) {
    plugin->draw(*config_manager_, header, *scene_graph_);
  }
}

void DynamicSceneGraphVisualizer::deleteMultiMarker(const std_msgs::Header& header,
                                                    const std::string& ns,
                                                    MarkerArray& msg) {
  if (!published_multimarkers_.count(ns)) {
    return;
  }

  Marker delete_marker = makeDeleteMarker(header, 0, ns);
  msg.markers.push_back(delete_marker);

  published_multimarkers_.erase(ns);
}

void DynamicSceneGraphVisualizer::addMultiMarkerIfValid(const Marker& marker,
                                                        MarkerArray& msg) {
  if (!marker.points.empty()) {
    msg.markers.push_back(marker);
    published_multimarkers_.insert(marker.ns);
    return;
  }

  deleteMultiMarker(marker.header, marker.ns, msg);
}

void DynamicSceneGraphVisualizer::displayLoop(const ros::WallTimerEvent&) {
  if (periodic_redraw_) {
    need_redraw_ = true;
  }
  redraw();
}

void DynamicSceneGraphVisualizer::deleteLayer(const std_msgs::Header& header,
                                              const SceneGraphLayer& layer,
                                              MarkerArray& msg) {
  deleteMultiMarker(header, getLayerNodeNamespace(layer.id), msg);
  deleteMultiMarker(header, getLayerEdgeNamespace(layer.id), msg);
  deleteMultiMarker(header, getLayerBboxNamespace(layer.id), msg);
  deleteMultiMarker(header, getLayerBboxEdgeNamespace(layer.id), msg);
  deleteMultiMarker(header, getLayerBoundaryNamespace(layer.id), msg);
  deleteMultiMarker(header, getLayerBoundaryEdgeNamespace(layer.id), msg);

  const std::string label_ns = getLayerLabelNamespace(layer.id);
  for (const auto& node : prev_labels_.at(layer.id)) {
    Marker marker = makeDeleteMarker(header, node, label_ns);
    msg.markers.push_back(marker);
  }
  prev_labels_.at(layer.id).clear();
}

Color getActiveColor(const SceneGraphNode& node) {
  return node.attributes().is_active ? Color(0, 255, 0) : Color();
}

Color getCleanupColor(const SceneGraphNode& node) {
  return node.attributes<Place2dNodeAttributes>().need_cleanup_splitting
             ? Color(255, 0, 0)
             : Color(0, 255, 0);
}

Color getActiveMeshColor(const SceneGraphNode& node) {
  return node.attributes<Place2dNodeAttributes>().has_active_mesh_indices
             ? Color(255, 255, 0)
             : Color(0, 0, 255);
}

Color getFrontierColor(const SceneGraphNode& node) {
  auto attrs = node.attributes<FrontierNodeAttributes>();
  if (attrs.real_place) {
    return Color();
  } else {
    if (attrs.is_predicted) {
      return Color(0, 0, 255);
    }
    if (attrs.active_frontier) {
      return Color(0, 255, 0);
    } else {
      return Color(255, 0, 0);
    }
  }
}

Color DynamicSceneGraphVisualizer::getParentColor(
    const SceneGraphNode& node) const {
  auto parent = node.getParent();
  if (!parent) {
    return Color();
  }

  return scene_graph_->getNode(*parent).attributes<SemanticNodeAttributes>().color;
}

void DynamicSceneGraphVisualizer::drawLayer(const std_msgs::Header& header,
                                            const SceneGraphLayer& layer,
                                            const LayerConfig& config,
                                            MarkerArray& msg) {
  const auto& viz_config = config_manager_->getVisualizerConfig();
  const std::string node_ns = getLayerNodeNamespace(layer.id);

  ColorFunction layer_color_func;
  auto iter = layer_colors_.find(layer.id);
  if (iter != layer_colors_.end()) {
    layer_color_func = iter->second;
  } else {
    const auto curr_mode = static_cast<NodeColorMode>(config.marker_color_mode);
    switch (curr_mode) {
      case NodeColorMode::ACTIVE:
        layer_color_func = getActiveColor;
        break;
      case NodeColorMode::ACTIVE_MESH:
        layer_color_func = getActiveMeshColor;
        break;
      case NodeColorMode::NEED_CLEANUP:
        layer_color_func = getCleanupColor;
        break;
      case NodeColorMode::FRONTIER:
        layer_color_func = getFrontierColor;
        break;
      case NodeColorMode::DISTANCE:
        layer_color_func = [&](const SceneGraphNode& node) -> Color {
          try {
            return getDistanceColor(
                viz_config,
                config_manager_->getColormapConfig("places_colormap"),
                node.attributes<PlaceNodeAttributes>().distance);
          } catch (const std::bad_cast&) {
            return Color();
          }
        };
        break;
      case NodeColorMode::PARENT:
        layer_color_func = [this](const auto& node) { return getParentColor(node); };
        break;
      case NodeColorMode::DEFAULT:
      default:
        layer_color_func = [](const SceneGraphNode& node) -> Color {
          try {
            return node.attributes<SemanticNodeAttributes>().color;
          } catch (const std::bad_cast&) {
            return Color();
          }
        };
        break;
    }
  }

  if (config.draw_frontier_ellipse) {
    std::vector<Marker> ellipsoids = makeEllipsoidMarkers(
        header, config, layer, viz_config, "frontier_ns", layer_color_func);
    for (auto e : ellipsoids) {
      msg.markers.push_back(e);
    }

    auto nodes = makePlaceCentroidMarkers(
        header, config, layer, viz_config, node_ns, layer_color_func);
    addMultiMarkerIfValid(nodes, msg);
  } else {
    auto nodes = makeCentroidMarkers(
        header, config, layer, viz_config, node_ns, layer_color_func);
    addMultiMarkerIfValid(nodes, msg);
  }

  const std::string edge_ns = getLayerEdgeNamespace(layer.id);
  Marker edges;
  if (config.color_edges_by_weight) {
    edges = makeLayerEdgeMarkers(header,
                                 config,
                                 layer,
                                 viz_config,
                                 config_manager_->getColormapConfig("places_colormap"),
                                 edge_ns);
  } else {
    edges = makeLayerEdgeMarkers(
        header, config, layer, viz_config, Color(), edge_ns);
  }
  addMultiMarkerIfValid(edges, msg);

  const std::string label_ns = getLayerLabelNamespace(layer.id);

  curr_labels_.at(layer.id).clear();
  for (const auto& id_node_pair : layer.nodes()) {
    const Node& node = *id_node_pair.second;

    if (config.use_label) {
      Marker label = makeTextMarker(header, config, node, viz_config, label_ns);
      msg.markers.push_back(label);
      curr_labels_.at(layer.id).insert(node.id);
    }
  }

  if (config.use_collapsed_label) {
    for (const auto& id_node_pair : layer.nodes()) {
      const Node& node = *id_node_pair.second;

      Marker label = makeTextMarkerNoHeight(header, config, node, viz_config, label_ns);
      msg.markers.push_back(label);
      curr_labels_.at(layer.id).insert(node.id);
    }
  }

  const std::string bbox_ns = getLayerBboxNamespace(layer.id);
  const std::string bbox_edge_ns = getLayerBboxEdgeNamespace(layer.id);
  if (config.use_bounding_box) {
    try {
      Marker bbox = makeLayerWireframeBoundingBoxes(
          header, config, layer, viz_config, bbox_ns, layer_color_func);
      addMultiMarkerIfValid(bbox, msg);

      if (config.collapse_bounding_box) {
        Marker bbox_edges = makeEdgesToBoundingBoxes(
            header, config, layer, viz_config, bbox_edge_ns, layer_color_func);
        addMultiMarkerIfValid(bbox_edges, msg);
      } else {
        deleteMultiMarker(header, bbox_edge_ns, msg);
      }
    } catch (const std::bad_cast&) {
      // TODO(nathan) consider warning
    }
  } else {
    deleteMultiMarker(header, bbox_ns, msg);
    deleteMultiMarker(header, bbox_edge_ns, msg);
  }

  const std::string boundary_ns = getLayerBoundaryNamespace(layer.id);
  const std::string boundary_edge_ns = getLayerBoundaryEdgeNamespace(layer.id);
  if (config.draw_boundaries) {
    try {
      Marker boundary =
          makeLayerPolygonBoundaries(header, config, layer, viz_config, boundary_ns);
      addMultiMarkerIfValid(boundary, msg);

      if (config.collapse_boundary) {
        Marker boundary_edges =
            makeLayerPolygonEdges(header, config, layer, viz_config, boundary_edge_ns);
        addMultiMarkerIfValid(boundary_edges, msg);
      } else {
        deleteMultiMarker(header, boundary_edge_ns, msg);
      }
    } catch (const std::bad_cast&) {
      // TODO(nathan) consider warning
    }
  } else {
    deleteMultiMarker(header, boundary_ns, msg);
    deleteMultiMarker(header, boundary_edge_ns, msg);
  }

  const std::string boundary_ellipse_ns = getLayerBoundaryEllipseNamespace(layer.id);
  if (config.draw_boundary_ellipse) {
    try {
      Marker boundary_ellipse = makeLayerEllipseBoundaries(
          header, config, layer, viz_config, boundary_ellipse_ns);
      addMultiMarkerIfValid(boundary_ellipse, msg);
    } catch (const std::bad_cast&) {
      // TODO(nathan) consider warning
    }
  } else {
    deleteMultiMarker(header, boundary_ellipse_ns, msg);
  }

  clearPrevMarkers(
      header, curr_labels_.at(layer.id), label_ns, prev_labels_.at(layer.id), msg);
}

void DynamicSceneGraphVisualizer::drawLayerMeshEdges(const std_msgs::Header& header,
                                                     LayerId layer_id,
                                                     const std::string& ns,
                                                     MarkerArray& msg) {
  if (!scene_graph_->hasLayer(layer_id)) {
    return;
  }

  const auto layer_config = config_manager_->getLayerConfig(layer_id);
  if (!layer_config) {
    return;
  }

  if (!layer_config->visualize) {
    deleteMultiMarker(header, ns, msg);
    return;
  }

  Marker mesh_edges = makeMeshEdgesMarker(header,
                                          *layer_config,
                                          config_manager_->getVisualizerConfig(),
                                          *scene_graph_,
                                          scene_graph_->getLayer(layer_id),
                                          ns);
  addMultiMarkerIfValid(mesh_edges, msg);
}

}  // namespace hydra
