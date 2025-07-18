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
#include "hydra_visualizer/scene_graph_renderer.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "hydra_visualizer/color/colormap_utilities.h"
#include "hydra_visualizer/drawing.h"

namespace hydra {

using namespace spark_dsg;
using namespace visualizer;

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

inline std::string keyToLayerName(LayerKey key) {
  std::stringstream ss;
  ss << "layer_" << key.layer;
  if (key.partition) {
    ss << "p" << key.partition;
  }

  return ss.str();
}

inline Marker makeNewEdgeList(const std_msgs::msg::Header& header,
                              const std::string& ns_prefix,
                              LayerKey source,
                              LayerKey target) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;

  std::stringstream ss;
  ss << ns_prefix << source << "_" << target;
  marker.ns = ss.str();
  return marker;
}

}  // namespace

struct MarkerNamespaces {
  static std::string layerNodeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_nodes";
  }

  static std::string layerEdgeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_edges";
  }

  static std::string layerTextNamespace(LayerKey key) {
    return keyToLayerName(key) + "_text";
  }

  static std::string layerBboxNamespace(LayerKey key) {
    return keyToLayerName(key) + "_bounding_boxes";
  }

  static std::string layerBoundaryNamespace(LayerKey key) {
    return keyToLayerName(key) + "_polygon_boundaries";
  }

  static std::string layerBoundaryEllipseNamespace(LayerKey key) {
    return keyToLayerName(key) + "_ellipsoid_boundaries";
  }

  static std::string layerBoundaryEdgeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_polygon_boundaries_edges";
  }

  static std::string meshEdgeNamespace(LayerKey key) {
    return keyToLayerName(key) + "_mesh_edges";
  }
};

void declare_config(GraphRenderConfig& config) {
  using namespace config;
  name("GraphRenderConfig");
  field(config.layer_z_step, "layer_z_step");
  field(config.collapse_layers, "collapse_layers");
}

void declare_config(SceneGraphRenderer::Config& config) {
  using namespace config;
  name("SceneGraphRenderer::Config");
  field(config.graph, "graph");
  field(config.layers, "layers");
  field(config.partitions, "partitions");
}

SceneGraphRenderer::SceneGraphRenderer(const Config& config, ianvs::NodeHandle nh)
    : nh_(nh),
      graph_config_("renderer", config.graph, [this]() { has_change_ = true; }),
      pub_(nh.create_publisher<MarkerArray>("graph", rclcpp::QoS(1).transient_local())),
      has_change_(false) {
  // init wrappers from parsed initial config
  for (const auto& [layer_id, layer_config] : config.layers) {
    const auto ns = "renderer/config/layer" + std::to_string(layer_id);
    layers_.emplace(layer_id,
                    std::make_unique<LayerConfigWrapper>(
                        ns, layer_config, [this]() { has_change_ = true; }));
  }

  for (const auto& [layer_id, layer_config] : config.partitions) {
    const auto ns = "renderer/config/partitions/layer" + std::to_string(layer_id);
    partitions_.emplace(layer_id,
                        std::make_unique<LayerConfigWrapper>(
                            ns, layer_config, [this]() { has_change_ = true; }));
  }
}

void SceneGraphRenderer::reset(const std_msgs::msg::Header& header) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

bool SceneGraphRenderer::hasChange() const { return has_change_; }

void SceneGraphRenderer::clearChangeFlag() { has_change_ = false; }

void SceneGraphRenderer::draw(const std_msgs::msg::Header& header,
                              const DynamicSceneGraph& graph) const {
  setConfigs(graph);

  MarkerArray msg;
  for (const auto& [layer_id, layer] : graph.layers()) {
    drawLayer(header, layer_infos_.at(layer_id), *layer, graph.mesh().get(), msg);
  }

  for (const auto& [l_id, partitions] : graph.layer_partitions()) {
    for (const auto& [partition_id, partition] : partitions) {
      drawLayer(header, partition_infos_.at(l_id), *partition, graph.mesh().get(), msg);
    }
  }

  MarkerArray edges;
  drawInterlayerEdges(header, graph, edges);
  tracker_.add(edges, msg);

  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_->publish(msg);
  }
}

void SceneGraphRenderer::drawInterlayerEdges(const std_msgs::msg::Header& header,
                                             const DynamicSceneGraph& graph,
                                             MarkerArray& msg) const {
  const std::string ns_prefix = "interlayer_edges_";
  std::map<LayerKey, size_t> marker_indices;
  std::map<LayerKey, size_t> num_since_last;
  for (const auto& [key, edge] : graph.interlayer_edges()) {
    const auto& source = graph.getNode(edge.source);
    const auto& source_info = getLayerInfo(source.layer);
    const auto& target = graph.getNode(edge.target);
    const auto& target_info = getLayerInfo(target.layer);
    if (!source_info.shouldVisualize(source) || !target_info.shouldVisualize(target)) {
      continue;
    }

    if (!source_info.config.edges.draw_interlayer ||
        !target_info.config.edges.draw_interlayer) {
      continue;
    }

    const auto use_source = source_info.config.edges.interlayer_use_source;
    const auto& info = use_source ? source_info : target_info;

    auto iter = marker_indices.find(source.layer);
    if (iter == marker_indices.end()) {
      iter = marker_indices.emplace(source.layer, msg.markers.size()).first;
      msg.markers.push_back(
          makeNewEdgeList(header, ns_prefix, source.layer, target.layer));
      msg.markers.back().scale.x = info.config.edges.interlayer_scale;
      // make sure we always draw at least one edge
      num_since_last[source.layer] = info.config.edges.interlayer_insertion_skip;
    }

    if (num_since_last[source.layer] >= info.config.edges.interlayer_insertion_skip) {
      num_since_last[source.layer] = 0;
    } else {
      num_since_last[source.layer]++;
      continue;
    }

    auto& marker = msg.markers.at(iter->second);
    geometry_msgs::msg::Point source_point;
    tf2::convert(source.attributes().position, source_point);
    source_point.z += source_info.z_offset;
    marker.points.push_back(source_point);

    geometry_msgs::msg::Point target_point;
    tf2::convert(target.attributes().position, target_point);
    target_point.z += target_info.z_offset;
    marker.points.push_back(target_point);

    const auto color = makeColorMsg(info.node_color(use_source ? source : target),
                                    info.config.edges.interlayer_alpha);
    marker.colors.push_back(color);
    marker.colors.push_back(color);
  }
}

void SceneGraphRenderer::drawLayer(const std_msgs::msg::Header& header,
                                   const LayerInfo& info,
                                   const SceneGraphLayer& layer,
                                   const Mesh* mesh,
                                   MarkerArray& msg) const {
  if (!info.config.visualize) {
    return;
  }

  if (info.config.draw_frontier_ellipse) {
    info.filter = [](const SceneGraphNode& node) {
      auto attrs = node.tryAttributes<PlaceNodeAttributes>();
      return attrs ? attrs->real_place : true;
    };
  }

  const auto node_ns = MarkerNamespaces::layerNodeNamespace(layer.id);
  tracker_.add(makeLayerNodeMarkers(header, info, layer, node_ns), msg);

  if (info.config.text.draw) {
    if (info.config.text.draw_layer) {
      LOG_FIRST_N(WARNING, 5) << "use_text and use_layer_text are mutually exclusive!";
    }

    const auto ns = MarkerNamespaces::layerTextNamespace(layer.id);
    tracker_.add(makeLayerNodeTextMarkers(header, info, layer, ns), msg);
  } else if (info.config.text.draw_layer && !layer.nodes().empty()) {
    const auto ns = MarkerNamespaces::layerTextNamespace(layer.id);
    tracker_.add(makeLayerTextMarker(header, info, layer, ns), msg);
  }

  if (info.config.bounding_boxes.draw) {
    const auto ns = MarkerNamespaces::layerBboxNamespace(layer.id);
    tracker_.add(makeLayerBoundingBoxes(header, info, layer, ns), msg);
  }

  if (info.config.boundaries.draw) {
    const auto ns = MarkerNamespaces::layerBoundaryNamespace(layer.id);
    const auto edge_ns = MarkerNamespaces::layerBoundaryEdgeNamespace(layer.id);
    tracker_.add(makeLayerPolygonBoundaries(header, info, layer, ns), msg);
    if (info.config.boundaries.collapse) {
      tracker_.add(makeLayerPolygonEdges(header, info, layer, edge_ns), msg);
    }
  }

  if (info.config.boundaries.draw_ellipse) {
    const auto ns = MarkerNamespaces::layerBoundaryEllipseNamespace(layer.id);
    tracker_.add(makeLayerEllipseBoundaries(header, info, layer, ns), msg);
  }

  if (info.config.draw_frontier_ellipse) {
    tracker_.add(makeEllipsoidMarkers(header, info, layer, "frontier_ns"), msg);
    info.filter = {};  // we reset the manual filter to draw edges to frontiers
  }

  const auto edge_ns = MarkerNamespaces::layerEdgeNamespace(layer.id);
  tracker_.add(makeLayerEdgeMarkers(header, info, layer, edge_ns), msg);

  if (mesh && info.config.draw_mesh_edges) {
    const std::string ns = MarkerNamespaces::meshEdgeNamespace(layer.id);
    tracker_.add(makeMeshEdgesMarker(header, info, layer, *mesh, ns), msg);
  }
}

void SceneGraphRenderer::setConfigs(const DynamicSceneGraph& graph) const {
  layer_infos_.clear();
  partition_infos_.clear();

  const auto& graph_config = graph_config_.get();
  for (const auto& [layer_id, layer] : graph.layers()) {
    auto iter = layers_.find(layer_id);
    if (iter == layers_.end()) {
      // TODO(nathan) think about logging
      const auto ns = "renderer/config/layer" + std::to_string(layer_id);
      iter = layers_.emplace(layer_id, std::make_unique<LayerConfigWrapper>(ns)).first;
      iter->second->setCallback([this]() { has_change_ = true; });
    }

    // TODO(nathan) this is ugly because layer info doesn't have a copy constructor
    layer_infos_.emplace(layer_id, LayerInfo(iter->second->get()))
        .first->second.offset(graph_config.layer_z_step, graph_config.collapse_layers)
        .graph(graph, layer_id);
  }

  for (const auto& [l_id, partitions] : graph.layer_partitions()) {
    auto iter = partitions_.find(l_id);
    if (iter == partitions_.end()) {
      // TODO(nathan) think about logging
      const auto ns = "renderer/config/partitions/layer" + std::to_string(l_id);
      iter = partitions_.emplace(l_id, std::make_unique<LayerConfigWrapper>(ns)).first;
      iter->second->setCallback([this]() { has_change_ = true; });
    }

    // TODO(nathan) this is ugly because layer info doesn't have a copy constructor
    partition_infos_.emplace(l_id, LayerInfo(iter->second->get()))
        .first->second.offset(graph_config.layer_z_step, graph_config.collapse_layers)
        .graph(graph, l_id);
  }
}

const LayerInfo& SceneGraphRenderer::getLayerInfo(LayerKey layer) const {
  if (!layer.partition) {
    return layer_infos_.at(layer.layer);
  } else {
    return partition_infos_.at(layer.layer);
  }
}

}  // namespace hydra
