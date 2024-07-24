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

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <std_msgs/String.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/config_manager.h"
#include "hydra_ros/visualizer/dsg_visualizer_plugin.h"
#include "hydra_ros/visualizer/visualizer_utilities.h"

namespace hydra {

using namespace spark_dsg;
using namespace visualizer;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

struct MarkerNamespaces {
  static std::string layerNodeNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_nodes_") + std::to_string(layer);
  }

  static std::string layerEdgeNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_edges_") + std::to_string(layer);
  }

  static std::string layerLabelNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_labels_") + std::to_string(layer);
  }

  static std::string layerBboxNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_bounding_boxes_") + std::to_string(layer);
  }

  static std::string layerBboxEdgeNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_bounding_boxes_edges_") + std::to_string(layer);
  }

  static std::string layerBoundaryNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_polygon_boundaries_") + std::to_string(layer);
  }

  static std::string layerBoundaryEllipseNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_ellipsoid_boundaries_") + std::to_string(layer);
  }

  static std::string layerBoundaryEdgeNamespace(spark_dsg::LayerId layer) {
    return std::string("layer_polygon_boundaries_edges_") + std::to_string(layer);
  }

  static std::string dynamicNodeNamespace(char layer_prefix) {
    return std::string("dynamic_nodes_") + layer_prefix;
  }

  static std::string dynamicEdgeNamespace(char layer_prefix) {
    return std::string("dynamic_edges_") + layer_prefix;
  }

  static std::string dynamicLabelNamespace(char layer_prefix) {
    return std::string("dynamic_label_") + layer_prefix;
  }
};

void declare_config(DynamicSceneGraphVisualizer::Config& config) {
  using namespace config;
  name("DynamicSceneGraphVisualizer::Config");
  field(config.visualizer_frame, "visualizer_frame");
  checkCondition(!config.visualizer_frame.empty(), "visualizer_frame");
}

DynamicSceneGraphVisualizer::DynamicSceneGraphVisualizer(const ros::NodeHandle& nh)
    : config(config::fromRos<Config>(nh)),
      nh_(nh),
      need_redraw_(false),
      periodic_redraw_(false),
      pub_(nh_.advertise<MarkerArray>("dsg_markers", 1, true)) {
  ConfigManager::init(nh_);
}

void DynamicSceneGraphVisualizer::reset() {
  if (!graph_) {
    return;
  }

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config.visualizer_frame;

  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }

  for (const auto& plugin : plugins_) {
    plugin->reset(header, *graph_);
  }

  graph_.reset();
}

bool DynamicSceneGraphVisualizer::redraw() {
  if (!graph_) {
    return false;
  }

  auto& manager = ConfigManager::instance();
  need_redraw_ |= manager.hasChange();
  for (const auto& plugin : plugins_) {
    need_redraw_ |= plugin->hasChange();
  }

  if (!need_redraw_) {
    return false;
  }

  need_redraw_ = false;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = config.visualizer_frame;

  redrawImpl(header);

  manager.clearChangeFlags();
  for (auto& plugin : plugins_) {
    plugin->clearChangeFlag();
  }

  return true;
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

void DynamicSceneGraphVisualizer::setGraph(const DynamicSceneGraph::Ptr& scene_graph,
                                           bool need_reset) {
  if (scene_graph == nullptr) {
    ROS_ERROR("Request to visualize invalid scene graph! Ignoring");
    return;
  }

  if (need_reset || !graph_) {
    ConfigManager::reset(*scene_graph);
  }

  if (need_reset) {
    reset();
  }

  graph_ = scene_graph;
  need_redraw_ = true;
}

bool DynamicSceneGraphVisualizer::graphIsSet() const { return graph_ != nullptr; }

DynamicSceneGraph::Ptr DynamicSceneGraphVisualizer::getGraph() { return graph_; }

void DynamicSceneGraphVisualizer::addPlugin(const DsgVisualizerPlugin::Ptr& plugin) {
  plugins_.push_back(plugin);
}

void DynamicSceneGraphVisualizer::clearPlugins() { plugins_.clear(); }

void DynamicSceneGraphVisualizer::setNeedRedraw() { need_redraw_ = true; }

void DynamicSceneGraphVisualizer::setGraphUpdated() { need_redraw_ = true; }

void DynamicSceneGraphVisualizer::displayLoop(const ros::WallTimerEvent&) {
  if (periodic_redraw_) {
    need_redraw_ = true;
  }

  redraw();
}

void DynamicSceneGraphVisualizer::redrawImpl(const std_msgs::Header& header) {
  visualizer::GraphInfo info;
  const auto& manager = ConfigManager::instance();

  MarkerArray msg;
  for (auto&& [layer_id, layer] : graph_->layers()) {
    const auto& conf = manager.getLayerConfig(layer_id);
    auto iter = info.layers.emplace(layer_id, conf.getInfo(*graph_)).first;
    drawLayer(header, iter->second, *layer, msg);
  }

  for (auto&& [layer_id, sublayers] : graph_->dynamicLayers()) {
    const auto& conf = manager.getDynamicLayerConfig(layer_id);
    auto iter = info.dynamic_layers.emplace(layer_id, conf.getInfo(*graph_)).first;
    for (auto&& [prefix, layer] : sublayers) {
      drawDynamicLayer(header, iter->second, *layer, msg);
    }
  }

  const auto static_edges = makeGraphEdgeMarkers(
      header, info, *graph_, graph_->interlayer_edges(), "interlayer_edges_");
  tracker_.add(static_edges, msg);

  const auto dynamic_edges = makeGraphEdgeMarkers(header,
                                                  info,
                                                  *graph_,
                                                  graph_->dynamic_interlayer_edges(),
                                                  "dynamic_interlayer_edges_");
  tracker_.add(dynamic_edges, msg);

  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }

  for (const auto& plugin : plugins_) {
    plugin->draw(header, *graph_);
  }
}

void DynamicSceneGraphVisualizer::drawLayer(const std_msgs::Header& header,
                                            const StaticLayerInfo& info,
                                            const SceneGraphLayer& layer,
                                            MarkerArray& msg) {
  if (!info.layer.visualize) {
    return;
  }

  if (info.layer.draw_frontier_ellipse) {
    info.filter = [](const SceneGraphNode& node) {
      try {
        return node.attributes<PlaceNodeAttributes>().real_place;
      } catch (const std::bad_cast&) {
        return true;
      }
    };
  }

  const auto node_ns = MarkerNamespaces::layerNodeNamespace(layer.id);
  tracker_.add(makeLayerNodeMarkers(header, info, layer, node_ns), msg);

  if (info.layer.use_label) {
    const auto ns = MarkerNamespaces::layerLabelNamespace(layer.id);
    tracker_.add(makeLayerLabelMarkers(header, info, layer, ns), msg);
  }

  if (info.layer.use_bounding_box) {
    const auto ns = MarkerNamespaces::layerBboxNamespace(layer.id);
    const auto edge_ns = MarkerNamespaces::layerBboxEdgeNamespace(layer.id);
    try {
      tracker_.add(makeLayerBoundingBoxes(header, info, layer, ns), msg);
      if (info.layer.collapse_bounding_box) {
        tracker_.add(makeEdgesToBoundingBoxes(header, info, layer, edge_ns), msg);
      }
    } catch (const std::bad_cast& e) {
      LOG_FIRST_N(WARNING, 5) << "unable to draw bounding boxes for layer " << layer.id
                              << ": " << e.what();
    }
  }

  if (info.layer.draw_boundaries) {
    const auto ns = MarkerNamespaces::layerBoundaryNamespace(layer.id);
    const auto edge_ns = MarkerNamespaces::layerBoundaryEdgeNamespace(layer.id);
    try {
      tracker_.add(makeLayerPolygonBoundaries(header, info, layer, ns), msg);
      if (info.layer.collapse_boundary) {
        tracker_.add(makeLayerPolygonEdges(header, info, layer, edge_ns), msg);
      }
    } catch (const std::bad_cast& e) {
      LOG_FIRST_N(WARNING, 5) << "Could not draw boundaries for layer " << layer.id
                              << ": " << e.what();
    }
  }

  if (info.layer.draw_boundary_ellipse) {
    const auto ns = MarkerNamespaces::layerBoundaryEllipseNamespace(layer.id);
    try {
      tracker_.add(makeLayerEllipseBoundaries(header, info, layer, ns), msg);
    } catch (const std::bad_cast& e) {
      LOG_FIRST_N(WARNING, 5) << "Could not draw boundary ellipses for layer "
                              << layer.id << ": " << e.what();
    }
  }

  if (info.layer.draw_frontier_ellipse) {
    tracker_.add(makeEllipsoidMarkers(header, info, layer, "frontier_ns"), msg);
    info.filter = {};  // we reset the manual filter to draw edges to frontiers
  }

  if (info.layer.draw_mesh_edges) {
    const std::string ns = "mesh_2d_place_connections";
    tracker_.add(makeMeshEdgesMarker(header, info, *graph_, layer, ns), msg);
  }

  const auto edge_ns = MarkerNamespaces::layerEdgeNamespace(layer.id);
  tracker_.add(makeLayerEdgeMarkers(header, info, layer, edge_ns), msg);
}

void DynamicSceneGraphVisualizer::drawDynamicLayer(const std_msgs::Header& header,
                                                   const DynamicLayerInfo& info,
                                                   const DynamicSceneGraphLayer& layer,
                                                   MarkerArray& msg) {
  if (!info.layer.visualize) {
    return;
  }

  const auto node_ns = MarkerNamespaces::dynamicNodeNamespace(layer.prefix);
  tracker_.add(makeDynamicNodeMarkers(header, info, layer, node_ns), msg);

  const auto edge_ns = MarkerNamespaces::dynamicEdgeNamespace(layer.prefix);
  tracker_.add(makeDynamicEdgeMarkers(header, info, layer, edge_ns), msg);

  const auto label_ns = MarkerNamespaces::dynamicLabelNamespace(layer.prefix);
  tracker_.add(makeDynamicLabelMarker(header, info, layer, label_ns), msg);
}

}  // namespace hydra
