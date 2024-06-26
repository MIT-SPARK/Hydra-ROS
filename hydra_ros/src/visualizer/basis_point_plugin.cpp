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
#include "hydra_ros/visualizer/basis_point_plugin.h"

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/semantic_color_map.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/visualizer_utilities.h"

namespace hydra {

using dsg_utils::makeColorMsg;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

void declare_config(BasisPointPlugin::Config& config) {
  using namespace config;
  name("BasisPlugin::Config");
  field(config.show_voxblox_connections, "show_voxblox_connections");
  field(config.draw_basis_points, "draw_basis_points");
  field(config.places_use_sphere, "places_use_sphere");
  field(config.draw_places_labels, "draw_places_labels");
  field(config.places_label_scale, "places_label_scale");
  field(config.places_node_scale, "places_node_scale");
  field(config.places_node_alpha, "places_node_alpha");
  field(config.places_edge_scale, "places_edge_scale");
  field(config.places_edge_alpha, "places_edge_alpha");
  field(config.basis_point_scale, "basis_point_scale");
  field(config.basis_point_alpha, "basis_point_alpha");
  field(config.label_colormap, "label_colormap");
}

struct BasisPoint {
  double x;
  double y;
  double z;
  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
};

std::vector<BasisPoint> getBasisPoints(const PlaceNodeAttributes& attrs,
                                       bool use_voxblox,
                                       const spark_dsg::Mesh* vertices,
                                       const SemanticColorMap* colormap) {
  std::vector<BasisPoint> to_return;
  if (!use_voxblox && !vertices) {
    return to_return;
  }

  if (use_voxblox) {
    for (const auto& info : attrs.voxblox_mesh_connections) {
      auto& basis_point = to_return.emplace_back();
      Eigen::Vector3d pos = Eigen::Map<const Eigen::Vector3d>(info.voxel_pos);
      basis_point.x = pos.x();
      basis_point.y = pos.y();
      basis_point.z = pos.z();
      if (info.label && colormap && colormap->isValid()) {
        const auto label_color = colormap->getColorFromLabel(*info.label);
        basis_point.r = label_color.r;
        basis_point.g = label_color.g;
        basis_point.b = label_color.b;
      }
    }
  } else {
    for (const auto idx : attrs.pcl_mesh_connections) {
      const auto pos = vertices->pos(idx);
      const auto color = vertices->colors.at(idx);
      auto& basis_point = to_return.emplace_back();
      basis_point.x = pos.x();
      basis_point.y = pos.y();
      basis_point.z = pos.z();
      basis_point.r = color.r / 255.0;
      basis_point.g = color.g / 255.0;
      basis_point.b = color.b / 255.0;
    }
  }

  return to_return;
}

BasisPointPlugin::BasisPointPlugin(const Config& config,
                                   const ros::NodeHandle& nh,
                                   const std::string& name)
    : DsgVisualizerPlugin(nh, name), config(config::checkValid(config)) {
  if (!config.label_colormap.empty()) {
    colormap_ = SemanticColorMap::fromCsv(config.label_colormap);
    if (!colormap_) {
      LOG(WARNING) << "Unable to load colormap from " << config.label_colormap;
    }
  }

  // namespacing gives us a reasonable topic
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("", 1, true);
}

void BasisPointPlugin::draw(const ConfigManager&,
                            const std_msgs::Header& header,
                            const DynamicSceneGraph& graph) {
  // TODO(nathan) actual use dynamic configs
  if (!graph.hasLayer(DsgLayers::PLACES)) {
    reset(header, graph);
    return;
  }

  const auto& places = graph.getLayer(DsgLayers::PLACES);
  if (places.numNodes() == 0) {
    reset(header, graph);
    return;
  }

  MarkerArray msg;
  drawNodes(header, graph, msg);
  drawEdges(header, graph, msg);
  if (config.draw_basis_points) {
    drawBasisPoints(header, graph, msg);
  }
  pub_.publish(msg);
}

void BasisPointPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  MarkerArray msg;
  for (const auto& ns : published_) {
    msg.markers.push_back(makeDeleteMarker(header, 0, ns));
  }
  published_.clear();

  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }
}

void BasisPointPlugin::drawNodes(const std_msgs::Header& header,
                                 const DynamicSceneGraph& graph,
                                 MarkerArray& msg) const {
  const auto& layer = graph.getLayer(DsgLayers::PLACES);

  VisualizerConfig viz_config;
  viz_config.layer_z_step = 0.0;

  hydra_ros::LayerVisualizerConfig layer_config;
  layer_config.marker_scale = config.places_node_scale;
  layer_config.marker_alpha = config.places_node_alpha;
  layer_config.use_sphere_marker = config.places_use_sphere;
  layer_config.use_label = config.draw_places_labels;
  layer_config.label_scale = config.places_label_scale;
  layer_config.use_bounding_box = false;
  auto nodes = makeCentroidMarkers(
      header,
      layer_config,
      layer,
      viz_config,
      "places_parent_graph_nodes",
      [&](const SceneGraphNode& node) -> Color {
        const auto parent = node.getParent();
        if (!parent) {
          return Color();
        } else {
          return graph.getNode(*parent).attributes<SemanticNodeAttributes>().color;
        }
      });

  published_.insert(nodes.ns);
  msg.markers.push_back(nodes);
}

void BasisPointPlugin::drawEdges(const std_msgs::Header& header,
                                 const DynamicSceneGraph& graph,
                                 MarkerArray& msg) const {
  auto& marker = msg.markers.emplace_back();
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "places_parent_edges";
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config.places_edge_scale;
  marker.color.a = config.places_edge_alpha;

  const auto& layer = graph.getLayer(DsgLayers::PLACES);
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();

    geometry_msgs::Point start;
    tf2::convert(attrs.position, start);

    const auto basis_points = getBasisPoints(
        attrs, config.show_voxblox_connections, graph.mesh().get(), colormap_.get());
    for (const auto& basis_point : basis_points) {
      marker.points.push_back(start);
      auto& point = marker.points.emplace_back();
      point.x = basis_point.x;
      point.y = basis_point.y;
      point.z = basis_point.z;
    }
  }

  published_.insert(marker.ns);
}

void BasisPointPlugin::drawBasisPoints(const std_msgs::Header& header,
                                       const DynamicSceneGraph& graph,
                                       MarkerArray& msg) const {
  const auto mesh = graph.mesh();
  if (!config.show_voxblox_connections && !mesh) {
    return;
  }

  auto& marker = msg.markers.emplace_back();
  marker.header = header;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "places_parents";
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config.basis_point_scale;
  marker.scale.y = config.basis_point_scale;
  marker.scale.z = config.basis_point_scale;

  const auto& layer = graph.getLayer(DsgLayers::PLACES);
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    const auto basis_points = getBasisPoints(
        attrs, config.show_voxblox_connections, mesh.get(), colormap_.get());
    for (const auto& basis_point : basis_points) {
      auto& point = marker.points.emplace_back();
      point.x = basis_point.x;
      point.y = basis_point.y;
      point.z = basis_point.z;
      auto& color = marker.colors.emplace_back();
      color.r = basis_point.r;
      color.g = basis_point.g;
      color.b = basis_point.b;
      color.a = config.basis_point_alpha;
    }
  }

  published_.insert(marker.ns);
}

}  // namespace hydra
