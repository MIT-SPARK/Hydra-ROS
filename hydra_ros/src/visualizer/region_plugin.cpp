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
#include "hydra_ros/visualizer/region_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/semantic_color_map.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/MarkerArray.h>

#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/polygon_utilities.h"
#include "hydra_ros/visualizer/visualizer_utilities.h"

namespace hydra {

using FlannStruct = pcl::KdTreeFLANN<pcl::PointXYZ>;

Eigen::MatrixXd getConcaveHull(const DynamicSceneGraph& graph,
                               const SceneGraphNode& parent,
                               double alpha,
                               bool use_convex) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& child : parent.children()) {
    const auto pos = graph.getPosition(child);
    auto& p = points->emplace_back();
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
  }

  return getHullPolygon(points, alpha, use_convex);
}

Eigen::MatrixXd getMeshConcaveHull(const DynamicSceneGraph& graph,
                                   const FlannStruct& flann,
                                   const SceneGraphNode& parent,
                                   double inflation_radius,
                                   double alpha,
                                   bool use_convex) {
  const auto mesh = graph.mesh();
  if (!mesh || mesh->empty()) {
    return {};
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& child : parent.children()) {
    const SceneGraphNode& child_node = graph.getNode(child).value();
    const auto& attrs = child_node.attributes<PlaceNodeAttributes>();
    for (const auto idx : attrs.pcl_mesh_connections) {
      const auto basis = mesh->pos(idx);
      pcl::Indices neighbors;
      std::vector<float> distances;
      pcl::PointXYZ pcl_basis;
      pcl_basis.x = basis.x();
      pcl_basis.y = basis.y();
      pcl_basis.z = basis.z();
      flann.radiusSearch(pcl_basis, inflation_radius, neighbors, distances);
      for (const auto n_idx : neighbors) {
        const auto p_n = mesh->pos(n_idx);
        auto& p = points->emplace_back();
        p.x = p_n.x();
        p.y = p_n.y();
        p.z = p_n.z();
      }
    }
  }

  LOG(ERROR) << "Using " << points->size() << " points for "
             << NodeSymbol(parent.id).getLabel() << " with alpha " << alpha;

  return getHullPolygon(points, alpha, use_convex);
}

void declare_config(RegionPluginConfig& config) {
  using namespace config;
  name("RegionPluginConfig");
  field(config.use_convex, "use_convex");
  field(config.skip_unknown, "skip_unknown");
  field(config.use_nodes_only, "use_nodes_only");
  field(config.draw_labels, "draw_labels");
  field(config.line_width, "line_width");
  field(config.line_alpha, "line_alpha");
  field(config.hull_alpha, "hull_alpha");
  field(config.inflation_radius, "inflation_radius");
  field(config.region_colormap, "region_colormap");

  check(config.line_width, GT, 0.0, "line_width");
  check(config.line_alpha, GT, 0.0, "line_alpha");
  check(config.hull_alpha, GT, 0.0, "hull_alpha");
}

RegionPlugin::RegionPlugin(const ros::NodeHandle& nh, const std::string& name)
    : DsgVisualizerPlugin(nh, name),
      config(config::checkValid(config::fromRos<RegionPluginConfig>(nh_))) {
  // namespacing gives us a reasonable topic
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("", 1, true);
}

RegionPlugin::~RegionPlugin() {}

void RegionPlugin::draw(const std_msgs::Header& header,
                        const DynamicSceneGraph& graph) {
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(3);

  msg.markers[0].header = header;
  msg.markers[0].ns = "region_plugin_mesh";
  msg.markers[0].id = 0;
  msg.markers[0].type = visualization_msgs::Marker::TRIANGLE_LIST;
  msg.markers[0].action = visualization_msgs::Marker::ADD;
  msg.markers[0].pose.orientation.w = 1.0;
  msg.markers[0].color.a = config.mesh_alpha;
  msg.markers[0].scale.x = 1.0;
  msg.markers[0].scale.y = 1.0;
  msg.markers[0].scale.z = 1.0;

  msg.markers[1].header = header;
  msg.markers[1].ns = "region_plugin_wireframe";
  msg.markers[1].id = 0;
  msg.markers[1].type = visualization_msgs::Marker::LINE_LIST;
  msg.markers[1].action = visualization_msgs::Marker::ADD;
  msg.markers[1].pose.orientation.w = 1.0;
  msg.markers[1].scale.x = config.line_width;
  msg.markers[1].scale.y = 0.0;
  msg.markers[1].scale.z = 0.0;

  msg.markers[2].header = header;
  msg.markers[2].ns = "region_plugin_wireframe_vertices";
  msg.markers[2].id = 0;
  msg.markers[2].type = visualization_msgs::Marker::SPHERE_LIST;
  msg.markers[2].action = visualization_msgs::Marker::ADD;
  msg.markers[2].pose.orientation.w = 1.0;
  msg.markers[2].scale.x = config.line_width;
  msg.markers[2].scale.y = config.line_width;
  msg.markers[2].scale.z = config.line_width;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  auto mesh = graph.mesh();
  if (mesh && !mesh->empty()) {
    for (size_t i = 0; i < mesh->numVertices(); ++i) {
      const auto pos = mesh->pos(i);
      pcl::PointXYZ p;
      p.x = pos.x();
      p.y = pos.y();
      p.z = pos.z();
      cloud->push_back(p);
    }
  }

  FlannStruct flann;
  flann.setInputCloud(cloud);

  const auto& regions = graph.getLayer(DsgLayers::ROOMS);
  for (auto&& [id, node] : regions.nodes()) {
    const auto& attrs = node->attributes<SemanticNodeAttributes>();
    if (config.skip_unknown && attrs.name == "unknown") {
      continue;
    }
    std_msgs::ColorRGBA color;
    color.r = attrs.color.x() / 255.0;
    color.g = attrs.color.y() / 255.0;
    color.b = attrs.color.z() / 255.0;
    color.a = config.line_alpha;

    double mean_z = getMeanChildHeight(graph, *node);
    Eigen::MatrixXd hull_points;
    if (config.use_nodes_only) {
      hull_points = getConcaveHull(graph, *node, config.hull_alpha, config.use_convex);
    } else {
      hull_points = getMeshConcaveHull(graph,
                                       flann,
                                       *node,
                                       config.inflation_radius,
                                       config.hull_alpha,
                                       config.use_convex);
    }

    if (config.draw_labels) {
      const auto& pos = attrs.position;
      visualization_msgs::Marker text_marker;
      text_marker.header = header;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.ns = "region_plugin_labels";
      text_marker.id = NodeSymbol(id).categoryId();
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.text = attrs.name;
      text_marker.scale.z = config.label_scale;
      text_marker.color.a = 1.0;
      text_marker.pose.orientation.w = 1.0;
      tf2::convert(pos, text_marker.pose.position);
      text_marker.pose.position.z = mean_z;
      published_labels_.insert(text_marker.id);
      msg.markers.push_back(text_marker);
    }

    auto mesh_color = color;
    mesh_color.a = config.mesh_alpha;
    makeFilledPolygon(hull_points, mesh_color, msg.markers[0], mean_z);
    makePolygonBoundary(hull_points, color, msg.markers[1], mean_z, &msg.markers[2]);
  }

  pub_.publish(msg);
}

void RegionPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  visualization_msgs::MarkerArray msg;
  msg.markers.push_back(makeDeleteMarker(header, 0, "region_plugin_mesh"));
  msg.markers.push_back(makeDeleteMarker(header, 0, "region_plugin_wireframe"));
  msg.markers.push_back(
      makeDeleteMarker(header, 0, "region_plugin_wireframe_vertices"));
  for (const auto id : published_labels_) {
    msg.markers.push_back(makeDeleteMarker(header, id, "region_plugin_labels"));
  }
  published_labels_.clear();
  pub_.publish(msg);
}

}  // namespace hydra
