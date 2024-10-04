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
#include "hydra_visualizer/utils/visualizer_utilities.h"

#include <glog/logging.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <tf2_eigen/tf2_eigen.h>

#include <random>

#include "hydra_visualizer/color/colormap_utilities.h"

namespace spark_dsg {

bool operator<(const LayerKey& lhs, const LayerKey& rhs) {
  if (lhs.prefix == rhs.prefix) {
    return lhs.layer < rhs.layer;
  }

  return lhs.prefix < rhs.prefix;
}

}  // namespace spark_dsg

namespace hydra::visualizer {

using namespace spark_dsg;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace {

inline void fillPoseWithIdentity(geometry_msgs::Pose& pose) {
  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), pose.orientation);
}

inline Marker makeNewEdgeList(const std_msgs::Header& header,
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
  fillPoseWithIdentity(marker.pose);
  return marker;
}

inline void convertVec3f(const Eigen::Vector3f& v, geometry_msgs::Point& p) {
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
}

}  // namespace

void drawBoundingBox(const spark_dsg::BoundingBox& bbox,
                     const std_msgs::ColorRGBA& color,
                     Marker& marker) {
  // marker.header = header;
  // marker.type = Marker::LINE_LIST;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.id = 0;
  // marker.ns = ns;
  // marker.scale.x = info.layer.bounding_box_scale;
  // fillPoseWithIdentity(marker.pose);

  const static std::array<size_t, 8> remapping{0, 1, 3, 2, 4, 5, 7, 6};
  const auto corners = bbox.corners();

  for (size_t c = 0; c < remapping.size(); ++c) {
    // edges are 1-bit pertubations
    size_t x_neighbor = c | 0x01;
    size_t y_neighbor = c | 0x02;
    size_t z_neighbor = c | 0x04;
    if (c != x_neighbor) {
      convertVec3f(corners[remapping[c]], marker.points.emplace_back());
      convertVec3f(corners[remapping[x_neighbor]], marker.points.emplace_back());
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }

    if (c != y_neighbor) {
      convertVec3f(corners[remapping[c]], marker.points.emplace_back());
      convertVec3f(corners[remapping[y_neighbor]], marker.points.emplace_back());
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }

    if (c != z_neighbor) {
      convertVec3f(corners[remapping[c]], marker.points.emplace_back());
      convertVec3f(corners[remapping[z_neighbor]], marker.points.emplace_back());
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }
  }
}

MarkerArray makeLayerBoundingBoxes(const std_msgs::Header& header,
                                   const StaticLayerInfo& info,
                                   const SceneGraphLayer& layer,
                                   const std::string& ns) {
  // we only draw edges if the graph is not collapsed but the boxes are
  const bool draw_edges =
      info.layer.collapse_bounding_box && info.getZOffset() >= 1.0e-6;

  MarkerArray markers;
  markers.markers.resize(draw_edges ? 2 : 1);

  auto& marker = markers.markers[0];
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = info.layer.bounding_box_scale;

  fillPoseWithIdentity(marker.pose);
  marker.pose.position.z += info.layer.collapse_bounding_box ? 0 : info.getZOffset();
  marker.points.reserve(24 * layer.numNodes());
  marker.colors.reserve(24 * layer.numNodes());

  Marker* edges = nullptr;
  if (draw_edges) {
    edges = &markers.markers[1];
    edges->header = header;
    edges->type = Marker::LINE_LIST;
    edges->action = visualization_msgs::Marker::ADD;
    edges->id = 1;
    edges->ns = ns;
    edges->scale.x = info.layer.bounding_box_edge_scale;
    fillPoseWithIdentity(edges->pose);
    edges->points.reserve(8 * layer.numNodes());
    edges->colors.reserve(8 * layer.numNodes());
  }

  for (const auto& [node_id, node] : layer.nodes()) {
    if (info.filter && !info.filter(*node)) {
      continue;
    }

    const auto& attrs = node->attributes<SemanticNodeAttributes>();
    const auto color =
        makeColorMsg(info.node_color(*node), info.layer.bounding_box_alpha);
    size_t offset = marker.points.size();
    drawBoundingBox(attrs.bounding_box, color, marker);

    if (edges) {
      geometry_msgs::Point node_centroid;
      tf2::convert(attrs.position, node_centroid);
      node_centroid.z += info.getZOffset();

      geometry_msgs::Point center_point;
      tf2::convert(attrs.position, center_point);
      center_point.z += info.layer.mesh_edge_break_ratio * info.getZOffset();

      edges->points.push_back(node_centroid);
      edges->colors.push_back(color);
      edges->points.push_back(center_point);
      edges->colors.push_back(color);

      for (size_t i = 0; i < 8; ++i) {
        edges->colors.push_back(color);
      }

      // top box corners appear as the fourth to last and last edge
      // bottom corners account for 8 edges
      edges->points.push_back(center_point);
      edges->points.push_back(marker.points.at(offset + 16));
      edges->points.push_back(center_point);
      edges->points.push_back(marker.points.at(offset + 17));
      edges->points.push_back(center_point);
      edges->points.push_back(marker.points.at(offset + 22));
      edges->points.push_back(center_point);
      edges->points.push_back(marker.points.at(offset + 23));
    }
  }

  return markers;
}

Marker makeLayerEllipseBoundaries(const std_msgs::Header& header,
                                  const StaticLayerInfo& info,
                                  const SceneGraphLayer& layer,
                                  const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = info.layer.boundary_wireframe_scale;
  fillPoseWithIdentity(marker.pose);
  marker.pose.position.z += info.layer.collapse_boundary ? 0.0 : info.getZOffset();

  geometry_msgs::Point last_point;
  std_msgs::ColorRGBA color;

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<Place2dNodeAttributes>();
    if (attrs.boundary.size() <= 1) {
      continue;
    }

    color = makeColorMsg(info.node_color(*node), info.layer.boundary_ellipse_alpha);
    const auto pos = attrs.position;
    last_point.x = attrs.ellipse_matrix_expand(0, 0) + attrs.ellipse_centroid(0);
    last_point.y = attrs.ellipse_matrix_expand(1, 0) + attrs.ellipse_centroid(1);
    last_point.z = pos.z();

    int npts = 20;
    for (int ix = 1; ix < npts + 1; ++ix) {
      marker.points.push_back(last_point);
      marker.colors.push_back(color);

      float t = ix * 2 * M_PI / npts;
      Eigen::Vector2d p2 =
          attrs.ellipse_matrix_expand * Eigen::Vector2d(cos(t), sin(t));
      last_point.x = p2(0) + attrs.ellipse_centroid(0);
      last_point.y = p2(1) + attrs.ellipse_centroid(1);
      last_point.z = pos.z();

      marker.points.push_back(last_point);
      marker.colors.push_back(color);
    }
  }
  return marker;
}

Marker makeLayerPolygonEdges(const std_msgs::Header& header,
                             const StaticLayerInfo& info,
                             const SceneGraphLayer& layer,
                             const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = info.layer.boundary_wireframe_scale;
  fillPoseWithIdentity(marker.pose);

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<Place2dNodeAttributes>();
    if (attrs.boundary.size() <= 1) {
      continue;
    }

    const auto pos = attrs.position;
    geometry_msgs::Point node_point;
    tf2::convert(pos, node_point);
    node_point.z += info.getZOffset();
    const auto color = makeColorMsg(info.node_color(*node), info.layer.boundary_alpha);

    for (size_t i = 0; i < attrs.boundary.size(); ++i) {
      geometry_msgs::Point boundary_point;
      tf2::convert(attrs.boundary[i], boundary_point);
      boundary_point.z = pos.z();

      marker.points.push_back(boundary_point);
      marker.colors.push_back(color);
      marker.points.push_back(node_point);
      marker.colors.push_back(color);
    }
  }

  return marker;
}

Marker makeLayerPolygonBoundaries(const std_msgs::Header& header,
                                  const StaticLayerInfo& info,
                                  const SceneGraphLayer& layer,
                                  const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.ns = ns;
  marker.scale.x = info.layer.boundary_wireframe_scale;

  fillPoseWithIdentity(marker.pose);
  marker.pose.position.z += info.layer.collapse_boundary ? 0.0 : info.getZOffset();

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<Place2dNodeAttributes>();
    if (attrs.boundary.size() <= 1) {
      continue;
    }

    const auto pos = attrs.position;

    std_msgs::ColorRGBA color;
    if (info.layer.boundary_use_node_color) {
      color = makeColorMsg(info.node_color(*node), info.layer.boundary_alpha);
    } else {
      color = makeColorMsg(Color(), info.layer.boundary_alpha);
    }

    geometry_msgs::Point last_point;
    tf2::convert(attrs.boundary.back(), last_point);
    last_point.z = pos.z();

    for (size_t i = 0; i < attrs.boundary.size(); ++i) {
      marker.points.push_back(last_point);
      marker.colors.push_back(color);

      tf2::convert(attrs.boundary[i], last_point);
      last_point.z = pos.z();
      marker.points.push_back(last_point);
      marker.colors.push_back(color);
    }
  }

  return marker;
}

MarkerArray makeEllipsoidMarkers(const std_msgs::Header& header,
                                 const StaticLayerInfo& info,
                                 const SceneGraphLayer& layer,
                                 const std::string& ns) {
  size_t id = 0;
  MarkerArray msg;
  for (const auto& [node_id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<PlaceNodeAttributes>();
    if (attrs.real_place) {
      continue;
    }

    Marker marker;
    marker.header = header;
    marker.type = Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id++;
    marker.ns = ns;

    marker.scale.x = attrs.frontier_scale.x();
    marker.scale.y = attrs.frontier_scale.y();
    marker.scale.z = attrs.frontier_scale.z();

    tf2::convert(attrs.position, marker.pose.position);
    tf2::convert(attrs.orientation, marker.pose.orientation);

    marker.pose.position.z += info.getZOffset();
    marker.color = makeColorMsg(info.node_color(*node), info.layer.marker_alpha);
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray makeLayerLabelMarkers(const std_msgs::Header& header,
                                  const StaticLayerInfo& info,
                                  const SceneGraphLayer& layer,
                                  const std::string& ns) {
  MarkerArray msg;
  if (!info.node_label) {
    LOG(WARNING) << "Missing node label function!";
    return msg;
  }

  for (const auto& [node_id, node] : layer.nodes()) {
    if (info.filter && !info.filter(*node)) {
      continue;
    }

    auto& marker = msg.markers.emplace_back();
    marker.header = header;
    marker.ns = ns;
    marker.id = node->id;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.action = Marker::ADD;
    marker.lifetime = ros::Duration(0);

    const auto name = info.node_label(*node);
    if (name.empty()) {
      continue;
    }

    marker.text = name.empty() ? NodeSymbol(node->id).getLabel() : name;
    marker.scale.z = info.layer.label_scale;
    marker.color = makeColorMsg(Color());

    fillPoseWithIdentity(marker.pose);
    tf2::convert(node->attributes().position, marker.pose.position);
    marker.pose.position.z += info.layer.label_height;
    if (!info.layer.collapse_label) {
      marker.pose.position.z += info.getZOffset();
    }

    if (info.layer.add_label_jitter) {
      static std::random_device rd;
      static std::mt19937 gen(rd());
      std::uniform_real_distribution dist(-1.0, 1.0);
      const auto z_jitter = info.layer.label_jitter_scale * dist(gen);
      marker.pose.position.z += z_jitter;
    }
  }

  return msg;
}

Marker makeLayerNodeMarkers(const std_msgs::Header& header,
                            const StaticLayerInfo& info,
                            const SceneGraphLayer& layer,
                            const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = info.layer.use_sphere_marker ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.ns = ns;

  marker.scale.x = info.layer.marker_scale;
  marker.scale.y = info.layer.marker_scale;
  marker.scale.z = info.layer.marker_scale;

  fillPoseWithIdentity(marker.pose);

  marker.points.reserve(layer.numNodes());
  marker.colors.reserve(layer.numNodes());
  for (const auto& [node_id, node] : layer.nodes()) {
    if (info.filter && !info.filter(*node)) {
      continue;
    }

    geometry_msgs::Point node_centroid;
    tf2::convert(node->attributes().position, node_centroid);
    node_centroid.z += info.getZOffset();
    marker.points.push_back(node_centroid);

    const auto desired_color = info.node_color(*node);
    marker.colors.push_back(makeColorMsg(desired_color, info.layer.marker_alpha));
  }

  return marker;
}

Marker makeLayerEdgeMarkers(const std_msgs::Header& header,
                            const StaticLayerInfo& info,
                            const SceneGraphLayer& layer,
                            const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.id = 0;
  marker.ns = ns;

  marker.action = Marker::ADD;
  marker.scale.x = info.layer.edge_scale;
  fillPoseWithIdentity(marker.pose);

  for (const auto& [key, edge] : layer.edges()) {
    const auto& source_node = layer.getNode(edge.source);
    const auto& target_node = layer.getNode(edge.target);
    if (info.filter && (!info.filter(source_node) || !info.filter(target_node))) {
      continue;
    }

    geometry_msgs::Point source;
    tf2::convert(source_node.attributes().position, source);
    source.z += info.getZOffset();
    marker.points.push_back(source);

    geometry_msgs::Point target;
    tf2::convert(target_node.attributes().position, target);
    target.z += info.getZOffset();
    marker.points.push_back(target);

    const auto c_source = info.edge_color(source_node, target_node, edge, true);
    const auto c_target = info.edge_color(source_node, target_node, edge, false);
    marker.colors.push_back(makeColorMsg(c_source, info.layer.edge_alpha));
    marker.colors.push_back(makeColorMsg(c_target, info.layer.edge_alpha));
  }

  return marker;
}

Marker makeMeshEdgesMarker(const std_msgs::Header& header,
                           const StaticLayerInfo& info,
                           const DynamicSceneGraph& graph,
                           const SceneGraphLayer& layer,
                           const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;

  marker.scale.x = info.layer.interlayer_edge_scale;
  fillPoseWithIdentity(marker.pose);

  const auto mesh = graph.mesh();
  if (!mesh || info.graph.collapse_layers) {
    return marker;
  }

  for (const auto& [node_id, node] : layer.nodes()) {
    const auto& attrs = node->attributes<Place2dNodeAttributes>();
    const auto& mesh_edge_indices = attrs.pcl_mesh_connections;
    if (mesh_edge_indices.empty()) {
      continue;
    }

    const auto alpha = info.layer.interlayer_edge_alpha;
    const auto color =
        info.layer.interlayer_edge_use_color ? info.node_color(*node) : Color();

    geometry_msgs::Point center_point;
    tf2::convert(attrs.position, center_point);
    center_point.z += info.layer.mesh_edge_break_ratio * info.getZOffset();

    geometry_msgs::Point centroid_location;
    tf2::convert(attrs.position, centroid_location);
    centroid_location.z += info.getZOffset();

    // make first edge
    marker.points.push_back(centroid_location);
    marker.points.push_back(center_point);
    marker.colors.push_back(makeColorMsg(color, alpha));
    marker.colors.push_back(makeColorMsg(color, alpha));

    size_t i = 0;
    for (const auto midx : mesh_edge_indices) {
      ++i;
      if ((i - 1) % (info.layer.interlayer_edge_insertion_skip + 1) != 0) {
        continue;
      }

      if (midx >= mesh->numVertices()) {
        continue;
      }

      Eigen::Vector3d vertex_pos = mesh->pos(midx).cast<double>();
      geometry_msgs::Point vertex;
      tf2::convert(vertex_pos, vertex);

      marker.points.push_back(center_point);
      marker.points.push_back(vertex);
      marker.colors.push_back(makeColorMsg(color, alpha));
      marker.colors.push_back(makeColorMsg(color, alpha));
    }
  }

  return marker;
}

Marker makeDynamicNodeMarkers(const std_msgs::Header& header,
                              const DynamicLayerInfo& info,
                              const DynamicSceneGraphLayer& layer,
                              const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = info.layer.node_use_sphere ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = ns;
  marker.id = layer.prefix;

  marker.scale.x = info.layer.node_scale;
  marker.scale.y = info.layer.node_scale;
  marker.scale.z = info.layer.node_scale;

  fillPoseWithIdentity(marker.pose);

  marker.points.reserve(layer.numNodes());
  for (const auto& node : layer.nodes()) {
    if (!node) {
      continue;
    }

    geometry_msgs::Point node_centroid;
    tf2::convert(node->attributes().position, node_centroid);
    node_centroid.z += info.getZOffset();
    marker.points.push_back(node_centroid);
    const auto color = info.node_color(*node);
    marker.colors.push_back(makeColorMsg(color, info.layer.node_alpha));
  }

  return marker;
}

Marker makeDynamicEdgeMarkers(const std_msgs::Header& header,
                              const DynamicLayerInfo& info,
                              const DynamicSceneGraphLayer& layer,
                              const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::LINE_LIST;
  marker.ns = ns;
  marker.id = layer.prefix;

  marker.action = Marker::ADD;
  marker.scale.x = info.layer.edge_scale;
  fillPoseWithIdentity(marker.pose);

  for (const auto& [key, edge] : layer.edges()) {
    const auto& source_node = layer.getNode(edge.source);
    const auto& target_node = layer.getNode(edge.target);

    const auto c_source = info.edge_color(source_node, target_node, edge, true);
    const auto c_target = info.edge_color(source_node, target_node, edge, false);
    marker.colors.push_back(makeColorMsg(c_source, info.layer.edge_alpha));
    marker.colors.push_back(makeColorMsg(c_target, info.layer.edge_alpha));

    geometry_msgs::Point source;
    tf2::convert(layer.getNode(edge.source).attributes().position, source);
    source.z += info.getZOffset();
    marker.points.push_back(source);

    geometry_msgs::Point target;
    tf2::convert(layer.getNode(edge.target).attributes().position, target);
    target.z += info.getZOffset();
    marker.points.push_back(target);
  }

  return marker;
}

Marker makeDynamicLabelMarker(const std_msgs::Header& header,
                              const DynamicLayerInfo& info,
                              const DynamicSceneGraphLayer& layer,
                              const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.ns = ns;
  marker.id = layer.prefix;
  marker.action = Marker::ADD;
  marker.lifetime = ros::Duration(0);
  marker.scale.z = info.layer.label_scale;
  marker.color = makeColorMsg(Color());

  const auto& node = layer.getNodeByIndex(layer.numNodes() - 1);
  marker.text = info.node_label(node);
  if (marker.text.empty()) {
    marker.text = std::to_string(layer.id) + ":" + layer.prefix.str();
  }

  Eigen::Vector3d latest_position = node.attributes().position;
  fillPoseWithIdentity(marker.pose);
  tf2::convert(latest_position, marker.pose.position);
  marker.pose.position.z += info.getZOffset() + info.layer.label_height;
  return marker;
}

MarkerArray makeGraphEdgeMarkers(const std_msgs::Header& header,
                                 const GraphInfo& info,
                                 const DynamicSceneGraph& graph,
                                 const EdgeContainer::Edges& edges,
                                 const std::string& ns_prefix) {
  MarkerArray msg;
  std::map<LayerKey, size_t> marker_indices;
  std::map<LayerKey, size_t> num_since_last;
  for (const auto& [key, edge] : edges) {
    const auto& source = graph.getNode(edge.source);
    const auto source_layer = graph.getLayerForNode(edge.source).value();
    const auto& target = graph.getNode(edge.target);
    const auto target_layer = graph.getLayerForNode(edge.target).value();
    const auto edge_info = info.getEdgeInfo(source_layer, source, target_layer, target);
    if (!edge_info.visualize) {
      continue;
    }

    auto iter = marker_indices.find(source_layer);
    if (iter == marker_indices.end()) {
      iter = marker_indices.emplace(source_layer, msg.markers.size()).first;
      msg.markers.push_back(
          makeNewEdgeList(header, ns_prefix, source_layer, target_layer));
      msg.markers.back().scale.x = edge_info.scale;
      // make sure we always draw at least one edge
      num_since_last[source_layer] = edge_info.num_to_skip;
    }

    if (num_since_last[source.layer] >= edge_info.num_to_skip) {
      num_since_last[source.layer] = 0;
    } else {
      num_since_last[source.layer]++;
      continue;
    }

    auto& marker = msg.markers.at(iter->second);
    geometry_msgs::Point source_point;
    tf2::convert(source.attributes().position, source_point);
    source_point.z += edge_info.source_offset;
    marker.points.push_back(source_point);

    geometry_msgs::Point target_point;
    tf2::convert(target.attributes().position, target_point);
    target_point.z += edge_info.target_offset;
    marker.points.push_back(target_point);

    marker.colors.push_back(edge_info.color);
    marker.colors.push_back(edge_info.color);
  }

  return msg;
}

kimera_pgmo_msgs::KimeraPgmoMesh makeMeshMsg(const std_msgs::Header& header,
                                             const spark_dsg::Mesh& mesh,
                                             const std::string& ns,
                                             MeshColoring::Ptr coloring) {
  kimera_pgmo_msgs::KimeraPgmoMesh msg;
  msg.header = header;
  msg.ns = ns;

  // Setup default coloring (which is mesh color if available)
  if (!coloring && !mesh.has_colors) {
    UniformMeshColoring::Config config{spark_dsg::Color::gray()};
    coloring = std::make_shared<UniformMeshColoring>(config);
  }

  MeshColorAdaptor adaptor(mesh, coloring);
  msg.vertices.resize(mesh.points.size());
  msg.vertex_colors.resize(mesh.points.size());
  for (size_t i = 0; i < mesh.points.size(); ++i) {
    auto& vertex = msg.vertices[i];
    tf2::convert(mesh.points[i].cast<double>().eval(), vertex);
    auto& color = msg.vertex_colors[i];
    color = visualizer::makeColorMsg(adaptor.getVertexColor(i));
  }

  msg.triangles.resize(mesh.faces.size());
  for (size_t i = 0; i < mesh.faces.size(); ++i) {
    const auto& face = mesh.faces[i];
    auto& triangle = msg.triangles[i].vertex_indices;
    triangle[0] = face[0];
    triangle[1] = face[1];
    triangle[2] = face[2];
  }

  return msg;
}

}  // namespace hydra::visualizer
