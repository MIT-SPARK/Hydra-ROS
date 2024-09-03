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
#include "hydra_ros/frontend/gvd_visualization_utilities.h"

#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/utils/visualizer_utilities.h>
#include <tf2_eigen/tf2_eigen.h>

#include <random>

#include "hydra_ros/visualizer/voxel_drawing.h"

namespace hydra {

using hydra_ros::GvdVisualizerConfig;
using places::GvdGraph;
using places::GvdLayer;
using places::GvdVoxel;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using visualizer::DiscreteColormap;
using visualizer::RangeColormap;

using EdgeMap = std::unordered_map<uint64_t, std::unordered_set<uint64_t>>;

namespace {

spark_dsg::Color getGvdColor(const GvdVisualizerConfig& config,
                             const RangeColormap& colors,
                             double distance,
                             uint8_t num_basis_points) {
  switch (static_cast<GvdVisualizationMode>(config.gvd_mode)) {
    case GvdVisualizationMode::BASIS_POINTS:
      return colors(static_cast<double>(num_basis_points),
                    static_cast<double>(config.min_num_basis),
                    static_cast<double>(config.max_num_basis));
    case GvdVisualizationMode::DISTANCE:
    case GvdVisualizationMode::DEFAULT:
    default:
      return colors(distance, config.gvd_min_distance, config.gvd_max_distance);
  }
}

size_t fillColors(const CompressedNodeMap& clusters,
                  std::map<uint64_t, size_t>& colors) {
  for (const auto& id_node_pair : clusters) {
    size_t max_color = 0;
    std::set<size_t> seen_colors;
    for (const auto sibling : id_node_pair.second.siblings) {
      const auto iter = colors.find(sibling);
      if (iter == colors.end()) {
        continue;
      }

      seen_colors.insert(iter->second);
      if (iter->second > max_color) {
        max_color = iter->second;
      }
    }

    if (seen_colors.empty()) {
      colors[id_node_pair.first] = 0;
      continue;
    }

    bool found_color = false;
    for (size_t i = 0; i < max_color; ++i) {
      if (!seen_colors.count(i)) {
        colors[id_node_pair.first] = i;
        found_color = true;
        break;
      }
    }

    if (found_color) {
      continue;
    }

    colors[id_node_pair.first] = max_color + 1;
  }

  size_t num_colors = 0;
  for (const auto& id_color_pair : colors) {
    if (id_color_pair.second > num_colors) {
      num_colors = id_color_pair.second;
    }
  }

  return num_colors + 1;
}

std::unordered_set<uint64_t>& getNodeSet(EdgeMap& edge_map, uint64_t node) {
  auto iter = edge_map.find(node);
  if (iter == edge_map.end()) {
    iter = edge_map.emplace(node, std::unordered_set<uint64_t>()).first;
  }
  return iter->second;
}

}  // namespace

Marker drawEsdf(const GvdVisualizerConfig& config,
                const RangeColormap& cmap,
                const Eigen::Isometry3d& pose,
                const GvdLayer& layer,
                const std::string& ns) {
  VoxelSliceConfig slice{config.slice_height, true};
  return drawVoxelSlice<GvdVoxel>(
      slice,
      std_msgs::Header(),
      layer,
      pose,
      [](const auto& voxel) { return voxel.observed; },
      [&](const auto& voxel) {
        return visualizer::makeColorMsg(
            cmap(voxel.distance, 0, config.esdf_distance),
            config.esdf_alpha);
      },
      ns);
}

Marker drawGvd(const GvdVisualizerConfig& config,
               const RangeColormap& colors,
               const GvdLayer& layer,
               const std::string& ns) {
  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size;
  marker.scale.y = layer.voxel_size;
  marker.scale.z = layer.voxel_size;

  for (const auto& block : layer) {
    for (size_t i = 0; i < block.numVoxels(); ++i) {
      const auto& voxel = block.getVoxel(i);
      if (!voxel.observed || voxel.num_extra_basis < config.basis_threshold) {
        continue;
      }

      const Eigen::Vector3d voxel_pos = block.getVoxelPosition(i).cast<double>();
      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      const auto color =
          getGvdColor(config, colors, voxel.distance, voxel.num_extra_basis);
      marker.colors.push_back(visualizer::makeColorMsg(color, config.gvd_alpha));
    }
  }

  return marker;
}

Marker drawGvdSurface(const GvdVisualizerConfig& config,
                      const RangeColormap& colors,
                      const GvdLayer& layer,
                      const std::string& ns) {
  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = ns;

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size;
  marker.scale.y = layer.voxel_size;
  marker.scale.z = layer.voxel_size;

  for (const auto& block : layer) {
    for (size_t i = 0; i < block.numVoxels(); ++i) {
      const auto& voxel = block.getVoxel(i);
      if (!voxel.on_surface) {
        continue;
      }

      Eigen::Vector3d voxel_pos = block.getVoxelPosition(i).cast<double>();
      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      const auto dist = voxel.distance;
      const auto color = colors(dist, -0.4, 0.4);
      marker.colors.push_back(visualizer::makeColorMsg(color, config.gvd_alpha));
    }
  }

  return marker;
}

Marker drawGvdError(const GvdVisualizerConfig& config,
                    const RangeColormap& colors,
                    const GvdLayer& lhs,
                    const GvdLayer& rhs,
                    double threshold) {
  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "error_locations";

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = lhs.voxel_size;
  marker.scale.y = lhs.voxel_size;
  marker.scale.z = lhs.voxel_size;

  for (const auto& lhs_block : lhs) {
    const auto& rhs_block = rhs.getBlock(lhs_block.index);

    for (size_t i = 0; i < lhs_block.numVoxels(); ++i) {
      const auto& lvoxel = lhs_block.getVoxel(i);
      const auto& rvoxel = rhs_block.getVoxel(i);

      if (!lvoxel.observed || !rvoxel.observed) {
        continue;
      }

      const double error = std::abs(lvoxel.distance - rvoxel.distance);
      if (error <= threshold) {
        continue;
      }

      const Eigen::Vector3d voxel_pos = lhs_block.getVoxelPosition(i).cast<double>();
      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      const auto color = colors(error, 0.0, 10.0);
      marker.colors.push_back(visualizer::makeColorMsg(color, config.gvd_alpha));
    }
  }

  return marker;
}

MarkerArray drawGvdGraph(const GvdGraph& graph,
                         const GvdVisualizerConfig& config,
                         const RangeColormap& colors,
                         const std::string& ns,
                         size_t marker_id) {
  MarkerArray marker;
  if (graph.empty()) {
    return marker;
  }

  const Eigen::Vector3d p_identity = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
  {  // scope to make handling stuff a little easier
    Marker nodes;
    nodes.type = Marker::SPHERE_LIST;
    nodes.id = marker_id;
    nodes.ns = ns + "_nodes";
    nodes.action = Marker::ADD;
    nodes.scale.x = config.gvd_graph_scale;
    nodes.scale.y = config.gvd_graph_scale;
    nodes.scale.z = config.gvd_graph_scale;
    tf2::convert(p_identity, nodes.pose.position);
    tf2::convert(q_identity, nodes.pose.orientation);
    marker.markers.push_back(nodes);
  }

  {  // scope to make handling stuff a little easier
    Marker edges;
    edges.type = Marker::LINE_LIST;
    edges.id = marker_id;
    edges.ns = ns + "_edges";
    edges.action = Marker::ADD;
    edges.scale.x = config.gvd_graph_scale;
    tf2::convert(p_identity, edges.pose.position);
    tf2::convert(q_identity, edges.pose.orientation);
    marker.markers.push_back(edges);
  }

  auto& nodes = marker.markers[0];
  auto& edges = marker.markers[1];

  EdgeMap seen_edges;
  for (const auto& id_node_pair : graph.nodes()) {
    geometry_msgs::Point node_centroid;
    tf2::convert(id_node_pair.second.position, node_centroid);
    nodes.points.push_back(node_centroid);
    const auto color = getGvdColor(config,
                                   colors,
                                   id_node_pair.second.distance,
                                   id_node_pair.second.num_basis_points);
    nodes.colors.push_back(visualizer::makeColorMsg(color, config.gvd_alpha));

    auto& curr_seen = getNodeSet(seen_edges, id_node_pair.first);
    for (const auto sibling : id_node_pair.second.siblings) {
      if (curr_seen.count(sibling)) {
        continue;
      }

      curr_seen.insert(sibling);
      getNodeSet(seen_edges, sibling).insert(id_node_pair.first);

      edges.points.push_back(nodes.points.back());
      edges.colors.push_back(nodes.colors.back());

      const auto& other = *graph.getNode(sibling);
      geometry_msgs::Point neighbor_centroid;
      tf2::convert(other.position, neighbor_centroid);
      edges.points.push_back(neighbor_centroid);
      const auto sibling_color =
          getGvdColor(config, colors, other.distance, other.num_basis_points);
      edges.colors.push_back(visualizer::makeColorMsg(sibling_color, config.gvd_alpha));
    }
  }

  return marker;
}

MarkerArray drawGvdClusters(const GvdGraph& graph,
                            const CompressedNodeMap& clusters,
                            const std::unordered_map<uint64_t, uint64_t>& remapping,
                            const GvdVisualizerConfig& config,
                            const std::string& ns,
                            const DiscreteColormap& colormap,
                            size_t marker_id) {
  MarkerArray marker;
  if (graph.empty()) {
    return marker;
  }

  const Eigen::Vector3d p_identity = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
  {  // scope to make handling stuff a little easier
    Marker nodes;
    nodes.type = Marker::SPHERE_LIST;
    nodes.id = marker_id;
    nodes.ns = ns + "_nodes";
    nodes.action = Marker::ADD;
    nodes.scale.x = config.gvd_graph_scale;
    nodes.scale.y = config.gvd_graph_scale;
    nodes.scale.z = config.gvd_graph_scale;
    tf2::convert(p_identity, nodes.pose.position);
    tf2::convert(q_identity, nodes.pose.orientation);
    marker.markers.push_back(nodes);
  }

  {  // scope to make handling stuff a little easier
    Marker edges;
    edges.type = Marker::LINE_LIST;
    edges.id = marker_id;
    edges.ns = ns + "_edges";
    edges.action = Marker::ADD;
    edges.scale.x = config.gvd_graph_scale;
    tf2::convert(p_identity, edges.pose.position);
    tf2::convert(q_identity, edges.pose.orientation);
    marker.markers.push_back(edges);
  }

  auto& nodes = marker.markers[0];
  auto& edges = marker.markers[1];

  std::map<uint64_t, size_t> color_mapping;
  const size_t num_colors = fillColors(clusters, color_mapping);
  std::vector<std_msgs::ColorRGBA> colors;
  for (size_t i = 0; i < num_colors; ++i) {
    colors.push_back(visualizer::makeColorMsg(colormap(i), config.gvd_alpha));
  }

  EdgeMap seen_edges;
  for (const auto& id_node_pair : graph.nodes()) {
    geometry_msgs::Point node_centroid;
    tf2::convert(id_node_pair.second.position, node_centroid);
    nodes.points.push_back(node_centroid);
    if (remapping.count(id_node_pair.first)) {
      const auto cluster_id = remapping.at(id_node_pair.first);
      const auto& cluster_color = colors.at(color_mapping.at(cluster_id));
      nodes.colors.push_back(cluster_color);
    } else {
      nodes.colors.push_back(
          visualizer::makeColorMsg(Color(0, 0, 0), config.gvd_alpha));
    }

    auto& curr_seen = getNodeSet(seen_edges, id_node_pair.first);
    for (const auto sibling : id_node_pair.second.siblings) {
      if (curr_seen.count(sibling)) {
        continue;
      }

      curr_seen.insert(sibling);
      getNodeSet(seen_edges, sibling).insert(id_node_pair.first);

      edges.points.push_back(nodes.points.back());
      edges.colors.push_back(nodes.colors.back());

      const auto& other = *graph.getNode(sibling);
      geometry_msgs::Point neighbor_centroid;
      tf2::convert(other.position, neighbor_centroid);
      edges.points.push_back(neighbor_centroid);

      if (remapping.count(sibling)) {
        const auto& neighbor_cluster = remapping.at(sibling);
        const auto& neighbor_color = colors.at(color_mapping.at(neighbor_cluster));
        edges.colors.push_back(neighbor_color);
      } else {
        edges.colors.push_back(
            visualizer::makeColorMsg(Color(0, 0, 0), config.gvd_alpha));
      }
    }
  }

  return marker;
}

MarkerArray drawPlaceFreespace(const std_msgs::Header& header,
                               const SceneGraphLayer& layer,
                               const std::string& ns,
                               const Color& color) {
  MarkerArray spheres;
  size_t id = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    const auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();

    Marker marker;
    marker.header = header;
    marker.type = Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.ns = ns;

    marker.scale.x = 2 * attrs.distance;
    marker.scale.y = 2 * attrs.distance;
    marker.scale.z = 2 * attrs.distance;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    tf2::convert(id_node_pair.second->attributes().position, marker.pose.position);

    marker.color = visualizer::makeColorMsg(color);
    spheres.markers.push_back(marker);
    ++id;
  }

  return spheres;
}

}  // namespace hydra
