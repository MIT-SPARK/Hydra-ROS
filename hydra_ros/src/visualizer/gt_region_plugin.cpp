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
#include "hydra_ros/visualizer/gt_region_plugin.h"

#include <config_utilities/config.h>
#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/polygon_utilities.h"

namespace hydra {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

void declare_config(GtRegionPlugin::Config& config) {
  using namespace config;
  name("GtRegionPlugin::Config");
  field(config.gt_regions_filepath, "gt_regions_filepath");
  field(config.skip_unknown, "skip_unknown");
  field(config.draw_labels, "draw_labels");
  field(config.fill_polygons, "fill_polygons");
  field(config.draw_polygon_boundaries, "draw_polygon_boundaries");
  field(config.draw_polygon_vertices, "draw_polygon_vertices");
  field(config.line_width, "line_width");
  field(config.line_alpha, "line_alpha");
  field(config.mesh_alpha, "mesh_alpha");
  field(config.label_scale, "label_scale");
  field(config.label_offset, "label_offset");
  field(config.z_offset, "z_offset");
  field(config.use_boundary_color, "use_boundary_color");
}

GtRegionPlugin::GtRegionPlugin(const Config& config,
                               const ros::NodeHandle& nh,
                               const std::string& name)
    : DsgVisualizerPlugin(nh, name),
      config(config::checkValid(config)),
      published_(false),
      pub_(nh_.advertise<MarkerArray>("", 1, true)) {
  std::filesystem::path region_path(config.gt_regions_filepath);
  if (!std::filesystem::exists(region_path)) {
    LOG_IF(INFO, !config.gt_regions_filepath.empty())
        << "Provided region path: '" << region_path.string() << "' does not exist";
    return;
  }

  const auto region_config = YAML::LoadFile(region_path.string());
  for (const auto& region : region_config) {
    const auto x_pos = region["boundary_x"].as<std::vector<double>>();
    const auto y_pos = region["boundary_y"].as<std::vector<double>>();
    const auto z_pos = region["boundary_z"].as<std::vector<double>>();
    if (x_pos.size() != y_pos.size() || x_pos.size() != z_pos.size()) {
      LOG(ERROR) << "Invalid region found!";
      continue;
    }

    const auto centroid = region["centroid"].as<std::vector<double>>();
    if (centroid.size() != 3) {
      LOG(ERROR) << "Invalid region found!";
      continue;
    }

    const auto color = region["color"].as<std::vector<int>>();
    if (color.size() != 3) {
      LOG(ERROR) << "Invalid region found!";
      continue;
    }

    const auto name = region["name"].as<std::string>();
    const auto result = name.find("unknown");
    if (config.skip_unknown && result != std::string::npos) {
      continue;
    }

    auto& new_region = regions_.emplace_back();
    new_region.points = Eigen::MatrixXd(3, x_pos.size());
    for (size_t i = 0; i < x_pos.size(); ++i) {
      new_region.points(0, i) = x_pos.at(i);
      new_region.points(1, i) = y_pos.at(i);
      new_region.points(2, i) = z_pos.at(i) + config.z_offset;
    }

    new_region.centroid << centroid.at(0), centroid.at(1), centroid.at(2);
    new_region.name = new_region.color.r = color.at(0) / 255.0;
    new_region.color.g = color.at(1) / 255.0;
    new_region.color.b = color.at(2) / 255.0;
  }
}

size_t getTotalMarkers(const GtRegionPlugin::Config& config) {
  size_t total = 0;
  total += config.fill_polygons ? 1 : 0;
  total += config.draw_polygon_boundaries ? 1 : 0;
  total += config.draw_polygon_vertices ? 1 : 0;
  return total;
}

Marker* setupFillMarker(const GtRegionPlugin::Config& config,
                        const std_msgs::Header& header,
                        MarkerArray& msg) {
  if (!config.fill_polygons) {
    return nullptr;
  }

  auto& marker = msg.markers[0];
  marker.header = header;
  marker.ns = "gt_region_polygon_fill";
  marker.id = 0;
  marker.type = Marker::TRIANGLE_LIST;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.a = config.mesh_alpha;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  return &marker;
}

Marker* setupBoundaryMarker(const GtRegionPlugin::Config& config,
                            const std_msgs::Header& header,
                            MarkerArray& msg) {
  if (!config.draw_polygon_boundaries) {
    return nullptr;
  }

  const auto index = config.fill_polygons ? 0 : 1;
  auto& marker = msg.markers[index];
  marker.header = header;
  marker.ns = "gt_region_polygon_boundaries";
  marker.id = 0;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config.line_width;
  return &marker;
}

Marker* setupVertexMarker(const GtRegionPlugin::Config& config,
                          const std_msgs::Header& header,
                          MarkerArray& msg) {
  if (!config.draw_polygon_vertices) {
    return nullptr;
  }

  auto& marker = msg.markers[getTotalMarkers(config) - 1];
  marker.header = header;
  marker.ns = "gt_region_polygon_vertices";
  marker.id = 0;
  marker.type = Marker::SPHERE_LIST;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config.line_width;
  marker.scale.y = config.line_width;
  marker.scale.z = config.line_width;
  return &marker;
}

void addLabelMarker(const GtRegionPlugin::Config& config,
                    const std_msgs::Header& header,
                    const Region& region,
                    MarkerArray& msg) {
  const std::string ns = "gt_region_labels";

  size_t index = msg.markers.size();
  auto& marker = msg.markers.emplace_back();
  marker.header = header;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.ns = ns;
  marker.id = index;
  marker.action = Marker::ADD;
  marker.text = region.name;
  marker.scale.z = config.label_scale;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1.0;
  tf2::convert(region.centroid, marker.pose.position);
  marker.pose.position.z += config.label_offset;
}

void GtRegionPlugin::draw(const std_msgs::Header& header,
                          const spark_dsg::DynamicSceneGraph&) {
  if (published_) {
    return;
  }

  MarkerArray markers;
  markers.markers.resize(getTotalMarkers(config));
  auto fill_marker = setupFillMarker(config, header, markers);
  auto boundary_marker = setupBoundaryMarker(config, header, markers);
  auto vertex_marker = setupVertexMarker(config, header, markers);

  for (const auto& region : regions_) {
    if (fill_marker) {
      auto color = region.color;
      color.a = config.mesh_alpha;
      makeFilledPolygon(region.points, color, *fill_marker);
    }

    if (boundary_marker) {
      auto color = region.color;
      color.a = config.line_alpha;
      if (!config.use_boundary_color) {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 0.0;
      }

      makePolygonBoundary(
          region.points, color, *boundary_marker, std::nullopt, vertex_marker);
    }

    if (config.draw_labels) {
      addLabelMarker(config, header, region, markers);
    }
  }

  MarkerArray msg;
  tracker_.add(markers, msg);
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }
  published_ = true;
}

void GtRegionPlugin::reset(const std_msgs::Header& header,
                           const spark_dsg::DynamicSceneGraph&) {
  MarkerArray msg;
  tracker_.clearPrevious(header, msg);
  if (!msg.markers.empty()) {
    pub_.publish(msg);
  }

  published_ = false;
}

}  // namespace hydra
