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
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/polygon_utilities.h"
#include "hydra_ros/visualizer/visualizer_utilities.h"

namespace hydra {

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
    : DsgVisualizerPlugin(nh, name), config(config::checkValid(config)) {
  // namespacing gives us a reasonable topic
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("", 1, true);

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

GtRegionPlugin::~GtRegionPlugin() {}

std::optional<size_t> GtRegionPlugin::getFillMarker(const std_msgs::Header& header,
                                                    MarkerArray& msg) {
  if (!config.fill_polygons) {
    return std::nullopt;
  }

  if (fill_index_ < 0) {
    fill_index_ = msg.markers.size();
    auto& marker = msg.markers.emplace_back();
    marker.header = header;
    marker.ns = "gt_region_polygon_fill";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.color.a = config.mesh_alpha;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    published_labels_[marker.ns] = marker.id;
  }

  return fill_index_;
}

std::optional<size_t> GtRegionPlugin::getBoundaryMarker(const std_msgs::Header& header,
                                                        MarkerArray& msg) {
  if (!config.draw_polygon_boundaries) {
    return std::nullopt;
  }

  if (boundary_index_ < 0) {
    boundary_index_ = msg.markers.size();
    auto& marker = msg.markers.emplace_back();
    marker.header = header;
    marker.ns = "gt_region_polygon_boundaries";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = config.line_width;
    published_labels_[marker.ns] = marker.id;
  }

  return boundary_index_;
}

std::optional<size_t> GtRegionPlugin::getVertexMarker(const std_msgs::Header& header,
                                                      MarkerArray& msg) {
  if (!config.draw_polygon_vertices) {
    return std::nullopt;
  }

  if (vertex_index_ < 0) {
    vertex_index_ = msg.markers.size();
    auto& marker = msg.markers.emplace_back();
    marker.header = header;
    marker.ns = "gt_region_polygon_vertices";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = config.line_width;
    marker.scale.y = config.line_width;
    marker.scale.z = config.line_width;
    published_labels_[marker.ns] = marker.id;
  }

  return vertex_index_;
}

void GtRegionPlugin::addLabelMarker(const std_msgs::Header& header,
                                    const Region& region,
                                    MarkerArray& msg) {
  const std::string ns = "gt_region_labels";
  auto iter = published_labels_.find(ns);
  if (iter == published_labels_.end()) {
    iter = published_labels_.emplace(ns, 0).first;
  } else {
    iter->second++;
  }

  auto& marker = msg.markers.emplace_back();
  marker.header = header;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.ns = ns;
  marker.id = iter->second;
  marker.action = visualization_msgs::Marker::ADD;
  marker.text = region.name;
  marker.scale.z = config.label_scale;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1.0;
  tf2::convert(region.centroid, marker.pose.position);
  marker.pose.position.z += config.label_offset;
}

void GtRegionPlugin::draw(const ConfigManager&,
                          const std_msgs::Header& header,
                          const DynamicSceneGraph&) {
  if (!published_labels_.empty()) {
    return;
  }

  fill_index_ = -1;
  boundary_index_ = -1;
  vertex_index_ = -1;
  visualization_msgs::MarkerArray msg;

  for (const auto& region : regions_) {
    auto f_index = getFillMarker(header, msg);
    if (f_index) {
      auto color = region.color;
      color.a = config.mesh_alpha;
      makeFilledPolygon(region.points, color, msg.markers.at(*f_index));
    }

    auto b_index = getBoundaryMarker(header, msg);
    auto v_index = getVertexMarker(header, msg);
    if (b_index) {
      auto color = region.color;
      color.a = config.line_alpha;
      if (!config.use_boundary_color) {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 0.0;
      }

      makePolygonBoundary(region.points,
                          color,
                          msg.markers.at(*b_index),
                          std::nullopt,
                          v_index ? &msg.markers.at(*v_index) : nullptr);
    }

    if (config.draw_labels) {
      addLabelMarker(header, region, msg);
    }
  }

  pub_.publish(msg);
}

void GtRegionPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  visualization_msgs::MarkerArray msg;
  for (auto&& [ns, max_id] : published_labels_) {
    for (size_t i = 0; i <= max_id; ++i) {
      msg.markers.push_back(makeDeleteMarker(header, i, ns));
    }
  }

  published_labels_.clear();
  pub_.publish(msg);
}

}  // namespace hydra
