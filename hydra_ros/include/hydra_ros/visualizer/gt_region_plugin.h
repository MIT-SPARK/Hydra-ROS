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
#pragma once
#include <config_utilities/factory.h>
#include <visualization_msgs/MarkerArray.h>

#include "hydra_ros/visualizer/dsg_visualizer_plugin.h"

namespace hydra {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

struct Region {
  Eigen::MatrixXd points;
  Eigen::Vector3d centroid;
  std::string name;
  std_msgs::ColorRGBA color;
};

class GtRegionPlugin : public DsgVisualizerPlugin {
 public:
  struct Config {
    std::string gt_regions_filepath = "";
    bool skip_unknown = true;
    bool draw_labels = false;
    bool fill_polygons = true;
    bool draw_polygon_boundaries = true;
    bool draw_polygon_vertices = true;
    double line_width = 0.05;
    double line_alpha = 0.8;
    double mesh_alpha = 0.6;
    double label_scale = 0.7;
    double label_offset = 0.0;
    double z_offset = 0.1;
    bool use_boundary_color = true;
  } const config;

  GtRegionPlugin(const Config& config,
                 const ros::NodeHandle& nh,
                 const std::string& name);

  virtual ~GtRegionPlugin();

  void draw(const ConfigManager& configs,
            const std_msgs::Header& header,
            const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 protected:
  std::optional<size_t> getFillMarker(const std_msgs::Header& header,
                                      visualization_msgs::MarkerArray& msg);

  std::optional<size_t> getBoundaryMarker(const std_msgs::Header& header,
                                          visualization_msgs::MarkerArray& msg);

  std::optional<size_t> getVertexMarker(const std_msgs::Header& header,
                                        visualization_msgs::MarkerArray& msg);

  void addLabelMarker(const std_msgs::Header& header,
                      const Region& region,
                      visualization_msgs::MarkerArray& msg);

  ros::Publisher pub_;
  std::vector<Region> regions_;
  std::map<std::string, size_t> published_labels_;
  int fill_index_ = -1;
  int boundary_index_ = -1;
  int vertex_index_ = -1;

  inline static const auto registration_ =
      config::RegistrationWithConfig<DsgVisualizerPlugin,
                                     GtRegionPlugin,
                                     GtRegionPlugin::Config,
                                     ros::NodeHandle,
                                     std::string>("GtRegionPlugin");
};

void declare_config(GtRegionPlugin::Config& config);

}  // namespace hydra
