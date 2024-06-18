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
#include <dynamic_reconfigure/server.h>
#include <hydra/frontend/gvd_place_extractor.h>
#include <hydra/places/gvd_voxel.h>
#include <hydra_ros/GvdVisualizerConfig.h>

#include "hydra_ros/visualizer/visualizer_types.h"

namespace hydra {
namespace places {
class GraphExtractorInterface;
class GvdGraph;
}  // namespace places

class MarkerGroupPub;

using hydra_ros::GvdVisualizerConfig;

class PlacesVisualizer : public GvdPlaceExtractor::Sink {
 public:
  struct Config {
    std::string ns = "~places";
    std::string place_marker_ns = "raw_places_graph";
    bool show_block_outlines = false;
    bool use_gvd_block_outlines = false;
    double outline_scale = 0.01;
    ColormapConfig colormap;
    GvdVisualizerConfig gvd;
    VisualizerConfig graph;
    LayerConfig graph_layer;
  };

  explicit PlacesVisualizer(const Config& config);

  virtual ~PlacesVisualizer();

  std::string printInfo() const override;

  void call(uint64_t timestamp_ns,
            const Eigen::Isometry3f& world_T_body,
            const places::GvdLayer& gvd,
            const places::GraphExtractorInterface* extractor) const override;

  void visualizeError(uint64_t timestamp_ns,
                      const places::GvdLayer& lhs,
                      const places::GvdLayer& rhs,
                      double threshold);

 private:
  void visualizeGraph(const std_msgs::Header& header,
                      const SceneGraphLayer& graph) const;

  void visualizeGvd(const std_msgs::Header& header,
                    const places::GvdLayer& gvd) const;

  void visualizeGvdGraph(const std_msgs::Header& header,
                         const places::GvdGraph& gvd_graph) const;

  void visualizeBlocks(const std_msgs::Header& header,
                       const places::GvdLayer& gvd) const;

  void publishGraphLabels(const std_msgs::Header& header,
                          const SceneGraphLayer& graph) const;

  void publishFreespace(const std_msgs::Header& header,
                        const SceneGraphLayer& graph) const;

  void gvdConfigCb(GvdVisualizerConfig& config, uint32_t level);

  void graphConfigCb(LayerConfig& config, uint32_t level);

  void colormapCb(ColormapConfig& config, uint32_t level);

  void setupConfigServers();

  template <typename Config, typename Callback>
  void startRqtServer(const std::string& config_ns,
                      std::unique_ptr<dynamic_reconfigure::Server<Config>>& server,
                      const Callback& callback) {
    ros::NodeHandle config_nh(nh_, config_ns);
    server.reset(new dynamic_reconfigure::Server<Config>(config_nh));
    server->setCallback(boost::bind(callback, this, _1, _2));
  }

 protected:
  Config config_;
  ros::NodeHandle nh_;
  std::unique_ptr<MarkerGroupPub> pubs_;

  mutable std::set<int> previous_labels_;
  mutable size_t previous_spheres_;
  mutable bool published_gvd_graph_;
  mutable bool published_gvd_clusters_;

  std::unique_ptr<dynamic_reconfigure::Server<GvdVisualizerConfig>> gvd_config_server_;
  std::unique_ptr<dynamic_reconfigure::Server<LayerConfig>> graph_config_server_;
  std::unique_ptr<dynamic_reconfigure::Server<ColormapConfig>> colormap_server_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<GvdPlaceExtractor::Sink, PlacesVisualizer, Config>(
          "PlacesVisualizer");
};

void declare_config(PlacesVisualizer::Config& config);

}  // namespace hydra
