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
#include <config_utilities/dynamic_config.h>
#include <ianvs/node_handle.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <string>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include "hydra_visualizer/layer_info.h"
#include "hydra_visualizer/utils/marker_tracker.h"

namespace hydra {

// NOTE(nathan) separate to make config wrapper easier to use
struct GraphRenderConfig {
  //! @brief Unit amount of distance between layers
  double layer_z_step = 5.0;  // [0, 50.0]
  //! @brief Whether or not to separate layers by adding z offsets
  bool collapse_layers = false;
};

void declare_config(GraphRenderConfig& config);

class SceneGraphRenderer {
 public:
  using Ptr = std::shared_ptr<SceneGraphRenderer>;
  using LayerConfigWrapper = config::DynamicConfig<visualizer::LayerConfig>;

  struct Config {
    //! @brief Overall graph config
    GraphRenderConfig graph;
    //! @brief Configuration for each layer
    std::map<spark_dsg::LayerId, visualizer::LayerConfig> layers;
    //! @brief Configuration for non-primary partitions by layer
    std::map<spark_dsg::LayerId, visualizer::LayerConfig> partitions;
  };

  explicit SceneGraphRenderer(const Config& config, ianvs::NodeHandle nh);

  virtual ~SceneGraphRenderer() = default;

  virtual void reset(const std_msgs::msg::Header& header);

  virtual void draw(const std_msgs::msg::Header& header,
                    const spark_dsg::DynamicSceneGraph& graph) const;

  virtual bool hasChange() const;

  virtual void clearChangeFlag();

 protected:
  virtual void setConfigs(const spark_dsg::DynamicSceneGraph& graph) const;

  virtual void drawInterlayerEdges(const std_msgs::msg::Header& header,
                                   const spark_dsg::DynamicSceneGraph& graph,
                                   visualization_msgs::msg::MarkerArray& msg) const;

  virtual void drawLayer(const std_msgs::msg::Header& header,
                         const visualizer::LayerInfo& info,
                         const spark_dsg::SceneGraphLayer& layer,
                         const spark_dsg::Mesh* mesh,
                         visualization_msgs::msg::MarkerArray& msg) const;

  const visualizer::LayerInfo& getLayerInfo(spark_dsg::LayerKey layer) const;

 protected:
  ianvs::NodeHandle nh_;
  config::DynamicConfig<GraphRenderConfig> graph_config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

  mutable std::atomic<bool> has_change_;
  mutable std::map<spark_dsg::LayerId, std::unique_ptr<LayerConfigWrapper>> layers_;
  mutable std::map<spark_dsg::LayerId, std::unique_ptr<LayerConfigWrapper>> partitions_;

  mutable MarkerTracker tracker_;
  mutable std::map<spark_dsg::LayerId, visualizer::LayerInfo> layer_infos_;
  mutable std::map<spark_dsg::LayerId, visualizer::LayerInfo> partition_infos_;
};

void declare_config(SceneGraphRenderer::Config& config);

}  // namespace hydra
