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
#include <spark_dsg/color.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <std_msgs/ColorRGBA.h>

#include "hydra_visualizer/DynamicLayerVisualizerConfig.h"
#include "hydra_visualizer/LayerVisualizerConfig.h"
#include "hydra_visualizer/VisualizerConfig.h"

namespace hydra::visualizer {

using FilterFunction = std::function<bool(const spark_dsg::SceneGraphNode&)>;

using NodeColorFunction =
    std::function<spark_dsg::Color(const spark_dsg::SceneGraphNode&)>;

using NodeLabelFunction = std::function<std::string(const spark_dsg::SceneGraphNode&)>;

using EdgeColorFunction =
    std::function<spark_dsg::Color(const spark_dsg::SceneGraphNode&,
                                   const spark_dsg::SceneGraphNode&,
                                   const spark_dsg::SceneGraphEdge&,
                                   bool)>;

struct DefaultNodeColorFunction {
  spark_dsg::Color operator()(const spark_dsg::SceneGraphNode& node) const;
};

struct DefaultNodeLabelFunction {
  std::string operator()(const spark_dsg::SceneGraphNode& node) const;
};

struct DefaultEdgeColorFunction {
  spark_dsg::Color operator()(const spark_dsg::SceneGraphNode& source,
                              const spark_dsg::SceneGraphNode& target,
                              const spark_dsg::SceneGraphEdge& edge,
                              bool is_source) const;
};

template <typename ConfigT>
struct LayerInfo {
  hydra_visualizer::VisualizerConfig graph;
  ConfigT layer;
  NodeColorFunction node_color = DefaultNodeColorFunction();
  NodeLabelFunction node_label = DefaultNodeLabelFunction();
  EdgeColorFunction edge_color = DefaultEdgeColorFunction();
  mutable FilterFunction filter = {};

  double getZOffset() const;
  bool shouldVisualize(const spark_dsg::SceneGraphNode& node) const;
};

using StaticLayerInfo = LayerInfo<hydra_visualizer::LayerVisualizerConfig>;
using DynamicLayerInfo = LayerInfo<hydra_visualizer::DynamicLayerVisualizerConfig>;

struct GraphInfo {
  struct EdgeInformation {
    bool visualize = false;
    size_t num_to_skip;
    double scale;
    std_msgs::ColorRGBA color;
    double source_offset;
    double target_offset;
  };

  EdgeInformation getEdgeInfo(const spark_dsg::LayerKey& source_layer,
                              const spark_dsg::SceneGraphNode& source,
                              const spark_dsg::LayerKey& target_layer,
                              const spark_dsg::SceneGraphNode& target) const;

  std::map<spark_dsg::LayerId, StaticLayerInfo> layers;
  std::map<spark_dsg::LayerId, DynamicLayerInfo> dynamic_layers;
};

}  // namespace hydra::visualizer
