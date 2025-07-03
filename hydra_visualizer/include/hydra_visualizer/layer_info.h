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
#include <config_utilities/virtual_config.h>
#include <spark_dsg/color.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include "hydra_visualizer/adapters/graph_color.h"
#include "hydra_visualizer/adapters/text.h"

namespace hydra::visualizer {

struct LayerConfig {
  //! @brief show layer
  bool visualize = false;
  //! @brief number of steps of offset to apply
  double z_offset_scale = 0.0;  // [-5.0, 10.0]
  //! @brief Draw current frontiers as ellipses
  bool draw_frontier_ellipse = false;
  //! @brief whether or not to draw mesh edges
  bool draw_mesh_edges = false;

  //! @brief Node settings
  struct Nodes {
    //! @brief whether or not to draw nodes
    bool draw = true;
    //! @brief size of the centroid marker
    double scale = 0.1;
    //! @brief Color adapter
    config::VirtualConfig<GraphColorAdapter> color{NodeColorAdapter::Config()};
    //! @brief alpha of the centroid marker
    double alpha = 1.0;
    //! @brief use sphere markers (instead of cubes)
    bool use_sphere = false;
  } nodes;

  //! @brief Edge configuration
  struct Edges {
    //! @brief draw intralayer edges
    bool draw = true;
    //! @brief intralayer edge size
    double scale = 0.03;  //[ 0.001, 1.0]
    //! @brief intralayer edge alpha
    double alpha = 1.0;  //[ 0.0, 1.0]
    //! @brief show intralayer edge in color
    bool use_color = true;
    //! @brief draw interlayer edges
    bool draw_interlayer = true;
    //! @brief use edge source layer for config
    bool interlayer_use_source = true;
    //! @brief interlayer edge size
    double interlayer_scale = 0.03;  // [0.001, 1.0]
    //! @brief interlayer edge alpha
    double interlayer_alpha = 1.0;  // [0.0, 1.0]
    //! @brief show interlayer edge in color
    bool interlayer_use_color = true;
    //! @brief Number of edges to skip when drawing interlayer edges
    size_t interlayer_insertion_skip = 0;  // [0, 1000]
  } edges;

  //! @brief Text settings
  struct Text {
    //! @brief add text for nodes
    bool draw = false;
    //! @brief add single text marker for layer
    bool draw_layer = false;
    //! @brief draw text without z offset
    bool collapse = false;
    //! @brief text adapter type
    config::VirtualConfig<GraphTextAdapter> adapter{IdTextAdapter::Config()};
    //! @brief height of text above node
    double height = 1.0;  //[ 0.0, 5.0]
    //! @brief scale of text above node
    double scale = 0.5;  //[ 0.05, 5.0]
    //! @brief add random noise to text z offset
    bool add_jitter = false;
    //! @brief amount of jitter to add
    double jitter_scale = 0.2;  //[ 0.05, 5.0]
  } text;

  struct BoundingBoxes {
    //! @brief display bounding box
    bool draw = false;
    //! @brief draw bounding box at ground level
    bool collapse = false;
    //! @brief scale of bounding box wireframe
    double scale = 0.1;  // [0.001, 1.0]
    //! @brief scale of edges drawn to bbox corners
    double edge_scale = 0.01;  // [ 0.001, 1.0]
    //! @brief alpha of bounding boxes
    double alpha = 0.5;  // [0.0, 1.0]
    //! @brief point at which to break the edge into many edges
    double edge_break_ratio = 0.5;  //[0.0, 1.0]
  } bounding_boxes;

  //! @brief configuration for polygon boundaries
  struct Boundaries {
    //! @brief display polygon boundaries
    bool draw = false;
    //! @brief draw polygons at mesh level
    bool collapse = false;
    //! @brief scale of boundary wireframe
    double wireframe_scale = 0.1;  //[ 0.001, 1.0]
    //! @brief draw polygons using node semantic color
    bool use_node_color = true;
    //! @brief alpha of boundary
    double alpha = 0.5;  //[0.0, 1.0]
    //! @brief display minimum bounding ellipse
    bool draw_ellipse = false;
    //! @brief alpha of bounding ellipse
    double ellipse_alpha = 0.5;  //[ 0.0, 1.0]
  } boundaries;
};

void declare_config(LayerConfig& config);

class LayerInfo {
 public:
  using FilterFunction = std::function<bool(const spark_dsg::SceneGraphNode&)>;
  using ColorFunction =
      std::function<spark_dsg::Color(const spark_dsg::SceneGraphNode&)>;
  using TextFunction = std::function<std::string(const spark_dsg::SceneGraphNode&)>;

  LayerInfo(const LayerConfig config);
  LayerInfo& offset(double offset_size = 1.0, bool collapse = true);
  LayerInfo& graph(const spark_dsg::DynamicSceneGraph& graph, spark_dsg::LayerId layer);

  bool shouldVisualize(const spark_dsg::SceneGraphNode& node) const;

  const LayerConfig config;

  double z_offset;
  ColorFunction node_color;
  TextFunction node_text;
  mutable FilterFunction filter;

 private:
  std::unique_ptr<GraphColorAdapter> color_adapter_;
  std::unique_ptr<GraphTextAdapter> text_adapter_;
};

}  // namespace hydra::visualizer
