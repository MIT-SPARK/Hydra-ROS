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
#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>
#include <spark_dsg/bounding_box.h>
#include <spark_dsg/color.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "hydra_visualizer/color/mesh_color_adaptor.h"
#include "hydra_visualizer/utils/visualizer_types.h"

namespace hydra::visualizer {

using spark_dsg::DynamicSceneGraph;
using spark_dsg::DynamicSceneGraphLayer;
using spark_dsg::EdgeContainer;
using spark_dsg::SceneGraphLayer;

void drawBoundingBox(const spark_dsg::BoundingBox& bbox,
                     const std_msgs::ColorRGBA& color,
                     visualization_msgs::Marker& marker);

visualization_msgs::MarkerArray makeLayerBoundingBoxes(const std_msgs::Header& header,
                                                       const StaticLayerInfo& info,
                                                       const SceneGraphLayer& layer,
                                                       const std::string& ns);

visualization_msgs::Marker makeLayerEllipseBoundaries(const std_msgs::Header& header,
                                                      const StaticLayerInfo& info,
                                                      const SceneGraphLayer& layer,
                                                      const std::string& ns);

visualization_msgs::Marker makeLayerPolygonEdges(const std_msgs::Header& header,
                                                 const StaticLayerInfo& info,
                                                 const SceneGraphLayer& layer,
                                                 const std::string& ns);

visualization_msgs::Marker makeLayerPolygonBoundaries(const std_msgs::Header& header,
                                                      const StaticLayerInfo& info,
                                                      const SceneGraphLayer& layer,
                                                      const std::string& ns);

visualization_msgs::MarkerArray makeEllipsoidMarkers(const std_msgs::Header& header,
                                                     const StaticLayerInfo& info,
                                                     const SceneGraphLayer& layer,
                                                     const std::string& ns);

visualization_msgs::MarkerArray makeLayerLabelMarkers(const std_msgs::Header& header,
                                                      const StaticLayerInfo& info,
                                                      const SceneGraphLayer& layer,
                                                      const std::string& ns);

visualization_msgs::Marker makeLayerNodeMarkers(const std_msgs::Header& header,
                                                const StaticLayerInfo& info,
                                                const SceneGraphLayer& layer,
                                                const std::string& ns);

visualization_msgs::Marker makeLayerEdgeMarkers(const std_msgs::Header& header,
                                                const StaticLayerInfo& config,
                                                const SceneGraphLayer& layer,
                                                const std::string& ns);

visualization_msgs::Marker makeMeshEdgesMarker(const std_msgs::Header& header,
                                               const StaticLayerInfo& info,
                                               const DynamicSceneGraph& graph,
                                               const SceneGraphLayer& layer,
                                               const std::string& ns);

visualization_msgs::Marker makeDynamicNodeMarkers(const std_msgs::Header& header,
                                                  const DynamicLayerInfo& info,
                                                  const DynamicSceneGraphLayer& layer,
                                                  const std::string& ns);

visualization_msgs::Marker makeDynamicEdgeMarkers(const std_msgs::Header& header,
                                                  const DynamicLayerInfo& info,
                                                  const DynamicSceneGraphLayer& layer,
                                                  const std::string& ns);

visualization_msgs::Marker makeDynamicLabelMarker(const std_msgs::Header& header,
                                                  const DynamicLayerInfo& info,
                                                  const DynamicSceneGraphLayer& layer,
                                                  const std::string& ns);

visualization_msgs::MarkerArray makeGraphEdgeMarkers(const std_msgs::Header& header,
                                                     const GraphInfo& info,
                                                     const DynamicSceneGraph& graph,
                                                     const EdgeContainer::Edges& edges,
                                                     const std::string& ns);

kimera_pgmo_msgs::KimeraPgmoMesh makeMeshMsg(const std_msgs::Header& header,
                                             const spark_dsg::Mesh& mesh,
                                             const std::string& ns,
                                             MeshColoring::Ptr coloring = nullptr);

}  // namespace hydra::visualizer
