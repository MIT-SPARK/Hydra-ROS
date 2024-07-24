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
#include <hydra/places/compression_graph_extractor.h>
#include <hydra/places/gvd_graph.h>
#include <hydra/places/gvd_voxel.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "hydra_ros/ColormapConfig.h"
#include "hydra_ros/GvdVisualizerConfig.h"

namespace hydra {

using CompressedNodeMap = std::unordered_map<uint64_t, places::CompressedNode>;

enum class GvdVisualizationMode : int {
  DEFAULT = hydra_ros::GvdVisualizer_DEFAULT,
  DISTANCE = hydra_ros::GvdVisualizer_DISTANCE,
  BASIS_POINTS = hydra_ros::GvdVisualizer_BASIS_POINTS,
};

GvdVisualizationMode getModeFromString(const std::string& mode);

visualization_msgs::Marker makeEsdfMarker(const hydra_ros::GvdVisualizerConfig& config,
                                          const hydra_ros::ColormapConfig& colors,
                                          const places::GvdLayer& layer,
                                          const std::string& ns);

visualization_msgs::Marker makeGvdMarker(const hydra_ros::GvdVisualizerConfig& config,
                                         const hydra_ros::ColormapConfig& colors,
                                         const places::GvdLayer& layer,
                                         const std::string& ns);

visualization_msgs::Marker makeSurfaceVoxelMarker(
    const hydra_ros::GvdVisualizerConfig& config,
    const hydra_ros::ColormapConfig& colors,
    const places::GvdLayer& layer,
    const std::string& ns);

visualization_msgs::Marker makeErrorMarker(const hydra_ros::GvdVisualizerConfig& config,
                                           const hydra_ros::ColormapConfig& colors,
                                           const places::GvdLayer& lhs,
                                           const places::GvdLayer& rhs,
                                           double threshold);

visualization_msgs::Marker makeBlocksMarker(const TsdfLayer& layer,
                                            double scale,
                                            const std::string& ns);

visualization_msgs::Marker makeBlocksMarker(const places::GvdLayer& layer,
                                            double scale,
                                            const std::string& ns);

visualization_msgs::MarkerArray makeGvdGraphMarkers(
    const places::GvdGraph& graph,
    const hydra_ros::GvdVisualizerConfig& config,
    const hydra_ros::ColormapConfig& colors,
    const std::string& ns,
    size_t marker_id = 0);

visualization_msgs::MarkerArray showGvdClusters(
    const places::GvdGraph& graph,
    const CompressedNodeMap& clusters,
    const std::unordered_map<uint64_t, uint64_t>& remapping,
    const hydra_ros::GvdVisualizerConfig& config,
    const hydra_ros::ColormapConfig& colors,
    const std::string& ns,
    size_t marker_id = 0);

visualization_msgs::MarkerArray makePlaceSpheres(const std_msgs::Header& header,
                                                 const SceneGraphLayer& layer,
                                                 const std::string& ns,
                                                 double alpha);

}  // namespace hydra
