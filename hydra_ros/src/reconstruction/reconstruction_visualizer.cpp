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
#include "hydra_ros/reconstruction/reconstruction_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <hydra/common/global_info.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra_ros/visualizer/colormap_utilities.h"
#include "hydra_ros/visualizer/gvd_visualization_utilities.h"

namespace hydra {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using VizConfig = ReconstructionVisualizer::Config;

using ColorFunction =
    std::function<std_msgs::ColorRGBA(const VizConfig&, const TsdfVoxel&)>;

std_msgs::ColorRGBA colorVoxelByDist(const VizConfig& config, const TsdfVoxel& voxel) {
  double ratio =
      dsg_utils::computeRatio(config.min_distance, config.max_distance, voxel.distance);
  auto color = dsg_utils::interpolateColorMap(config.colors, ratio);
  return dsg_utils::makeColorMsg(color, config.marker_alpha);
}

std_msgs::ColorRGBA colorVoxelByWeight(const VizConfig& config,
                                       const TsdfVoxel& voxel) {
  // TODO(nathan) consider exponential
  double ratio =
      dsg_utils::computeRatio(config.min_weight, config.max_weight, voxel.weight);
  auto color = dsg_utils::interpolateColorMap(config.colors, ratio);
  return dsg_utils::makeColorMsg(color, config.marker_alpha);
}

// adapted from khronos
Marker makeTsdfMarker(const VizConfig& config,
                      const std_msgs::Header& header,
                      const TsdfLayer& layer,
                      const Eigen::Isometry3d& world_T_sensor,
                      const ColorFunction& color_func,
                      const std::string& ns) {
  Marker msg;
  msg.header = header;
  msg.action = visualization_msgs::Marker::ADD;
  msg.id = 0;
  msg.ns = ns;
  msg.type = visualization_msgs::Marker::CUBE_LIST;
  msg.scale.x = layer.voxel_size;
  msg.scale.y = layer.voxel_size;
  msg.scale.z = layer.voxel_size;

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, msg.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), msg.pose.orientation);

  auto height = config.slice_height;
  if (config.use_relative_height) {
    height += world_T_sensor.translation().z();
  }

  const Point slice_pos(0, 0, height);
  const auto slice_index = layer.getBlockIndex(slice_pos);
  const auto origin =
      spatial_hash::originPointFromIndex(slice_index, layer.blockSize());
  const auto grid_index = spatial_hash::indexFromPoint<VoxelIndex>(
      slice_pos - origin, layer.voxel_size_inv);

  for (const auto& block : layer) {
    if (block.index.z() != slice_index.z()) {
      continue;
    }

    for (size_t x = 0; x < block.voxels_per_side; ++x) {
      for (size_t y = 0; y < block.voxels_per_side; ++y) {
        const VoxelIndex voxel_index(x, y, grid_index.z());
        const auto& voxel = block.getVoxel(voxel_index);
        if (voxel.weight < config.min_observation_weight) {
          continue;
        }

        const Eigen::Vector3d pos =
            block.getVoxelPosition(voxel_index).cast<double>();
        geometry_msgs::Point marker_pos;
        tf2::convert(pos, marker_pos);
        msg.points.push_back(marker_pos);
        msg.colors.push_back(color_func(config, voxel));
      }
    }
  }

  return msg;
}

void declare_config(ReconstructionVisualizer::Config& config) {
  using namespace config;
  name("ReconstructionVisualizerConfig");
  field(config.ns, "ns");
  field(config.min_weight, "min_weight");
  field(config.max_weight, "max_weight");
  field(config.min_distance, "min_distance", "m");
  field(config.max_distance, "max_distance", "m");
  field(config.marker_alpha, "marker_alpha");
  field(config.use_relative_height, "use_relative_height");
  field(config.slice_height, "slice_height", "m");
  field(config.min_observation_weight, "min_observation_weight");
  field(config.colors.min_hue, "min_hue");
  field(config.colors.max_hue, "max_hue");
  field(config.colors.min_luminance, "min_luminance");
  field(config.colors.max_luminance, "max_luminance");
  field(config.colors.min_saturation, "min_saturation");
  field(config.colors.max_saturation, "max_saturation");
}

ReconstructionVisualizer::ReconstructionVisualizer(const Config& config)
    : config_(config), nh_(config.ns) {
  pubs_.reset(new MarkerGroupPub(nh_));
}

ReconstructionVisualizer::~ReconstructionVisualizer() {}

std::string ReconstructionVisualizer::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config_);
  return ss.str();
}

void ReconstructionVisualizer::call(uint64_t timestamp_ns,
                                    const Eigen::Isometry3d& world_T_sensor,
                                    const TsdfLayer& tsdf,
                                    const ReconstructionOutput&) const {
  std_msgs::Header header;
  header.frame_id = GlobalInfo::instance().getFrames().map;
  header.stamp.fromNSec(timestamp_ns);

  pubs_->publish("tsdf_viz", [&](Marker& msg) {
    msg = makeTsdfMarker(
        config_, header, tsdf, world_T_sensor, colorVoxelByDist, "tsdf_distance_slice");

    if (msg.points.size()) {
      return true;
    } else {
      LOG(INFO) << "visualizing empty TSDF slice";
      return false;
    }
  });

  pubs_->publish("tsdf_weight_viz", [&](Marker& msg) {
    msg = makeTsdfMarker(
        config_, header, tsdf, world_T_sensor, colorVoxelByWeight, "tsdf_weight_slice");

    if (msg.points.size()) {
      return true;
    } else {
      return false;
    }
  });
}

}  // namespace hydra
