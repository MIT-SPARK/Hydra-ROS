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
#include "hydra_ros/active_window/reconstruction_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <hydra/common/global_info.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <hydra_visualizer/utils/visualizer_utilities.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra_ros/utils/input_data_to_messages.h"
#include "hydra_ros/utils/node_handle_factory.h"
#include "hydra_ros/visualizer/voxel_drawing.h"

namespace hydra {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using visualizer::ContinuousPalette;
using visualizer::DivergentPalette;
using visualizer::RangeColormap;

namespace {

bool isVoxelObserved(const ReconstructionVisualizer::Config& config,
                     const TsdfVoxel& voxel) {
  return voxel.weight >= config.min_observation_weight;
}

std_msgs::ColorRGBA colorVoxelByDist(const ReconstructionVisualizer::Config& config,
                                     double truncation_distance,
                                     const visualizer::RangeColormap& cmap,
                                     const TsdfVoxel& voxel) {
  auto color = cmap(voxel.distance, -truncation_distance, truncation_distance);
  return visualizer::makeColorMsg(color, config.marker_alpha);
}

std_msgs::ColorRGBA colorVoxelByWeight(const ReconstructionVisualizer::Config& config,
                                       const visualizer::RangeColormap& cmap,
                                       const TsdfVoxel& voxel) {
  // TODO(nathan) consider exponential
  auto color = cmap(voxel.weight, config.min_weight, config.max_weight);
  return visualizer::makeColorMsg(color, config.marker_alpha);
}

}  // namespace

void declare_config(ReconstructionVisualizer::Config& config) {
  using namespace config;
  name("ReconstructionVisualizerConfig");
  field(config.ns, "ns");
  field(config.min_weight, "min_weight");
  field(config.max_weight, "max_weight");
  field(config.marker_alpha, "marker_alpha");
  field(config.use_relative_height, "use_relative_height");
  field(config.slice_height, "slice_height", "m");
  field(config.min_observation_weight, "min_observation_weight");
  field(config.tsdf_block_scale, "tsdf_block_scale");
  field(config.tsdf_block_color, "tsdf_block_color");
  field(config.tsdf_block_alpha, "tsdf_block_alpha");
  field(config.mesh_block_scale, "mesh_block_scale");
  field(config.mesh_block_alpha, "mesh_block_alpha");
  field(config.mesh_block_color, "mesh_block_color");
  field(config.point_size, "point_size");
  field(config.filter_points_by_range, "filter_points_by_range");
  field(config.colormap, "colormap");
  field(config.label_colormap, "label_colormap");
  config.mesh_coloring.setOptional();
  field(config.mesh_coloring, "mesh_coloring");
}

ImagePublisherGroup::ImagePublisherGroup(ros::NodeHandle& nh) : transport_(nh) {}

bool ImagePublisherGroup::shouldPublish(const image_transport::Publisher& pub) const {
  return pub.getNumSubscribers() > 0;
}

image_transport::Publisher ImagePublisherGroup::makePublisher(
    const std::string& topic) const {
  return transport_.advertise(topic, 1);
}

void ImagePublisherGroup::publishMsg(const image_transport::Publisher& pub,
                                     const sensor_msgs::Image::Ptr& img) const {
  pub.publish(img);
}

ReconstructionVisualizer::ReconstructionVisualizer(const Config& config)
    : config(config),
      nh_(NodeHandleFactory::getNodeHandle(config.ns)),
      pubs_(nh_),
      active_mesh_pub_(nh_.advertise<kimera_pgmo_msgs::KimeraPgmoMesh>("mesh", 1)),
      image_pubs_(nh_),
      cloud_pubs_(nh_),
      colormap_(config.colormap),
      mesh_coloring_(config.mesh_coloring.create()) {}

ReconstructionVisualizer::~ReconstructionVisualizer() {}

std::string ReconstructionVisualizer::printInfo() const {
  return config::toString(config);
}

void ReconstructionVisualizer::call(uint64_t timestamp_ns,
                                    const VolumetricMap& map,
                                    const ActiveWindowOutput& output) const {
  const auto truncation_distance = map.config.truncation_distance;
  const auto& tsdf = map.getTsdfLayer();
  const auto pose = output.world_T_body();

  std_msgs::Header header;
  header.frame_id = GlobalInfo::instance().getFrames().map;
  header.stamp.fromNSec(timestamp_ns);

  const RangeColormap cmap(RangeColormap::Config{});
  const VoxelSliceConfig slice{config.slice_height, config.use_relative_height};
  const Filter<TsdfVoxel> filter = [&](const auto& voxel) {
    return isVoxelObserved(config, voxel);
  };

  const auto distance_colormap = [&](const auto& voxel) {
    return colorVoxelByDist(config, truncation_distance, cmap, voxel);
  };
  const auto weight_colormap = [&](const auto& voxel) {
    return colorVoxelByWeight(config, colormap_, voxel);
  };

  pubs_.publish("tsdf_viz", header, [&]() -> Marker {
    return drawVoxelSlice<TsdfVoxel>(
        slice, header, tsdf, pose, filter, distance_colormap, "distances");
  });

  pubs_.publish("tsdf_weight_viz", header, [&]() -> Marker {
    return drawVoxelSlice<TsdfVoxel>(
        slice, header, tsdf, pose, filter, weight_colormap, "weights");
  });

  ActiveBlockColoring block_cmap(config.tsdf_block_color);
  pubs_.publish("tsdf_block_viz", header, [&]() -> Marker {
    return drawSpatialGrid(tsdf,
                           config.tsdf_block_scale,
                           "blocks",
                           config.tsdf_block_alpha,
                           block_cmap.getCallback<TsdfBlock>());
  });

  publishMesh(output);

  if (output.sensor_data) {
    std_msgs::Header header;
    header.stamp.fromNSec(output.timestamp_ns);
    header.frame_id = GlobalInfo::instance().getFrames().map;

    const auto sensor_name = output.sensor_data->getSensor().name;
    image_pubs_.publish(sensor_name + "/labels", [&]() {
      return makeImage(header, *output.sensor_data, [this](uint32_t label) {
        return label_colormap_(label);
      });
    });
    cloud_pubs_.publish(sensor_name + "/pointcloud", [&]() {
      return makeCloud(header, *output.sensor_data, config.filter_points_by_range);
    });
  }
}

void ReconstructionVisualizer::publishMesh(const ActiveWindowOutput& out) const {
  std_msgs::Header header;
  header.stamp.fromNSec(out.timestamp_ns);
  header.frame_id = GlobalInfo::instance().getFrames().map;

  const auto& mesh = out.map().getMeshLayer();
  ActiveBlockColoring block_cmap(config.mesh_block_color);
  pubs_.publish("mesh_block_viz", header, [&]() -> Marker {
    return drawSpatialGrid(mesh,
                           config.mesh_block_scale,
                           "mesh_blocks",
                           config.mesh_block_alpha,
                           block_cmap.getCallback<MeshBlock>());
  });

  if (active_mesh_pub_.getNumSubscribers() == 0) {
    return;
  }

  if (mesh.numBlocks() == 0) {
    return;
  }

  auto iter = mesh.begin();
  auto combined_mesh = iter->clone();
  ++iter;
  while (iter != mesh.end()) {
    *combined_mesh += *iter;
    ++iter;
  }

  auto msg = visualizer::makeMeshMsg(
      header, *combined_mesh, "active_window_mesh", mesh_coloring_);
  active_mesh_pub_.publish(msg);
}

}  // namespace hydra
