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
#include <cv_bridge/cv_bridge.h>
#include <hydra/common/global_info.h>
#include <sensor_msgs/Image.h>
#include <tf2_eigen/tf2_eigen.h>

#include "hydra_ros/visualizer/draw_voxel_slice.h"

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
  field(config.colormap, "colormap");
  field(config.label_colormap, "label_colormap");
}

struct ImageGroupPub {
  using Callback = std::function<sensor_msgs::Image::ConstPtr()>;
  explicit ImageGroupPub(ros::NodeHandle& nh) : nh_(nh), transport_(nh_) {}

  void publish(const std::string& name, const Callback& callback) {
    auto iter = pubs_.find(name);
    if (iter == pubs_.end()) {
      iter = pubs_.emplace(name, transport_.advertise(name, 1)).first;
    }

    if (!iter->second.getNumSubscribers()) {
      return;  // don't do work if no subscription
    }

    iter->second.publish(callback());
  }

  ros::NodeHandle nh_;
  image_transport::ImageTransport transport_;
  std::map<std::string, image_transport::Publisher> pubs_;
};

ReconstructionVisualizer::ReconstructionVisualizer(const Config& config)
    : config(config),
      nh_(config.ns),
      pubs_(nh_),
      image_pubs_(std::make_unique<ImageGroupPub>(nh_)),
      colormap_(config.colormap) {}

ReconstructionVisualizer::~ReconstructionVisualizer() {}

std::string ReconstructionVisualizer::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config);
  return ss.str();
}

void ReconstructionVisualizer::call(uint64_t timestamp_ns,
                                    const Eigen::Isometry3d& pose,
                                    const TsdfLayer& tsdf,
                                    const ReconstructionOutput& output) const {
  const auto& info = GlobalInfo::instance();
  const auto truncation_distance = info.getMapConfig().truncation_distance;
  std_msgs::Header header;
  header.frame_id = info.getFrames().map;
  header.stamp.fromNSec(timestamp_ns);

  const RangeColormap cmap(
      {config::VirtualConfig<ContinuousPalette>{DivergentPalette::Config()}});

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

  publishLabelImage(output);
}

void ReconstructionVisualizer::publishLabelImage(
    const ReconstructionOutput& output) const {
  if (!output.sensor_data) {
    return;
  }

  const auto topic = output.sensor_data->getSensor().name + "/labels";
  image_pubs_->publish(topic, [&]() {
    const auto& labels = output.sensor_data->label_image;
    cv_bridge::CvImagePtr msg(new cv_bridge::CvImage());
    msg->encoding = "rgb8";
    msg->image = cv::Mat(labels.rows, labels.cols, CV_8UC3);
    for (int r = 0; r < labels.rows; ++r) {
      for (int c = 0; c < labels.cols; ++c) {
        auto pixel = msg->image.ptr<uint8_t>(r, c);
        const auto label = labels.at<int>(r, c);
        const auto color = label_colormap_(label);
        *pixel = color.r;
        *(pixel + 1) = color.g;
        *(pixel + 2) = color.b;
      }
    }
    return msg->toImageMsg();
  });
}

}  // namespace hydra
