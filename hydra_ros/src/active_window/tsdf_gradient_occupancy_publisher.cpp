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
#include "hydra_ros/active_window/tsdf_gradient_occupancy_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>

#include <cmath>

namespace hydra {

namespace {

// 8-way neighbor offsets (cardinal + diagonal)
const std::array<Index2D, 8> kNeighborOffsetsArray = {{
    {0, -1},   // bottom
    {-1, 0},   // left
    {0, 1},    // top
    {1, 0},    // right
    {-1, -1},  // bottom-left
    {-1, 1},   // top-left
    {1, 1},    // top-right
    {1, -1}    // bottom-right
}};

}  // namespace

const std::array<Index2D, 8>
    TsdfGradientOccupancyPublisher::kNeighborOffsets = kNeighborOffsetsArray;

void declare_config(TsdfGradientOccupancyPublisher::Config& config) {
  using namespace config;
  name("TsdfGradientOccupancyPublisher::Config");
  field(config.ns, "ns");
  field(config.collate, "collate");
  field(config.use_relative_height, "use_relative_height");
  field(config.slice_height, "slice_height", "m");
  field(config.num_slices, "num_slices");
  field(config.add_robot_footprint, "add_robot_footprint");
  field(config.footprint_min, "footprint_min");
  field(config.footprint_max, "footprint_max");
  field(config.gradient_threshold, "gradient_threshold");
  field(config.min_weight, "min_weight");
  field(config.min_confidence, "min_confidence");
  field(config.smoothing, "smoothing");
  field(config.probabilistic, "probabilistic");

  checkCondition(config.gradient_threshold > 0.0f,
                 "gradient_threshold must be positive");
  checkInRange(config.min_confidence, 0.0f, 1.0f, "min_confidence");
}

TsdfGradientOccupancyPublisher::TsdfGradientOccupancyPublisher(const Config& config)
    : config(config::checkValid(config)),
      pub_(ianvs::NodeHandle::this_node(config.ns)
               .create_publisher<nav_msgs::msg::OccupancyGrid>(
                   "occupancy",
                   rclcpp::QoS(1).transient_local())) {}

std::string TsdfGradientOccupancyPublisher::printInfo() const {
  return config::toString(config);
}

void TsdfGradientOccupancyPublisher::call(uint64_t timestamp_ns,
                                          const VolumetricMap& map,
                                          const ActiveWindowOutput& output) const {
  if (!pub_->get_subscription_count()) {
    return;
  }

  const auto& tsdf_layer = map.getTsdfLayer();
  const auto& world_T_body = output.world_T_body();

  // Compute vertical range
  double base_z = config.slice_height;
  if (config.use_relative_height) {
    base_z += world_T_body.translation().z();
  }

  const float voxel_size = tsdf_layer.voxel_size;
  const float min_z = static_cast<float>(base_z);
  const float max_z = static_cast<float>(base_z + config.num_slices * voxel_size);

  // Build height map (Pass 1)
  Index2DMap<float> height_map;
  buildHeightMap(tsdf_layer, min_z, max_z, height_map);

  // Compute gradient map (Pass 2)
  Index2DMap<GradientInfo> gradient_map;
  computeGradientMap(height_map, voxel_size, gradient_map);

  // Fill and publish occupancy grid (Pass 3)
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = GlobalInfo::instance().getFrames().map;
  msg.header.stamp = rclcpp::Time(timestamp_ns);
  msg.info.map_load_time = msg.header.stamp;

  fillOccupancyGrid(gradient_map, world_T_body, tsdf_layer, msg);
  pub_->publish(msg);
}

std::optional<float> TsdfGradientOccupancyPublisher::extractSurfaceHeight(
    const TsdfLayer& layer,
    const Index2D& global_2d,
    float min_z,
    float max_z) const {
  const float voxel_size = layer.voxel_size;
  const int vps = static_cast<int>(layer.voxels_per_side);

  // Convert global 2D index to block coordinates
  const int block_x = global_2d.x() / vps;
  const int block_y = global_2d.y() / vps;
  const int local_x = global_2d.x() % vps;
  const int local_y = global_2d.y() % vps;

  // Get vertical range in voxel coordinates
  const auto min_key = layer.getVoxelKey(spatial_hash::Point(0, 0, min_z));
  const auto max_key = layer.getVoxelKey(spatial_hash::Point(0, 0, max_z));

  // Scan from top to bottom to find highest surface
  for (int block_z = max_key.first.z(); block_z >= min_key.first.z(); --block_z) {
    const auto tsdf_block = layer.getBlockPtr(BlockIndex(block_x, block_y, block_z));
    if (!tsdf_block) {
      continue;
    }

    const int min_voxel_z = block_z == min_key.first.z() ? min_key.second.z() : 0;
    const int max_voxel_z =
        block_z == max_key.first.z() ? max_key.second.z() : vps - 1;

    for (int z = max_voxel_z; z >= min_voxel_z; --z) {
      const auto& voxel = tsdf_block->getVoxel(VoxelIndex(local_x, local_y, z));

      if (voxel.weight < config.min_weight) {
        continue;
      }

      if (voxel.distance < voxel_size) {
        const VoxelKey key(BlockIndex(block_x, block_y, block_z),
                           VoxelIndex(local_x, local_y, z));
        return layer.getVoxelPosition(key).z();
      }
    }
  }

  return std::nullopt;
}

void TsdfGradientOccupancyPublisher::buildHeightMap(
    const TsdfLayer& layer,
    float min_z,
    float max_z,
    Index2DMap<float>& height_map) const {
  const int vps = static_cast<int>(layer.voxels_per_side);

  // Iterate over all allocated TSDF blocks
  for (const auto& block : layer) {
    // For each 2D column in the block
    for (int local_x = 0; local_x < vps; ++local_x) {
      for (int local_y = 0; local_y < vps; ++local_y) {
        // Compute global 2D index
        const Index2D global_2d(block.index.x() * vps + local_x,
                                        block.index.y() * vps + local_y);

        // Extract surface height for this column
        auto surface_height = extractSurfaceHeight(layer, global_2d, min_z, max_z);
        if (surface_height) {
          height_map[global_2d] = *surface_height;
        }
      }
    }
  }
}

void TsdfGradientOccupancyPublisher::computeGradientMap(
    const Index2DMap<float>& height_map,
    float voxel_size,
    Index2DMap<GradientInfo>& gradient_map) const {
  // Optional smoothing pass
  Index2DMap<float> smoothed_height_map;
  if (config.smoothing) {
    smoothed_height_map.reserve(height_map.size());
    for (const auto& [center_idx, center_height] : height_map) {
      float height_sum = center_height;
      int count = 1;

      for (const auto& offset : kNeighborOffsets) {
        const Index2D neighbor_idx(center_idx.x() + offset.x(),
                                           center_idx.y() + offset.y());
        auto it = height_map.find(neighbor_idx);
        if (it != height_map.end()) {
          height_sum += it->second;
          count++;
        }
      }

      smoothed_height_map[center_idx] = height_sum / count;
    }
  }

  const auto& grad_height_map = config.smoothing ? smoothed_height_map : height_map;

  // Compute gradients
  gradient_map.reserve(grad_height_map.size());
  for (const auto& [center_idx, center_height] : grad_height_map) {
    float gradient_sum = 0.0f;
    int num_neighbors_observed = 0;

    for (const auto& offset : kNeighborOffsets) {
      const Index2D neighbor_idx(center_idx.x() + offset.x(),
                                         center_idx.y() + offset.y());

      auto neighbor_it = grad_height_map.find(neighbor_idx);
      if (neighbor_it == grad_height_map.end()) {
        continue;
      }

      num_neighbors_observed++;

      const float height_diff = std::abs(neighbor_it->second - center_height);
      const float horiz_dist = computeHorizontalDistance(offset, voxel_size);
      gradient_sum += height_diff / horiz_dist;
    }

    GradientInfo info;
    info.gradient =
        num_neighbors_observed > 0 ? gradient_sum / num_neighbors_observed : 0.0f;
    info.confidence = num_neighbors_observed / 8.0f;
    gradient_map[center_idx] = info;
  }
}

void TsdfGradientOccupancyPublisher::fillOccupancyGrid(
    const Index2DMap<GradientInfo>& gradient_map,
    const Eigen::Isometry3d& world_T_sensor,
    const TsdfLayer& layer,
    nav_msgs::msg::OccupancyGrid& msg) const {
  if (gradient_map.empty()) {
    return;
  }

  // Compute bounds
  Eigen::Vector2f x_min = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector2f x_max =
      Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest());

  const float voxel_size = layer.voxel_size;
  for (const auto& [idx, _] : gradient_map) {
    const Eigen::Vector2f pos = idx.cast<float>() * voxel_size;
    x_min = x_min.array().min(pos.array());
    x_max = x_max.array().max(pos.array());
  }

  // Add one voxel margin
  x_min -= Eigen::Vector2f::Constant(voxel_size);
  x_max += Eigen::Vector2f::Constant(voxel_size);

  const Eigen::Vector2f dims = (x_max - x_min) / voxel_size;

  // Initialize grid
  msg.info.resolution = voxel_size;
  msg.info.width = std::ceil(dims.x());
  msg.info.height = std::ceil(dims.y());
  msg.info.origin.position.x = x_min.x();
  msg.info.origin.position.y = x_min.y();
  msg.info.origin.position.z = config.slice_height;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(msg.info.width * msg.info.height, -1);

  // Setup robot footprint if needed
  spark_dsg::BoundingBox bbox;
  Eigen::Isometry3f sensor_T_world;
  if (config.add_robot_footprint) {
    bbox = spark_dsg::BoundingBox(config.footprint_min, config.footprint_max);
    sensor_T_world = world_T_sensor.inverse().cast<float>();
  }

  // Fill occupancy grid
  for (const auto& [global_idx, gradient_info] : gradient_map) {
    const Eigen::Vector2f pos = global_idx.cast<float>() * voxel_size;
    const Eigen::Vector2f rel_pos = pos - x_min;

    const auto r = std::floor(rel_pos.y() / voxel_size);
    const auto c = std::floor(rel_pos.x() / voxel_size);
    const size_t index = r * msg.info.width + c;

    if (index >= msg.data.size()) {
      continue;
    }

    // Check robot footprint
    if (config.add_robot_footprint) {
      const Eigen::Vector3f pos_3d(pos.x(), pos.y(), 0.0f);
      if (bbox.contains((sensor_T_world * pos_3d).eval())) {
        msg.data[index] = 0;
        continue;
      }
    }

    // Map gradient to occupancy
    msg.data[index] =
        gradientToOccupancy(gradient_info.gradient, gradient_info.confidence);
  }
}

float TsdfGradientOccupancyPublisher::computeHorizontalDistance(
    const Index2D& offset,
    float voxel_size) const {
  // Diagonal: sqrt(2) * voxel_size
  if (offset.x() != 0 && offset.y() != 0) {
    return voxel_size * std::sqrt(2.0f);
  }

  // Cardinal: voxel_size
  return voxel_size;
}

float TsdfGradientOccupancyPublisher::computeTraversabilityFromGradient(
    float gradient) const {
  if (gradient >= config.gradient_threshold) {
    return 0.0f;  // Intraversable
  }

  // Linear interpolation: 1.0 at gradient=0, 0.0 at gradient=threshold
  return 1.0f - (gradient / config.gradient_threshold);
}

int8_t TsdfGradientOccupancyPublisher::gradientToOccupancy(float gradient,
                                                           float confidence) const {
  // Check confidence threshold
  if (confidence < config.min_confidence) {
    return -1;  // Unknown
  }

  if (config.probabilistic) {
    // Continuous mode: map traversability to occupancy
    const float traversability = computeTraversabilityFromGradient(gradient);
    const float occupancy_float = (1.0f - traversability) * 100.0f;
    return static_cast<int8_t>(std::clamp(occupancy_float, 0.0f, 100.0f));
  } else {
    // Binary mode: threshold-based
    return gradient >= config.gradient_threshold ? 100 : 0;
  }
}

namespace {

static const auto registration_ =
    config::RegistrationWithConfig<ReconstructionModule::Sink,
                                   TsdfGradientOccupancyPublisher,
                                   TsdfGradientOccupancyPublisher::Config>(
        "TsdfGradientOccupancyPublisher");

}  // namespace

}  // namespace hydra
