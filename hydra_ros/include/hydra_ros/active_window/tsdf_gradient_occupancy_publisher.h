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
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/places/traversability_layer.h>
#include <ianvs/node_handle.h>
#include <spark_dsg/bounding_box.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/publisher.hpp>

#include <Eigen/Geometry>
#include <optional>

namespace hydra {

using Index2D = Eigen::Vector2i;

struct Index2DHash {
  inline static const auto s = Index2D(1, 1290);
  int operator()(const Index2D& index) const { return index.dot(s); }
};

template <typename ValueT>
using Index2DMap =
    std::unordered_map<Index2D,
                       ValueT,
                       Index2DHash,
                       std::equal_to<Index2D>,
                       Eigen::aligned_allocator<std::pair<const Index2D, ValueT>>>;

struct GradientInfo {
  float gradient = 0.0f;    // mean gradient magnitude
  float confidence = 0.0f;  // num_neighbors / 8.0
};

class TsdfGradientOccupancyPublisher : public ReconstructionModule::Sink {
 public:
  struct Config {
    // Base occupancy config fields
    std::string ns = "~/tsdf_gradient";
    bool collate = false;
    bool use_relative_height = true;
    double slice_height = -1.0;
    size_t num_slices = 20; // if if 10 cm, we want 2 m from -1 m to +1 m
    bool add_robot_footprint = false;
    Eigen::Vector3f footprint_min = Eigen::Vector3f::Zero();
    Eigen::Vector3f footprint_max = Eigen::Vector3f::Zero();

    // Gradient-specific parameters
    float gradient_threshold = 0.5f;  // m/m - max traversable gradient
    float min_weight = 1.0e-6f;       // min TSDF weight for observed voxel
    float min_confidence = 0.5f;      // min confidence (neighbors/8) for valid cell
    bool smoothing = true;            // apply box filter to reduce TSDF ripple
    bool probabilistic = false;       // continuous vs binary occupancy
  } const config;

  explicit TsdfGradientOccupancyPublisher(const Config& config);

  virtual ~TsdfGradientOccupancyPublisher() = default;

  std::string printInfo() const override;

  void call(uint64_t timestamp_ns,
            const VolumetricMap& map,
            const ActiveWindowOutput& output) const override;

 private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;

  // Helper functions
  std::optional<float> extractSurfaceHeight(const TsdfLayer& layer,
                                            const Index2D& global_2d,
                                            float min_z,
                                            float max_z) const;

  void buildHeightMap(const TsdfLayer& layer,
                      float min_z,
                      float max_z,
                      Index2DMap<float>& height_map) const;

  void computeGradientMap(const Index2DMap<float>& height_map,
                          float voxel_size,
                          Index2DMap<GradientInfo>& gradient_map) const;

  void fillOccupancyGrid(const Index2DMap<GradientInfo>& gradient_map,
                         const Eigen::Isometry3d& world_T_sensor,
                         const TsdfLayer& layer,
                         nav_msgs::msg::OccupancyGrid& msg) const;

  float computeHorizontalDistance(const Index2D& offset,
                                  float voxel_size) const;

  float computeTraversabilityFromGradient(float gradient) const;

  int8_t gradientToOccupancy(float gradient, float confidence) const;

  // 8-way neighbor offsets
  static const std::array<Index2D, 8> kNeighborOffsets;
};

void declare_config(TsdfGradientOccupancyPublisher::Config& config);

}  // namespace hydra
